/* ============================================================================
 *  infusion.c - Control de infusion por volumen y tiempo (ver infusion.h)
 * ==========================================================================*/
#include "infusion.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_log.h"

static const char *TAG = "INFUSION";

/* ---------------------------------------------------------------------------
 *  Secuencia de bobinas  (columnas: IN1 IN2 IN3 IN4)
 *  IN1/IN2 = bobina A, IN3/IN4 = bobina B. Misma secuencia de main.c original.
 * -------------------------------------------------------------------------*/
#if MOTOR_MEDIO_PASO
#define N_ESTADOS            8
#define SUBPASOS_POR_VUELTA  (MOTOR_PASOS_POR_VUELTA * 2)
static const uint8_t SECUENCIA[N_ESTADOS][4] = {
    {1, 0, 1, 0}, {0, 0, 1, 0}, {0, 1, 1, 0}, {0, 1, 0, 0},
    {0, 1, 0, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}, {1, 0, 0, 0},
};
#else
#define N_ESTADOS            4
#define SUBPASOS_POR_VUELTA  MOTOR_PASOS_POR_VUELTA
static const uint8_t SECUENCIA[N_ESTADOS][4] = {
    {1, 0, 1, 0}, {0, 1, 1, 0}, {0, 1, 0, 1}, {1, 0, 0, 1},
};
#endif

/* [CAMBIO] Se quito el protocolo '#I' / '#P': la pantalla ahora la maneja
 * nextion_bomba.c y main.c llama a infusion_iniciar / pausar / detener.     */

/* ---------------------------------------------------------------------------
 *  Estado interno
 * -------------------------------------------------------------------------*/
static gpio_num_t        s_pin[4];
static gptimer_handle_t  s_timer;
static TaskHandle_t      s_tarea;
static SemaphoreHandle_t s_mutex;

static volatile bool     s_activa;            /* el ISR esta dando pasos       */
static volatile uint32_t s_pasos_restantes;
static volatile uint32_t s_pasos_dados;
static volatile int      s_estado;            /* indice en SECUENCIA           */
static bool              s_timer_corriendo;
static infusion_plan_t   s_plan;
static volatile bool     s_pausada;           /* [CAMBIO] en pausa             */
static volatile bool     s_completada;        /* [CAMBIO] dio todos los pasos  */

/* ======================================================================== */

static inline void escribir_estado(int e)
{
    gpio_set_level(s_pin[0], SECUENCIA[e][0]);
    gpio_set_level(s_pin[1], SECUENCIA[e][1]);
    gpio_set_level(s_pin[2], SECUENCIA[e][2]);
    gpio_set_level(s_pin[3], SECUENCIA[e][3]);
}

static void liberar_bobinas(void)
{
    for (int i = 0; i < 4; i++) gpio_set_level(s_pin[i], 0);
}

/* Interrupcion del gptimer: se ejecuta cada periodo_us y da UN paso. */
static bool IRAM_ATTR on_alarma(gptimer_handle_t timer,
                                const gptimer_alarm_event_data_t *ev, void *arg)
{
    if (!s_activa) return false;

    s_estado = (s_estado + MOTOR_SENTIDO_INFUSION + N_ESTADOS) % N_ESTADOS;
    escribir_estado(s_estado);
    s_pasos_dados++;

    if (--s_pasos_restantes == 0) {
        s_activa = false;                         /* avisar a la tarea       */
        BaseType_t despertar = pdFALSE;
        vTaskNotifyGiveFromISR(s_tarea, &despertar);
        return despertar == pdTRUE;
    }
    return false;
}

/* [CAMBIO] Se quitaron nextion_cmd, nextion_progreso y nextion_estado: lo que
 * se muestra en la pantalla lo manda main.c con nextion_bomba.c             */

/* ======================================================================== */

esp_err_t infusion_calcular(float volumen_ml, float tiempo_s, infusion_plan_t *plan)
{
    if (plan == NULL || volumen_ml <= 0.0f || tiempo_s <= 0.0f) {
        ESP_LOGW(TAG, "Volumen y tiempo deben ser mayores que 0");
        return ESP_ERR_INVALID_ARG;
    }
    if (volumen_ml > JERINGA_VOLUMEN_MAX_ML) {
        ESP_LOGW(TAG, "%.2f mL supera la capacidad de la jeringa (%.1f mL)",
                 volumen_ml, JERINGA_VOLUMEN_MAX_ML);
        return ESP_ERR_INVALID_ARG;
    }

    const float radio_mm  = JERINGA_DIAMETRO_INTERNO_MM / 2.0f;
    const float area_mm2  = (float)M_PI * radio_mm * radio_mm;
    const float avance_mm = volumen_ml * 1000.0f / area_mm2;
    const float vueltas   = avance_mm / AVANCE_POR_VUELTA_MM;
    const uint32_t pasos  = (uint32_t)lroundf(vueltas * SUBPASOS_POR_VUELTA);

    if (pasos == 0) {
        ESP_LOGW(TAG, "Volumen demasiado pequeno: equivale a menos de un paso");
        return ESP_ERR_INVALID_SIZE;
    }

    const float pasos_por_seg = (float)pasos / tiempo_s;
    if (pasos_por_seg > MOTOR_MAX_PASOS_POR_SEG) {
        ESP_LOGW(TAG, "Muy rapido: %.0f pasos/s (max %d). Tiempo minimo: %.1f s",
                 pasos_por_seg, MOTOR_MAX_PASOS_POR_SEG,
                 (float)pasos / MOTOR_MAX_PASOS_POR_SEG);
        return ESP_ERR_INVALID_SIZE;
    }

    plan->volumen_ml    = volumen_ml;
    plan->tiempo_s      = tiempo_s;
    plan->pasos_totales = pasos;
    plan->periodo_us    = (uint64_t)llround((double)tiempo_s * 1e6 / pasos);
    plan->pasos_por_seg = pasos_por_seg;
    plan->caudal_ml_h   = volumen_ml * 3600.0f / tiempo_s;
    plan->rpm           = pasos_por_seg / SUBPASOS_POR_VUELTA * 60.0f;          /* [CAMBIO] */
    plan->mm_s          = pasos_por_seg / SUBPASOS_POR_VUELTA * AVANCE_POR_VUELTA_MM; /* [CAMBIO] */
    return ESP_OK;
}

/* Detiene el temporizador y suelta el motor. Llamar con s_mutex tomado. */
static void detener_interno(void)
{
    s_activa = false;
    if (s_timer_corriendo) {
        gptimer_stop(s_timer);
        s_timer_corriendo = false;
    }
    liberar_bobinas();
}

esp_err_t infusion_iniciar(float volumen_ml, float tiempo_s)
{
    infusion_plan_t plan;
    esp_err_t err = infusion_calcular(volumen_ml, tiempo_s, &plan);
    if (err != ESP_OK) {
        return err;                               /* [CAMBIO] sin nextion_estado */
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    detener_interno();

    s_plan            = plan;
    s_pasos_dados     = 0;
    s_pasos_restantes = plan.pasos_totales;
    s_pausada         = false;                    /* [CAMBIO] */
    s_completada      = false;                    /* [CAMBIO] */
    escribir_estado(s_estado);                    /* energizar en la posicion actual */

    gptimer_alarm_config_t alarma = {
        .alarm_count  = plan.periodo_us,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    gptimer_set_raw_count(s_timer, 0);
    gptimer_set_alarm_action(s_timer, &alarma);

    s_activa = true;
    err = gptimer_start(s_timer);
    s_timer_corriendo = (err == ESP_OK);
    if (err != ESP_OK) s_activa = false;
    xSemaphoreGive(s_mutex);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Inicio: %.2f mL en %.1f s -> %.2f mL/h", plan.volumen_ml,
                 plan.tiempo_s, plan.caudal_ml_h);
        ESP_LOGI(TAG, "  %lu pasos, 1 paso cada %llu us (%.2f pasos/s)",
                 (unsigned long)plan.pasos_totales,
                 (unsigned long long)plan.periodo_us, plan.pasos_por_seg);
    }
    return err;
}

void infusion_detener(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool estaba_activa = s_activa || s_pausada;   /* [CAMBIO] tambien desde pausa */
    s_pausada = false;                            /* [CAMBIO] */
    detener_interno();
    xSemaphoreGive(s_mutex);

    if (estaba_activa) {
        ESP_LOGI(TAG, "Detenida por el usuario: %.2f mL infundidos",
                 infusion_volumen_infundido_ml());
    }
}

bool infusion_activa(void)
{
    return s_activa;
}

float infusion_volumen_infundido_ml(void)
{
    if (s_plan.pasos_totales == 0) return 0.0f;
    return s_plan.volumen_ml * (float)s_pasos_dados / (float)s_plan.pasos_totales;
}

/* ---------------------------------------------------------------------------
 *  [CAMBIO] Funciones nuevas: pausa, reanudar y datos para la pantalla
 * -------------------------------------------------------------------------*/
void infusion_pausar(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_activa) {
        s_activa = false;
        if (s_timer_corriendo) {
            gptimer_stop(s_timer);
            s_timer_corriendo = false;
        }
        /* Las bobinas quedan energizadas para que el embolo no se devuelva
         * por la presion de la jeringa (el tornillo T8 se puede mover solo). */
        s_pausada = true;
        ESP_LOGI(TAG, "Pausa: %.2f mL infundidos", infusion_volumen_infundido_ml());
    }
    xSemaphoreGive(s_mutex);
}

void infusion_reanudar(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_pausada && s_pasos_restantes > 0) {
        gptimer_alarm_config_t alarma = {
            .alarm_count  = s_plan.periodo_us,
            .reload_count = 0,
            .flags.auto_reload_on_alarm = true,
        };
        gptimer_set_raw_count(s_timer, 0);
        gptimer_set_alarm_action(s_timer, &alarma);
        s_pausada = false;
        s_activa  = true;
        s_timer_corriendo = (gptimer_start(s_timer) == ESP_OK);
        if (!s_timer_corriendo) s_activa = false;
        ESP_LOGI(TAG, "Reanudada");
    }
    xSemaphoreGive(s_mutex);
}

infusion_estado_t infusion_estado(void)
{
    if (s_completada) return INFUSION_COMPLETADA;
    if (s_pausada)    return INFUSION_PAUSADA;
    if (s_activa)     return INFUSION_ACTIVA;
    return INFUSION_INACTIVA;
}

uint32_t infusion_pasos_dados(void)
{
    return s_pasos_dados;
}

uint32_t infusion_tiempo_restante_s(void)
{
    return (uint32_t)((uint64_t)s_pasos_restantes * s_plan.periodo_us / 1000000ULL);
}

float infusion_avance_mm(void)
{
    return (float)s_pasos_dados / SUBPASOS_POR_VUELTA * AVANCE_POR_VUELTA_MM;
}

infusion_plan_t infusion_plan_actual(void)
{
    return s_plan;
}

/* [CAMBIO] La tarea ya no lee la Nextion ni manda progreso: solo espera el
 * aviso del ISR de que se dieron todos los pasos y apaga el motor.          */
static void tarea_infusion(void *arg)
{
    for (;;) {
        /* El ISR aviso que se dieron todos los pasos */
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0) {
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            bool terminada = !s_activa && s_timer_corriendo;
            if (terminada) {
                detener_interno();
                s_completada = true;              /* [CAMBIO] main.c lo lee */
            }
            xSemaphoreGive(s_mutex);
            if (terminada) {
                ESP_LOGI(TAG, "Infusion completada: %.2f mL",
                         infusion_volumen_infundido_ml());
            }
        }
    }
}

/* ======================================================================== */

esp_err_t infusion_init(const infusion_config_t *cfg)
{
    s_pin[0] = cfg->in1;
    s_pin[1] = cfg->in2;
    s_pin[2] = cfg->in3;
    s_pin[3] = cfg->in4;

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << cfg->in1) | (1ULL << cfg->in2) |
                        (1ULL << cfg->in3) | (1ULL << cfg->in4),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) return err;
    liberar_bobinas();

    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) return ESP_ERR_NO_MEM;

    gptimer_config_t tcfg = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,                  /* 1 tick = 1 us */
    };
    err = gptimer_new_timer(&tcfg, &s_timer);
    if (err != ESP_OK) return err;

    gptimer_event_callbacks_t cbs = { .on_alarm = on_alarma };
    err = gptimer_register_event_callbacks(s_timer, &cbs, NULL);
    if (err != ESP_OK) return err;
    err = gptimer_enable(s_timer);
    if (err != ESP_OK) return err;

    if (xTaskCreate(tarea_infusion, "infusion", 4096, NULL, 5, &s_tarea) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Listo. Jeringa D=%.2f mm, avance %.2f mm/vuelta, %d pasos/vuelta",
             JERINGA_DIAMETRO_INTERNO_MM, AVANCE_POR_VUELTA_MM, SUBPASOS_POR_VUELTA);
    return ESP_OK;
}
