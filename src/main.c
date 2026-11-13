#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h> 

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_intr_alloc.h"

#include <math.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/ledc.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"

#include "infusion.h"   // control del motor por volumen y tiempo
#include "nextion_bomba.h"  // [CAMBIO] pantalla Nextion (botones y textos)
#include "esp_timer.h"      // [CAMBIO] reloj en microsegundos para el encoder

// Variables globales
#define PASOS_POR_REV 200
#define timer_Hz 1000000 // cada tick = 1us

// [CAMBIO] Antes eran TickType_t con xTaskGetTickCount(), que cuenta ticks de
// 10 ms y no microsegundos: las rpm salian miles de veces mas altas. Ahora el
// tiempo de cada pulso lo toma el ISR con esp_timer_get_time() (en us).
int64_t ultimo_pulso_encoder_us = 0;
int64_t nuevo_pulso_encoder_us = 0;
int64_t periodo_entre_pulsos = 0;
static volatile int64_t tiempo_pulso_isr_us = 0;   // [CAMBIO]
static volatile bool evento_encoder = false;
int contador_pulsos_encoder = 0;
float vel_angular_rpm = 0;                          // [CAMBIO] global: la usa la pantalla


static const char *TAG_ADC = "ADC_SHARP";
static const char *TAG_ENCODER = "ENCODER";
static const char *TAG_STEPPER = "STEPPER";
static const char *TAG_SERVO = "SERVO";
static const char *TAG_NEXTION = "NEXTION";

// UART y Nextion
#define UART_PORT UART_NUM_0
#define UART_BAUD_RATE 115200
#define UART_NEXTION UART_NUM_2

#define NEXTION_RX 16
#define NEXTION_TX 17

#define PERIODO_NEXTION 250

// Salidas
// Motor paso a paso
#define STEPPER_PORT_1 GPIO_NUM_32
#define STEPPER_PORT_2 GPIO_NUM_33
#define STEPPER_PORT_3 GPIO_NUM_25
#define STEPPER_PORT_4 GPIO_NUM_26

// La secuencia de pasos, el calculo de volumen -> pasos y el temporizador
// estan ahora en infusion.c / infusion.h

// Servo
#define SERVO_PORT GPIO_NUM_27
#define SERVO_PWM_FREQ 50U
#define SERVO_CERRADA  -90   // [CAMBIO] angulo de valvula cerrada (cambiar si es al reves)
#define SERVO_ABIERTA   90   // [CAMBIO] angulo de valvula abierta

// Entradas
    // Sensor distancia SHARP (ADC)
#define SHARP_ADC_CHANNEL ADC_CHANNEL_6 // GPIO34

    // Encoder (GPIO)
#define ENCODER_PORT GPIO_NUM_14

static void IRAM_ATTR encoder_isr_handler(void *argument){
    (void) argument;
    tiempo_pulso_isr_us = esp_timer_get_time();     // [CAMBIO] momento exacto del pulso
    evento_encoder = true;
}

static void posicionar_servo(int grados){
    uint32_t pwm_up_us = 0;
    uint32_t pwm_up_bits = 0;

    if (grados < -90) grados = -90;
    if (grados > 90) grados = 90;

    pwm_up_us = 10.5556*grados + 1450.5; // Ecuacion de grados a PWM
    if (pwm_up_us < 500) pwm_up_us = 500;
    if (pwm_up_us > 2400) pwm_up_us = 2400;

    pwm_up_bits = (pwm_up_us*16384)/(1000000/SERVO_PWM_FREQ); // el 1.000.000 es para pasar de segundos a microsegundos.

    ESP_ERROR_CHECK(ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm_up_bits, 0));
}

static void enviar_dato_nextion(uint32_t contador) {
    char mensaje[24];
    const uint8_t fin_mensaje[] = {0xFF, 0xFF, 0xFF};
    int longitud = snprintf(mensaje, sizeof(mensaje), "n0.val=%lu", (unsigned long)contador);
    uart_write_bytes(UART_NEXTION, mensaje, longitud);
    uart_write_bytes(UART_NEXTION, fin_mensaje, sizeof(fin_mensaje));
}

static float leer_distancia(adc_oneshot_unit_handle_t adc1) {
    int raw = 0; 
    float voltaje_distancia = 0;
    float distancia_sharp = 0;
    esp_err_t adc_lectura = adc_oneshot_read(adc1, SHARP_ADC_CHANNEL, &raw);

    if (adc_lectura == ESP_OK){
        voltaje_distancia = (raw*3.3f/4095.0f);
        ESP_LOGD(TAG_ADC, "Lectura Sharp (V): %.3f", voltaje_distancia); // [CAMBIO] LOGI -> LOGD: no llena el monitor
        if (voltaje_distancia < 0.05f) return -1.0f;   // [CAMBIO] con 0 V la formula da infinito
        distancia_sharp = 11.587f * powf(voltaje_distancia, -1.0977f);
        return distancia_sharp;
    }

    else{
        ESP_LOGW(TAG_ADC, "Error de lectura de sharp");
        return -1.0f; // Return an error value
    }
}




/* ===========================================================================
 *  [CAMBIO] Maquina de estados de la bomba + pantalla Nextion
 *  CONFIG -> INFUNDIENDO <-> PAUSA -> CONFIG
 *               |-> FIN -> CONFIG          |-> ALARMA -> CONFIG
 * =========================================================================*/

// Rangos y pasos de los botones + / -
#define VOLUMEN_INICIAL_ML      10.0f
#define VOLUMEN_MIN_ML          0.5f
#define VOLUMEN_PASO_ML         0.5f      // el maximo es JERINGA_VOLUMEN_MAX_ML (infusion.h)
#define CAUDAL_INICIAL_ML_H     60.0f
#define CAUDAL_MIN_ML_H         1.0f
#define CAUDAL_MAX_ML_H         999.0f

// Sensor Sharp y fin de carrera
#define PERIODO_SHARP_MS        200       // cada cuanto se lee el Sharp
#define SHARP_FIN_CARRERA_CM    4.5f      // mas cerca que esto = el carro llego al final (0 = desactivar)
#define SHARP_LECTURAS_ALARMA   3         // lecturas seguidas para confirmarlo

// Encoder y motor bloqueado
#define ENCODER_SIN_PULSOS_MS   2000      // sin pulsos en este tiempo -> rpm = 0
#define BLOQUEO_RPM_MINIMA      5.0f      // por debajo de esta rpm comandada no se vigila (el encoder no alcanza a medir)
#define BLOQUEO_FRACCION        0.5f      // medida < 50 % de la comandada = sospecha de bloqueo
#define BLOQUEO_TIEMPO_MS       5000      // sospecha sostenida este tiempo = alarma

typedef enum { EST_CONFIG, EST_INFUNDIENDO, EST_PAUSA, EST_FIN, EST_ALARMA } estado_bomba_t;

static estado_bomba_t estado = EST_CONFIG;
static float volumen_ml = VOLUMEN_INICIAL_ML;
static float caudal_ml_h = CAUDAL_INICIAL_ML_H;
static float    fin_ml = 0;               // lo que se muestra en la pagina FIN
static uint32_t fin_pasos = 0;
static int      lecturas_fin_carrera = 0;
static int64_t  t_inicio_sospecha_bloqueo = 0;


// El caudal sube de a 1 mL/h hasta 20, de a 5 hasta 100 y de a 10 despues
static float paso_caudal(float caudal, bool subir) {
    float ref = subir ? caudal : caudal - 0.01f;
    if (ref < 20.0f)  return 1.0f;
    if (ref < 100.0f) return 5.0f;
    return 10.0f;
}

static void ir_a_config(void) {
    estado = EST_CONFIG;
    nex_ir_a(NEX_PAG_CONFIG);
}

static void ir_a_alarma(const char *mensaje) {
    infusion_detener();
    posicionar_servo(SERVO_CERRADA);
    estado = EST_ALARMA;
    nex_ir_a(NEX_PAG_ALARMA);
    nex_mostrar_alarma(mensaje);
    ESP_LOGW(TAG_NEXTION, "ALARMA: %s", mensaje);
}

static void atender_evento(nex_evento_t ev) {
    switch (estado) {
    case EST_CONFIG:
        if (ev == NEX_EV_VOLUMEN_MAS)   volumen_ml += VOLUMEN_PASO_ML;
        if (ev == NEX_EV_VOLUMEN_MENOS) volumen_ml -= VOLUMEN_PASO_ML;
        if (ev == NEX_EV_CAUDAL_MAS)    caudal_ml_h += paso_caudal(caudal_ml_h, true);
        if (ev == NEX_EV_CAUDAL_MENOS)  caudal_ml_h -= paso_caudal(caudal_ml_h, false);
        if (volumen_ml < VOLUMEN_MIN_ML)        volumen_ml = VOLUMEN_MIN_ML;
        if (volumen_ml > JERINGA_VOLUMEN_MAX_ML) volumen_ml = JERINGA_VOLUMEN_MAX_ML;
        if (caudal_ml_h < CAUDAL_MIN_ML_H)      caudal_ml_h = CAUDAL_MIN_ML_H;
        if (caudal_ml_h > CAUDAL_MAX_ML_H)      caudal_ml_h = CAUDAL_MAX_ML_H;

        if (ev == NEX_EV_INICIAR) {
            float tiempo_s = volumen_ml / caudal_ml_h * 3600.0f;   // t = V / Q
            if (infusion_iniciar(volumen_ml, tiempo_s) == ESP_OK) {
                posicionar_servo(SERVO_ABIERTA);
                lecturas_fin_carrera = 0;
                t_inicio_sospecha_bloqueo = 0;
                estado = EST_INFUNDIENDO;
                nex_ir_a(NEX_PAG_INFUSION);
            } else {
                ESP_LOGW(TAG_NEXTION, "No se pudo iniciar: revise volumen y caudal");
            }
        }
        break;

    case EST_INFUNDIENDO:
        if (ev == NEX_EV_PAUSA) {
            infusion_pausar();
            posicionar_servo(SERVO_CERRADA);
            estado = EST_PAUSA;
        }
        break;

    case EST_PAUSA:
        if (ev == NEX_EV_PAUSA) {                    // REANUDAR
            posicionar_servo(SERVO_ABIERTA);
            infusion_reanudar();
            t_inicio_sospecha_bloqueo = 0;
            estado = EST_INFUNDIENDO;
        }
        if (ev == NEX_EV_CANCELAR) {
            infusion_detener();
            ir_a_config();
        }
        break;

    case EST_FIN:
    case EST_ALARMA:
        if (ev == NEX_EV_ACEPTAR) ir_a_config();     // NUEVA o ACEPTAR
        break;
    }
}

// Cuando el motor dio todos los pasos -> pagina FIN
static void revisar_fin_de_infusion(void) {
    if (estado == EST_INFUNDIENDO && infusion_estado() == INFUSION_COMPLETADA) {
        posicionar_servo(SERVO_CERRADA);
        fin_ml = infusion_volumen_infundido_ml();
        fin_pasos = infusion_pasos_dados();
        estado = EST_FIN;
        nex_ir_a(NEX_PAG_FIN);
        nex_mostrar_fin(fin_ml, fin_pasos);
    }
}

// Sharp: el carro llego al final de la jeringa
static void revisar_fin_de_carrera(float distancia_cm) {
    if (estado != EST_INFUNDIENDO || SHARP_FIN_CARRERA_CM <= 0) return;
    if (distancia_cm > 0 && distancia_cm < SHARP_FIN_CARRERA_CM) {
        if (++lecturas_fin_carrera >= SHARP_LECTURAS_ALARMA) ir_a_alarma("FIN DE CARRERA");
    } else {
        lecturas_fin_carrera = 0;
    }
}

// Encoder: el motor deberia girar y no gira
static void revisar_motor_bloqueado(int64_t ahora) {
    if (estado != EST_INFUNDIENDO) { t_inicio_sospecha_bloqueo = 0; return; }
    float rpm_esperada = infusion_plan_actual().rpm;
    if (rpm_esperada < BLOQUEO_RPM_MINIMA) return;           // muy lento para medirlo

    if (vel_angular_rpm < rpm_esperada * BLOQUEO_FRACCION) {
        if (t_inicio_sospecha_bloqueo == 0) t_inicio_sospecha_bloqueo = ahora;
        if (ahora - t_inicio_sospecha_bloqueo >= BLOQUEO_TIEMPO_MS * 1000LL) {
            ir_a_alarma("MOTOR BLOQUEADO");
        }
    } else {
        t_inicio_sospecha_bloqueo = 0;
    }
}

// Manda a la pantalla los datos de la pagina actual
static void actualizar_pantalla(float distancia_cm) {
    if (estado == EST_CONFIG) {
        float tiempo_s = volumen_ml / caudal_ml_h * 3600.0f;
        infusion_plan_t plan = {0};
        infusion_calcular(volumen_ml, tiempo_s, &plan);      // solo calcula, no mueve el motor
        nex_config_t c = {
            .volumen_ml = volumen_ml,
            .caudal_mlh = caudal_ml_h,
            .rpm        = plan.rpm,
            .hz         = plan.pasos_por_seg,
            .mm_s       = plan.mm_s,
            .duracion_s = (uint32_t)tiempo_s,
            .ir_cm      = distancia_cm,
        };
        nex_mostrar_config(&c);
    } else if (estado == EST_INFUNDIENDO || estado == EST_PAUSA) {
        infusion_plan_t plan = infusion_plan_actual();
        nex_infusion_t d = {
            .pausada       = (estado == EST_PAUSA),
            .caudal_mlh    = plan.caudal_ml_h,
            .ml_entregados = infusion_volumen_infundido_ml(),
            .ml_objetivo   = plan.volumen_ml,
            .restante_s    = infusion_tiempo_restante_s(),
            .rpm_cmd       = (estado == EST_PAUSA) ? 0.0f : plan.rpm,
            .rpm_med       = vel_angular_rpm,
            .ir_cm         = distancia_cm,
            .avance_mm     = infusion_avance_mm(),
        };
        nex_mostrar_infusion(&d);
    } else if (estado == EST_FIN) {
        nex_mostrar_fin(fin_ml, fin_pasos);
    }
}

void app_main() {

    //Configuracion PWM Servo
    ledc_timer_config_t timer_pwm ={
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .freq_hz = SERVO_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer_pwm));

    ledc_channel_config_t canal_pwm_servo = {
        .gpio_num = SERVO_PORT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };

    ESP_ERROR_CHECK(ledc_channel_config(&canal_pwm_servo));

    //Configuracion ADC Sensor SHARP

    adc_oneshot_unit_handle_t adc1 = NULL;
    adc_oneshot_unit_init_cfg_t adc_sharp = {
        .unit_id = ADC_UNIT_1
    };

    posicionar_servo(SERVO_CERRADA); // Posicion inicial del servo  [CAMBIO] -90 -> SERVO_CERRADA

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_sharp, &adc1));

    adc_oneshot_chan_cfg_t adc_sharp_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, SHARP_ADC_CHANNEL,&adc_sharp_config));

    // Configuración encoder

    gpio_config_t encoder_config = {
        .pin_bit_mask = (1ULL << ENCODER_PORT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };

    ESP_ERROR_CHECK(gpio_config(&encoder_config));

    // Configuracion UART NEXTION
    uart_config_t uart_nextion_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // [CAMBIO] buffer de transmision 0 -> 512: asi enviar a la pantalla no frena el ciclo
    ESP_ERROR_CHECK(uart_driver_install(UART_NEXTION, 256, 512, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NEXTION, &uart_nextion_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NEXTION, NEXTION_TX, NEXTION_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Configuracion ISR Encoder
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENCODER_PORT,encoder_isr_handler, NULL));

    // Configuracion motor paso a paso + infusion (despues de instalar el UART)
    infusion_config_t infusion_cfg = {
        .in1 = STEPPER_PORT_1,
        .in2 = STEPPER_PORT_2,
        .in3 = STEPPER_PORT_3,
        .in4 = STEPPER_PORT_4,
        // [CAMBIO] se quito .uart_nextion: ahora la pantalla la maneja nextion_bomba.c
    };
    ESP_ERROR_CHECK(infusion_init(&infusion_cfg));

    nex_init();   // [CAMBIO] muestra la pagina de configuracion (despues de instalar el UART)

    // Prueba sin Nextion: descomentar para infundir 2 mL en 30 s al arrancar
    // infusion_iniciar(2.0f, 30.0f);

    // main 


    // [CAMBIO] Variables del ciclo principal
    float   distancia_sharp = -1.0f;
    int64_t t_sharp = 0, t_pantalla = 0;

    for(;;){
        int64_t ahora = esp_timer_get_time();

        // Lectura sensor de distancia
        // [CAMBIO] cada PERIODO_SHARP_MS en vez de cada 10 ms
        if (ahora - t_sharp >= PERIODO_SHARP_MS * 1000LL) {
            t_sharp = ahora;
            distancia_sharp = leer_distancia(adc1);
            revisar_fin_de_carrera(distancia_sharp);    // [CAMBIO]
        }

        //
        if (evento_encoder == true){
            ultimo_pulso_encoder_us = nuevo_pulso_encoder_us;
            nuevo_pulso_encoder_us = tiempo_pulso_isr_us;                // [CAMBIO] us reales del ISR
            periodo_entre_pulsos = nuevo_pulso_encoder_us - ultimo_pulso_encoder_us;
            ESP_LOGD(TAG_ENCODER, "Pulso encoder detectado. Tiempo entre pulsos: %lld us", (long long)periodo_entre_pulsos); // [CAMBIO] LOGD y %lld
            contador_pulsos_encoder++;
            if (periodo_entre_pulsos > 0) {                               // [CAMBIO] evita dividir entre 0
                float frecuencia_de_pulsos = 1000000.0 / periodo_entre_pulsos; // Frecuencia en Hz
                vel_angular_rpm = (frecuencia_de_pulsos-11.0f)/0.14434f; // Ecuacion de frecuencia a RPM
                if (vel_angular_rpm < 0) vel_angular_rpm = 0;             // [CAMBIO]
            }
            ESP_LOGD(TAG_ENCODER, "Velocidad angular: %.2f RPM", vel_angular_rpm); // [CAMBIO] LOGD
            evento_encoder = false;

        }
        // [CAMBIO] si no llegan pulsos, el motor esta quieto
        if (ahora - nuevo_pulso_encoder_us > ENCODER_SIN_PULSOS_MS * 1000LL) {
            vel_angular_rpm = 0;
        }

        // [CAMBIO] Pantalla, maquina de estados y alarmas
        atender_evento(nex_leer_evento());
        revisar_fin_de_infusion();
        revisar_motor_bloqueado(ahora);

        if (ahora - t_pantalla >= PERIODO_NEXTION * 1000LL) {
            t_pantalla = ahora;
            actualizar_pantalla(distancia_sharp);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // ceder la CPU (evita el watchdog)
    }
}
