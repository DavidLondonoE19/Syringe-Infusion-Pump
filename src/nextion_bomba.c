// nextion_bomba.c — implementación de la interfaz Nextion de la bomba (versión de 4 páginas)
#include "nextion_bomba.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "nextion";

// Colores RGB565 (decimal) que cambia el código; el resto se fija en el editor
#define C_TEAL      3249    // encabezado de page1 infundiendo
#define C_AMBAR     56224   // encabezado de page1 en pausa

static const uint8_t FIN_TRAMA[3] = {0xFF, 0xFF, 0xFF};

static nex_pagina_t pagina_actual = NEX_PAG_CONFIG;

// ------------------------------------------------------------ envío ----
void nex_enviar(const char *cmd) {
    uart_write_bytes(NEX_UART, cmd, strlen(cmd));
    uart_write_bytes(NEX_UART, (const char *)FIN_TRAMA, 3);
}

static void enviarf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void enviarf(const char *fmt, ...) {
    char cmd[160];
    va_list a;
    va_start(a, fmt);
    vsnprintf(cmd, sizeof(cmd), fmt, a);
    va_end(a);
    nex_enviar(cmd);
}

// --------------------------------------------- caché: mandar solo lo nuevo ----
// Cada campo de la pantalla actual tiene una "casilla" con lo último que se le
// mandó. Si el texto nuevo es igual, no se envía nada. Al cambiar de página se
// borra todo, para que la página nueva se llene completa.
#define N_CASILLAS  12
static char cache[N_CASILLAS][120];   // 120: cabe el párrafo de tablaresumen

static void cache_borrar(void) {
    memset(cache, 0, sizeof(cache));
}

// objname.txt="texto"
static void poner_txt(int casilla, const char *obj, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
static void poner_txt(int casilla, const char *obj, const char *fmt, ...) {
    char texto[120];
    va_list a;
    va_start(a, fmt);
    vsnprintf(texto, sizeof(texto), fmt, a);
    va_end(a);
    if (strcmp(texto, cache[casilla]) == 0) return;
    strcpy(cache[casilla], texto);
    enviarf("%s.txt=\"%s\"", obj, texto);
}

// Comandos numéricos: "jVol.val=", "tEstado.bco=", "vis bCancel,"
static void poner_num(int casilla, const char *cmd_sin_valor, int valor) {
    char texto[40];
    snprintf(texto, sizeof(texto), "%d", valor);
    if (strcmp(texto, cache[casilla]) == 0) return;
    strcpy(cache[casilla], texto);
    enviarf("%s%d", cmd_sin_valor, valor);
}

// Segundos -> "6m 45s" o "1h 12m"
static void formato_tiempo(uint32_t s, char *out, size_t n) {
    if (s >= 3600) snprintf(out, n, "%luh %02lum", (unsigned long)(s / 3600), (unsigned long)((s % 3600) / 60));
    else           snprintf(out, n, "%lum %02lus", (unsigned long)(s / 60),   (unsigned long)(s % 60));
}

// ------------------------------------------------------------- init ----
// El UART2 lo instala main.c (uart_driver_install + uart_param_config +
// uart_set_pin). Aquí solo se espera a que la pantalla arranque y se muestra
// la página de configuración. Llamar DESPUÉS de instalar el UART.
void nex_init(void) {
    vTaskDelay(pdMS_TO_TICKS(300));          // la pantalla tarda en arrancar
    nex_ir_a(NEX_PAG_CONFIG);
    ESP_LOGI(TAG, "Nextion lista (UART %d)", (int)NEX_UART);
}

void nex_ir_a(nex_pagina_t pagina) {
    pagina_actual = pagina;
    enviarf("page %d", (int)pagina);
    cache_borrar();
}

// ------------------------------------------------------------ toques ----
// Botón + o - que se está manteniendo presionado
static nex_evento_t sostenido = NEX_EV_NINGUNO;
static int64_t      t_presion = 0;     // µs cuando se presionó
static int64_t      t_ultimo  = 0;     // µs de la última repetición

static bool es_repetible(nex_evento_t e) {
       return e >= NEX_EV_VOLUMEN_MAS && e <= NEX_EV_CAUDAL_MENOS;
}


// (página, id) -> evento
static nex_evento_t traducir(uint8_t pag, uint8_t id) {
    switch (pag) {
    case NEX_PAG_CONFIG:
        switch (id) {
        case ID_P0_VOL_MAS:   return NEX_EV_VOLUMEN_MAS;
        case ID_P0_VOL_MENOS: return NEX_EV_VOLUMEN_MENOS;
        case ID_P0_CAU_MAS:   return NEX_EV_CAUDAL_MAS;
        case ID_P0_CAU_MENOS: return NEX_EV_CAUDAL_MENOS;
        case ID_P0_INICIAR:   return NEX_EV_INICIAR;
        }
        break;
    case NEX_PAG_INFUSION:
        if (id == ID_P1_PAUSA)    return NEX_EV_PAUSA;
        if (id == ID_P1_CANCELAR) return NEX_EV_CANCELAR;
        break;
    case NEX_PAG_FIN:
        if (id == ID_P2_NUEVA)    return NEX_EV_ACEPTAR;
        break;
    case NEX_PAG_ALARMA:
        if (id == ID_P3_ACEPTAR)  return NEX_EV_ACEPTAR;
        break;
    }
    return NEX_EV_NINGUNO;
}

// Procesa un toque (presionar o soltar) y devuelve el evento que genera
static nex_evento_t procesar_toque(uint8_t pag, uint8_t id, bool presionado) {
    nex_evento_t e = traducir(pag, id);
    if (e == NEX_EV_NINGUNO) return NEX_EV_NINGUNO;
    int64_t ahora = esp_timer_get_time();

    // + y -: actúan al PRESIONAR y repiten mientras se mantengan
    if (es_repetible(e)) {
        if (presionado) { sostenido = e; t_presion = t_ultimo = ahora; return e; }
        if (sostenido == e) sostenido = NEX_EV_NINGUNO;
        return NEX_EV_NINGUNO;
    }
    return presionado ? NEX_EV_NINGUNO : e;
}

nex_evento_t nex_leer_evento(void) {
    static uint8_t buf[8];
    static uint8_t n = 0;
    uint8_t b;

    while (uart_read_bytes(NEX_UART, &b, 1, 0) == 1) {
        buf[n++] = b;
        bool fin = (n >= 3 && buf[n - 1] == 0xFF && buf[n - 2] == 0xFF && buf[n - 3] == 0xFF);
        if (fin) {
#if NEX_DEBUG
            printf("Nextion -> ");
            for (uint8_t i = 0; i < n; i++) printf("%02X ", buf[i]);
            printf("\n");
#endif
            bool es_toque = (n == 7 && buf[0] == 0x65);
            uint8_t pag = buf[1], id = buf[2], ev = buf[3];
            n = 0;
            if (es_toque) {
                nex_evento_t e = procesar_toque(pag, id, ev == 0x01);
                if (e != NEX_EV_NINGUNO) return e;
            }
        } else if (n >= sizeof(buf)) {
            n = 0;                               // basura: se descarta
        }
    }

    // Sin tramas nuevas: ¿hay un botón sostenido?
    int64_t ahora = esp_timer_get_time();
    if (es_repetible(sostenido) &&
        ahora - t_presion >= NEX_REPETIR_DESPUES_MS * 1000LL &&
        ahora - t_ultimo  >= NEX_REPETIR_CADA_MS * 1000LL) {
        t_ultimo = ahora;
        return sostenido;
    }
    return NEX_EV_NINGUNO;
}

// ------------------------------------------------------- pantallas ----
void nex_mostrar_config(const nex_config_t *c) {
    if (pagina_actual != NEX_PAG_CONFIG) return;
    char dur[16];
    formato_tiempo(c->duracion_s, dur, sizeof(dur));
    poner_txt(1, "volumen", "%.1f mL", c->volumen_ml);
    poner_txt(2, "caudal", "%.0f mL/h", c->caudal_mlh);
    poner_txt(3, "t0", "%.1f rpm  %.0f Hz  %.2f mm/s", c->rpm, c->hz, c->mm_s);
    poner_txt(4, "duracion", "Duracion %s IR %.1fcm", dur, c->ir_cm);
}

void nex_mostrar_infusion(const nex_infusion_t *d) {
    if (pagina_actual != NEX_PAG_INFUSION) return;
    char rest[16];
    formato_tiempo(d->restante_s, rest, sizeof(rest));
    int pct = (d->ml_objetivo > 0) ? (int)(d->ml_entregados * 100.0f / d->ml_objetivo) : 0;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;

    poner_txt(0, "titulo0", "%s", d->pausada ? "EN PAUSA" : "SUMINISTRANDO...");
    poner_num(1, "titulo0.bco=", d->pausada ? C_AMBAR : C_TEAL);
    poner_txt(2, "caudal", "%.1f mL/h", d->caudal_mlh);
    poner_num(3, "barrainfun.val=", pct);
    poner_txt(4, "mLrestante", "%.2f/%.2fmL", d->ml_entregados, d->ml_objetivo);
    poner_txt(5, "porcenta", "%d%%", pct);
    // Párrafo de tablaresumen: "\\r" llega a la Nextion como \r = salto de línea
    poner_txt(6, "tablaresumen",
              "Restante %s\\rMotor %.1f rpm\\rEncoder %.1f rpm\\rIR %.1f cm\\rAvance %.1fmm",
              rest, d->rpm_cmd, d->rpm_med, d->ir_cm, d->avance_mm);
    poner_txt(9, "bpausar", "%s", d->pausada ? "REANUDAR" : "PAUSAR");
    poner_num(10, "vis bcancelar,", d->pausada ? 1 : 0);   // CANCELAR solo aparece en pausa
}

void nex_mostrar_fin(float ml_entregados, uint32_t pasos) {
    if (pagina_actual != NEX_PAG_FIN) return;
    poner_txt(0, "volumen", "%.2f mL entregados", ml_entregados);
    poner_txt(1, "npasos", "%lu pasos", (unsigned long)pasos);
}

void nex_mostrar_alarma(const char *mensaje) {
    if (pagina_actual != NEX_PAG_ALARMA) return;
    poner_txt(0, "mbloqueado", "%s", mensaje);
}