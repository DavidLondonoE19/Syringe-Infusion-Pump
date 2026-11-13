/* ============================================================================
 *  infusion.h - Control de infusion: volumen + tiempo -> pasos del NEMA17
 *  Proyecto: Syringe-Infusion-Pump (ESP32, ESP-IDF, puente H L298N)
 * ----------------------------------------------------------------------------
 *  Flujo:
 *    1. [CAMBIO] main.c recibe volumen y caudal de la Nextion (nextion_bomba.c),
 *       calcula el tiempo y llama a infusion_iniciar(volumen, tiempo).
 *    2. Se calcula cuantos pasos equivalen a ese volumen y cada cuanto
 *       tiempo hay que dar un paso para terminar justo en el tiempo pedido.
 *    3. Un temporizador de hardware (gptimer, 1 us de resolucion) da cada
 *       paso, independiente del resto del programa.
 *
 *  Calculo:
 *    area_mm2   = pi * (D/2)^2                 (seccion interna de la jeringa)
 *    avance_mm  = volumen_ml * 1000 / area_mm2 (1 mL = 1000 mm^3)
 *    vueltas    = avance_mm / AVANCE_POR_VUELTA_MM
 *    pasos      = vueltas * pasos_por_vuelta
 *    periodo_us = tiempo_s * 1e6 / pasos       (tiempo entre pasos)
 * ==========================================================================*/
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

/* ------------------------------------------------------------------------
 *  PARAMETROS MECANICOS  -> AJUSTAR A SU MONTAJE
 * ----------------------------------------------------------------------*/
/* Diametro INTERNO del cuerpo de la jeringa (medir con calibrador).
 * Referencias tipicas: 5 mL ~ 12.0 mm | 10 mL ~ 14.5 mm | 20 mL ~ 19.1 mm   */
#define JERINGA_DIAMETRO_INTERNO_MM   14.5f
#define JERINGA_VOLUMEN_MAX_ML        10.0f

/* Cuanto avanza el embolo (mm) por cada vuelta completa del motor.
 * Tornillo T8 (4 entradas) = 8 mm | varilla roscada M8 = 1.25 mm.
 * Si hay engranajes/poleas entre motor y tornillo, incluir la relacion.     */
#define AVANCE_POR_VUELTA_MM          8.0f

#define MOTOR_PASOS_POR_VUELTA        200   /* NEMA17 de 1.8 grados          */
#define MOTOR_MEDIO_PASO              0     /* 1 = medio paso (mas suave)    */
#define MOTOR_SENTIDO_INFUSION        1     /* cambiar a -1 si empuja al reves */
#define MOTOR_MAX_PASOS_POR_SEG       300   /* limite para no perder pasos   */

/* [CAMBIO] Se quito la seccion NEXTION (NEXTION_ESCALA_*, NEXTION_OBJ_*):
 * ahora toda la comunicacion con la pantalla esta en nextion_bomba.c/.h      */

/* ------------------------------------------------------------------------
 *  API
 * ----------------------------------------------------------------------*/
typedef struct {
    gpio_num_t  in1, in2, in3, in4;   /* pines hacia IN1..IN4 del L298N     */
    /* [CAMBIO] se quito uart_nextion: infusion.c ya no lee la pantalla      */
} infusion_config_t;

typedef struct {
    float    volumen_ml;
    float    tiempo_s;
    uint32_t pasos_totales;
    uint64_t periodo_us;              /* tiempo entre pasos                 */
    float    pasos_por_seg;
    float    caudal_ml_h;
    float    rpm;                     /* [CAMBIO] velocidad del motor       */
    float    mm_s;                    /* [CAMBIO] velocidad del embolo      */
} infusion_plan_t;

/* [CAMBIO] Estado de la infusion, para que main.c sepa cuando termino */
typedef enum {
    INFUSION_INACTIVA = 0,            /* sin iniciar o detenida             */
    INFUSION_ACTIVA,                  /* dando pasos                        */
    INFUSION_PAUSADA,                 /* detenida a mitad, se puede reanudar */
    INFUSION_COMPLETADA,              /* dio todos los pasos                */
} infusion_estado_t;

/* Configura los pines del motor, el temporizador y la tarea que avisa cuando
 * termina la infusion. Llamar UNA vez en app_main.                          */
esp_err_t infusion_init(const infusion_config_t *cfg);

/* Solo calcula (no mueve el motor). Util para mostrar datos o validar.     */
esp_err_t infusion_calcular(float volumen_ml, float tiempo_s, infusion_plan_t *plan);

/* Calcula y arranca la infusion. Tambien se puede llamar desde app_main para
 * probar sin la Nextion, por ejemplo: infusion_iniciar(2.0f, 30.0f);        */
esp_err_t infusion_iniciar(float volumen_ml, float tiempo_s);

void  infusion_detener(void);
bool  infusion_activa(void);
float infusion_volumen_infundido_ml(void);

/* [CAMBIO] Funciones nuevas para la pantalla ------------------------------ */
void              infusion_pausar(void);           /* el motor se queda quieto y con fuerza */
void              infusion_reanudar(void);         /* sigue donde iba                    */
infusion_estado_t infusion_estado(void);
uint32_t          infusion_pasos_dados(void);
uint32_t          infusion_tiempo_restante_s(void);
float             infusion_avance_mm(void);        /* cuanto ha avanzado el embolo       */
infusion_plan_t   infusion_plan_actual(void);      /* el plan de la infusion en curso    */
