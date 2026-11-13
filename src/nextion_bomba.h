// nextion_bomba.h — interfaz táctil Nextion de la bomba de infusión (versión de 4 páginas)
//
// La pantalla no decide nada: muestra lo que el ESP32 le manda y avisa cuando
// se toca un botón. Toda la lógica (jeringa, caudal, motor, alarmas) sigue en
// main.c. Este módulo solo traduce:
//   toques de la pantalla  ->  eventos (nex_evento_t) que main.c atiende
//   datos de la bomba      ->  textos en la pantalla
//
// Pantallas (una por estado de la máquina de main.c):
//   page0 = CONFIG     page1 = INFUNDIENDO / PAUSA     page2 = FIN     page3 = ALARMA
//
// Protocolo: Nextion Instruction Set por UART2 a 9600 baudios.
//   ESP32 -> pantalla:  objname.atributo=valor  + 0xFF 0xFF 0xFF
//   pantalla -> ESP32:  0x65 pagina id evento 0xFF 0xFF 0xFF   (evento 1 = presionar, 0 = soltar)

#pragma once
#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------- configuración ----
// UART2 en GPIO 16 (RX) y 17 (TX), configurado en main.c.
#define NEX_UART        UART_NUM_2
// Los pines (16 y 17) y los 9600 baudios se configuran en main.c
#define NEX_DEBUG       1       // 1 = imprime cada toque (página e id) en el monitor

// Auto-repetición de los botones + y - al mantenerlos presionados
#define NEX_REPETIR_DESPUES_MS   500   // empieza a repetir tras medio segundo
#define NEX_REPETIR_CADA_MS      120   // luego, un paso cada 120 ms

// --------------------------------------------------- ids de los botones ----
// Los asigna el Nextion Editor en el orden en que se crean los componentes
// (la página es el id 0). Si al tocar un botón el monitor muestra otro id,
// corrígelo aquí.
#define ID_P0_VOL_MENOS   4
#define ID_P0_VOL_MAS     7
#define ID_P0_CAU_MENOS   5
#define ID_P0_CAU_MAS     6
#define ID_P0_INICIAR     12

#define ID_P1_PAUSA       7
#define ID_P1_CANCELAR    8

#define ID_P2_NUEVA       4

#define ID_P3_ACEPTAR     4

// ---------------------------------------------------------------- tipos ----
typedef enum {
    NEX_PAG_CONFIG   = 0,
    NEX_PAG_INFUSION = 1,
    NEX_PAG_FIN      = 2,
    NEX_PAG_ALARMA   = 3,
} nex_pagina_t;

typedef enum {
    NEX_EV_NINGUNO = 0,
    // page0 (CONFIG)
    NEX_EV_VOLUMEN_MAS,
    NEX_EV_VOLUMEN_MENOS,
    NEX_EV_CAUDAL_MAS,
    NEX_EV_CAUDAL_MENOS,
    NEX_EV_INICIAR,
    // page1 (INFUNDIENDO)
    NEX_EV_PAUSA,             // pausar o reanudar (main.c decide según el estado)
    NEX_EV_CANCELAR,          // solo tiene sentido en pausa
    // page2 y page3
    NEX_EV_ACEPTAR,           // "NUEVA" en FIN o "ACEPTAR" en ALARMA: volver a CONFIG
} nex_evento_t;

// Lo que muestra la pantalla CONFIG
typedef struct {
    float    volumen_ml;
    float    caudal_mlh;
    float    rpm;             // del motor, calculadas con flujo.c
    float    hz;              // pasos/s
    float    mm_s;            // velocidad del émbolo
    uint32_t duracion_s;
    float    ir_cm;           // lectura actual del sensor IR
} nex_config_t;

// Lo que muestra la pantalla INFUNDIENDO
typedef struct {
    bool     pausada;
    float    caudal_mlh;
    float    ml_entregados;
    float    ml_objetivo;
    uint32_t restante_s;
    float    rpm_cmd;         // rpm comandada al motor
    float    rpm_med;         // rpm medida por el encoder
    float    ir_cm;
    float    avance_mm;
} nex_infusion_t;

// ------------------------------------------------------------ funciones ----
// Muestra la página CONFIG. El UART2 lo instala main.c: llamar después.
void nex_init(void);

// Devuelve el siguiente evento táctil (o NEX_EV_NINGUNO). Llamarla seguido,
// cada 10–20 ms, porque también maneja la auto-repetición de + y -.
nex_evento_t nex_leer_evento(void);

// Cambia de pantalla. Llamarla cuando main.c cambie de estado.
void nex_ir_a(nex_pagina_t pagina);

// Actualizan la pantalla actual. Solo mandan lo que cambió, así que se pueden
// llamar en cada vuelta del ciclo (o cada ~200 ms).
void nex_mostrar_config(const nex_config_t *c);
void nex_mostrar_infusion(const nex_infusion_t *d);
void nex_mostrar_fin(float ml_entregados, uint32_t pasos);
void nex_mostrar_alarma(const char *mensaje);

// Manda un comando crudo con su terminador FF FF FF (para pruebas).
void nex_enviar(const char *cmd);
