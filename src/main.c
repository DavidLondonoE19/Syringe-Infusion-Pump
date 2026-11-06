#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h> 

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/ledc.h"

#include "esp_adc/adc_oneshot.h"

#include "esp_err.h"
#include "esp_log.h"

// Variables globales
#define PASOS_POR_REV 200

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

static const uint32_t stepper_sequence[4][4] = {
    {1, 0, 1, 0},
    {0, 1, 1, 0},
    {0, 1, 0, 1},
    {1, 0, 0, 1},
};

static int estado_actual = 0;

static void stepper_estado(int estado) {
    ESP_ERROR_CHECK(gpio_set_level(STEPPER_PORT_1, stepper_sequence[estado][0]));
    ESP_ERROR_CHECK(gpio_set_level(STEPPER_PORT_2, stepper_sequence[estado][1]));
    ESP_ERROR_CHECK(gpio_set_level(STEPPER_PORT_3, stepper_sequence[estado][2]));
    ESP_ERROR_CHECK(gpio_set_level(STEPPER_PORT_4, stepper_sequence[estado][3]));
}

static bool stepper_finalizado = false;

static void calcular_pasos(float volumen_total, float tiempo_total) {
    while(stepper_finalizado == false){
        
    }

// Servo
#define SERVO_PORT GPIO_NUM_27
#define SERVO_PWM_FREQ 50U

// Entradas
    // Sensor distancia SHARP (ADC)
#define SHARP_ADC_CHANNEL ADC_CHANNEL_6 // GPIO34

    // Encoder (GPIO)
#define ENCODER_PORT GPIO_NUM_14

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
    char_mensaje[24];
    const uint8_t fin_mensaje[] = {0xFF, 0xFF, 0xFF};
    int longitud = snprintf(mensaje, sizeof(mensaje), "n0.val=%lu", (unsigned long)contador);
    uart_write_bytes(UART_NEXTION, mensaje, longitud);
    uart_write_bytes(UART_NEXTION, fin_mensaje, sizeof(fin_mensaje));
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

    // Configuracion Paso a Paso

    gpio_config_t stepper_config = {
        .pin_bit_mask = (1ULL << STEPPER_PORT_1) |
                        (1ULL << STEPPER_PORT_2) |
                        (1ULL << STEPPER_PORT_3) |
                        (1ULL << STEPPER_PORT_4),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&stepper_config));

    GPIO_SET_LEVEL(STEPPER_PORT_1, 0);
    GPIO_SET_LEVEL(STEPPER_PORT_2, 0); 
    GPIO_SET_LEVEL(STEPPER_PORT_3, 0);
    GPIO_SET_LEVEL(STEPPER_PORT_4, 0);

    // Configuracion UART NEXTION

    uart_config_t uart_nextion_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NEXTION, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NEXTION, &uart_nextion_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NEXTION, NEXTION_TX, NEXTION_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}