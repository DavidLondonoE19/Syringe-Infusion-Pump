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

#include "esp_adc_oneshot.h"

#include "esp_err.h"
#include "esp_log.h"

// UART y Nextion
#define UART_PORT UART_NUM_0
#define UART_BAUD_RATE 115200
#define UART_NEXTION UART_NUM_2
#define COLOR_ACTIVO_NEXTION 2016
#define COLOR_INACTIVO_NEXTION

#define NEXTION_RX 16
#define NEXTION_TX 17

#define PERIODO_NEXTION 250

// Salidas
    // Motor paso a paso
#define STEPPER_PORT_A1 GPIO_NUM_32
#define STEPPER_PORT_A2 GPIO_NUM_33
#define STEPPER_PORT_B1 GPIO_NUM_25
#define STEPPER_PORT_B2 GPIO_NUM_26

    // Servo
#define uint32 CONT_DUTY_CYCLE_SERVO = 0;
#define SERVO_PORT GPIO_NUM_12

// Entradas
    // Sensor distancia SHARP (ADC)
#define SHARP_ADC_CHANNEL ADC_CHANNEL_6


void posicionar_servo(int grados){

}

void app_main() {
    ledc_timer_config_t timer_pwm ={
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer_pwm));
}