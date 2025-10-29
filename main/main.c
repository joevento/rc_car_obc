#include <stdio.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h" // For PWM control

// UART defines
#define UART_NUM UART_NUM_2
#define UART_TX_PIN GPIO_NUM_17
#define UART_RX_PIN GPIO_NUM_16
#define BUF_SIZE 1024

// Motor control defines
#define MOTOR_CTRL_PIN GPIO_NUM_13 // GPIO for motor control (CTRL_MOTO)
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT // 10-bit resolution (0-1023)
#define LEDC_FREQUENCY 5000 // 5 KHz PWM frequency

// Command for starting a scan (SYNC_BYTE, CMD_SCAN)
const uint8_t START_SCAN_COMMAND[] = {0xA5, 0x20};
const size_t START_SCAN_COMMAND_LEN = sizeof(START_SCAN_COMMAND);

static const char *TAG = "RPLIDAR_TEST";

void set_motor_speed(uint32_t speed_percentage) {
    if (speed_percentage > 100) {
        speed_percentage = 100;
    }
    uint32_t duty = (speed_percentage * ((1 << LEDC_DUTY_RES) - 1)) / 100;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    ESP_LOGI(TAG, "Motor speed set to %lu%% (duty: %lu)", speed_percentage, duty);
}

void app_main(void) {
    // 1. Configure UART2 for communication with RPLIDAR
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "ESP32 RPLIDAR Scan Test");

    // 2. Configure PWM for motor control
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_APB_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_CTRL_PIN,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // 3. Start the motor
    ESP_LOGI(TAG, "Starting motor...");
    set_motor_speed(100);
    vTaskDelay(pdMS_TO_TICKS(4000)); //  This wait is SUPER critical, if start_scan is sent before the lidar stabilizes the command fails

    // 4. Send the start scan command
    uart_write_bytes(UART_NUM, (const char*)START_SCAN_COMMAND, START_SCAN_COMMAND_LEN);
    ESP_LOGI(TAG, "Sent START_SCAN command.");

    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer for UART data");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000)); // Keep alive
        }
    }

    // 5. Read and print data
    while (1) {
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            // Print received data to console (Serial monitor)
            for (int i = 0; i < len; i++) {
                printf("%02X ", data[i]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to yield to other tasks
    }
}