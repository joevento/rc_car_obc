#include "lidar.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include <string.h>

// UART defines
#define UART_NUM UART_NUM_2
#define UART_TX_PIN GPIO_NUM_17 // Orange and green wire
#define UART_RX_PIN GPIO_NUM_16 // yellow wire
#define BUF_SIZE 1024

// Motor control defines
#define MOTOR_CTRL_PIN GPIO_NUM_13
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT // 10-bit resolution (0-1023)
#define LEDC_FREQUENCY 5000 // 5 KHz PWM frequency

// Command for starting a scan (SYNC_BYTE, CMD_SCAN)
const uint8_t START_SCAN_COMMAND[] = {0xA5, 0x20};
const size_t START_SCAN_COMMAND_LEN = sizeof(START_SCAN_COMMAND);

static const char *TAG = "Lidar";

// Internal function to set motor speed
static void set_motor_speed(uint32_t speed_percentage) {
    if (speed_percentage > 100) {
        speed_percentage = 100;
    }
    uint32_t duty = (speed_percentage * ((1 << LEDC_DUTY_RES) - 1)) / 100;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    ESP_LOGI(TAG, "Motor speed set to %lu%% (duty: %lu)", speed_percentage, duty);
}

esp_err_t lidar_init(void) {
	ESP_LOGI(TAG, "Initializing lidar.");
    
    #ifdef DEBUG
        ESP_LOGI(TAG, "Initializing UART.");
    #endif
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE));
    #ifdef DEBUG
        ESP_LOGI(TAG, "UART initialized.");
    #endif
    
    #ifdef DEBUG
        ESP_LOGI(TAG, "Initializing motor PWM.");
    #endif
    // PWM for motor control
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_APB_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_CTRL_PIN,
        .duty = 0,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    #ifdef DEBUG
        ESP_LOGI(TAG, "PWM initialized.");
    #endif

    #ifdef DEBUG
        ESP_LOGI(TAG, "Starting up motor.");
    #endif
    set_motor_speed(100);
    ESP_LOGI(TAG, "Waiting for motor to reach constant rotation rate.");
    vTaskDelay(pdMS_TO_TICKS(3000)); // Critical wait because if the motor isnt spinning at a constant rate the next command fails and im too lazy to make proper code.
    #ifdef DEBUG
        ESP_LOGI(TAG, "Motor at constant speed.");
    #endif

    #ifdef DEBUG
        ESP_LOGI(TAG, "Starting scanning.");
    #endif
    int bytes_written =
        uart_write_bytes(UART_NUM, (const char *)START_SCAN_COMMAND,
                         START_SCAN_COMMAND_LEN);
    if (bytes_written != START_SCAN_COMMAND_LEN) {
        ESP_LOGE(TAG, "Failed to send START_SCAN command. Wrote %d of %zu bytes",
                 bytes_written, START_SCAN_COMMAND_LEN);
        return ESP_FAIL;
    }
    #ifdef DEBUG
        ESP_LOGI(TAG, "Started scanning.");
    #endif
    
    #ifdef DEBUG
        ESP_LOGI(TAG, "Validating startup.");
    #endif
    uint8_t data_byte;
    int pattern_idx = 0;
    uint8_t response_descriptor[7];
    TickType_t start_time = xTaskGetTickCount();

    while (pattern_idx < 7) {
        if ((xTaskGetTickCount() - start_time) > pdMS_TO_TICKS(10000)) {
            ESP_LOGW(TAG, "Timeout waiting for scan descriptor");
            return ESP_ERR_TIMEOUT;
        }

        int len = uart_read_bytes(UART_NUM, &data_byte, 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            ESP_LOGD(TAG, "Received byte: 0x%02X", data_byte);
            
            if (pattern_idx == 0 && data_byte == 0xA5) {
                response_descriptor[0] = data_byte;
                pattern_idx = 1;
            } else if (pattern_idx == 1 && data_byte == 0x5A) {
                response_descriptor[1] = data_byte;
                pattern_idx = 2;
            } else if (pattern_idx >= 2) {
                response_descriptor[pattern_idx] = data_byte;
                pattern_idx++;
            } else {
                pattern_idx = 0;
                if (data_byte == 0xA5) {
                    response_descriptor[0] = data_byte;
                    pattern_idx = 1;
                }
            }
        }
    }
    #ifdef DEBUG
        ESP_LOGI(TAG, "Scanning successfully.");
    #endif

    ESP_LOGI(TAG, "Initialized.");
    return ESP_OK;
}

// could maybe add debug logs to this but ehh should be fine
esp_err_t get_lidar_scan_data(uint8_t *buffer, size_t buffer_size, size_t *read_len) {
    if (buffer == NULL || read_len == NULL || buffer_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    *read_len = 0;
    uint8_t packet[5];
    bool found_first_start = false;
    size_t packet_count = 0;
    TickType_t start_time = xTaskGetTickCount();

    while (1) {
        if ((xTaskGetTickCount() - start_time) > pdMS_TO_TICKS(5000)) {
            ESP_LOGW(TAG, "Timeout waiting for scan data");
            return ESP_ERR_TIMEOUT;
        }

        int bytes_read = uart_read_bytes(UART_NUM, packet, 5, pdMS_TO_TICKS(100));
        
        if (bytes_read == 5) {
            bool is_start_of_scan = (packet[0] & 0x01) == 0x01;

            if (!found_first_start) {
                if (is_start_of_scan) {
                    found_first_start = true;
                    
                    if (*read_len + 5 <= buffer_size) {
                        memcpy(buffer + *read_len, packet, 5);
                        *read_len += 5;
                        packet_count++;
                    } else {
                        ESP_LOGW(TAG, "Buffer full");
                        return ESP_ERR_NO_MEM;
                    }
                }
            } else {
                if (is_start_of_scan) {
                    return ESP_OK;
                }

                if (*read_len + 5 <= buffer_size) {
                    memcpy(buffer + *read_len, packet, 5);
                    *read_len += 5;
                    packet_count++;
                } else {
                    ESP_LOGW(TAG, "Buffer full with %d packets", packet_count);
                    return ESP_ERR_NO_MEM;
                }
            }
        }
    }

    return ESP_OK;
}
