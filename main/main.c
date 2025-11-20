#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lidar.h"
#include "nrf24_comm.h"
#include "rf24_wrapper.h"
#include "motor.h"

static const char *TAG = "OBC_MAIN";

// debug mode flag
#define DEBUG

void app_main(void) {
    #ifdef DEBUG
        ESP_LOGI(TAG, "===== Debug mode enabled =====");
    #endif

    // Motor things
    motor_init();
    ESP_LOGI(TAG, "SparkFun Dual TB6612FNG ESP32 setup complete.");
    /*
    while (1) {
        ESP_LOGI(TAG, "Motor A Forward, Motor B Forward");
        motorA_control(200, true);
        motorB_control(150, true);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Motor A Reverse, Motor B Reverse");
        motorA_control(180, false);
        motorB_control(220, false);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Motor A Stop, Motor B Stop");
        motorA_control(0, true);
        motorB_control(0, true);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "Motor A Forward, Motor B Reverse");
        motorA_control(100, true);
        motorB_control(100, false);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Motor A Reverse, Motor B Forward");
        motorA_control(120, false);
        motorB_control(120, true);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }*/

    // Radio things
   esp_err_t ret = nrf24_init();
   if (ret != ESP_OK) {
       ESP_LOGE(TAG, "Failed to initialize nrf24");
       return;
    }
    ESP_LOGI(TAG, "Ready on channel %d...", rf24_get_channel());
    
    while (true) {
        // Generate test data
        uint8_t test_data[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
        
        // Send data
        esp_err_t result = nrf24_send(test_data, sizeof(test_data));
        
        if (result == ESP_OK) {
            ESP_LOGI(TAG, "Sent %d bytes successfully", sizeof(test_data));
        } else {
            ESP_LOGW(TAG, "Failed to send data: %s", esp_err_to_name(result));
        }
        
        // Wait before next send
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
    
    /*void app_main(void) {
        ESP_LOGI(TAG, "Starting Car OBC Main Application");
        
        if (init_lidar() != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize LIDAR");
            while (1) {
                vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Allocate buffer for lidar scan data
    uint8_t *scan_data = (uint8_t *)malloc(2048); // 2KB buffer, actual size is like 1.5KB but ye

    if (scan_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer for scan data");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    while (1) {
        size_t data_len = 0;
        esp_err_t err = get_lidar_scan_data(scan_data, 2048, &data_len);
        
        if (err == ESP_OK && data_len > 0) {
            ESP_LOGI(TAG, "Lidar Scan Data (%u bytes):", data_len);
            // Only print first 100 bytes to avoid flooding the log
            for (int i = 0; i < (data_len > 100 ? 100 : data_len); i++) {
                printf("%02X ", scan_data[i]);
            }
            if (data_len > 100) {
                printf("... (truncated)\n");
            } else {
                printf("\n");
            }
        } else if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error getting lidar scan data: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
*/