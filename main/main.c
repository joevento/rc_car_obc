#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lidar.h"
#include "nrf24_comm.h"
#include "rf24_wrapper.h"

static const char *TAG = "OBC_MAIN";

void app_main(void) {
    esp_err_t ret = nrf24_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize nrf24");
        return;
    }
    // Change channel before starting to listen
    rf24_set_channel(108);
    rf24_print_details();

    uint8_t rx_buffer[32];
    ESP_LOGI(TAG, "Receiver ready on channel 108...");

    while (1) {
        esp_err_t ret = nrf24_receive(rx_buffer, sizeof(rx_buffer));

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Received %d bytes:", (int)sizeof(rx_buffer));
            for (int i = 0; i < sizeof(rx_buffer); i++) {
                printf("%02X ", rx_buffer[i]);
            }
            printf("\nAs text: %.*s\n", (int)sizeof(rx_buffer), (char *)rx_buffer);
        }
        ESP_LOGI(TAG, "%s", esp_err_to_name(ret));

        vTaskDelay(pdMS_TO_TICKS(50));
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