#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lidar.h"
#include "nrf24_comm.h"

static const char *TAG = "OBC_MAIN";

void app_main(void) {
    // Create and configure NRF24 structure
    nrf24_t nrf24 = {
        // SPI Configuration
        .spi_host = SPI3_HOST,     // VSPI on ESP32
        .miso_gpio = 19,           // From your menuconfig
        .mosi_gpio = 23,           // From your menuconfig
        .sclk_gpio = 18,           // From your menuconfig
        .ce_gpio = 21,             // From your menuconfig
        .csn_gpio = 5,             // From your menuconfig
        
        // Radio Configuration
        .channel = 90,             // RF Channel (0-125)
        .payload_size = 16,        // Size of payload (1-32 bytes)
        .tx_address = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7},  // Match receiver
        .rx_address = {0xD2, 0xD2, 0xD2, 0xD2, 0xD2}   // For receiving (if needed)
    };
    
    // Initialize NRF24L01 module
    esp_err_t ret = nrf24_init(&nrf24);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NRF24L01: %d", ret);
        return;
    }
    
    // Print NRF24L01 details
    nrf24_print_details(&nrf24);
    
    // Set maximum power
    nrf24_set_power(&nrf24, NRF24_PA_MAX);
    
    // Test message
    char message[] = "Hello from ESP32!";
    uint8_t tx_buffer[nrf24.payload_size];
    
    // Fill buffer with zeros
    memset(tx_buffer, 0, nrf24.payload_size);
    
    // Copy message into buffer (truncate if necessary)
    size_t message_len = strlen(message);
    if (message_len > nrf24.payload_size) {
        message_len = nrf24.payload_size;
    }
    memcpy(tx_buffer, message, message_len);
    
    // Main loop to send message every second
    while (1) {
        ESP_LOGI(TAG, "Sending message: %s", message);
        
        ret = nrf24_send(&nrf24, tx_buffer, 1000);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Message sent successfully");
        } else if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Send timeout");
        } else {
            ESP_LOGE(TAG, "Failed to send message: %d", ret);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
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