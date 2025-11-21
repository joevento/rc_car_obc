#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lidar.h"
#include "nrf24_comm.h"
#include "rf24_wrapper.h"
#include "motor.h"

static const char *TAG = "OBC_MAIN";

// Motor control commands
typedef struct {
    uint8_t motor_a_speed;
    bool motor_a_direction;
    uint8_t motor_b_speed;
    bool motor_b_direction;
} motor_command_t;

// Lidar things
typedef struct {
    uint8_t data[2048]; 
    size_t length;
} lidar_packet_t;

static QueueHandle_t lidar_data_queue = NULL;

// Lidar data fragments
typedef struct {
    uint8_t fragment_id; // Unique ID for each fragment within a full scan
    bool is_last_fragment; // True if this is the last fragment of the scan
    uint8_t fragment_data[32 - sizeof(uint8_t) - sizeof(bool)]; // Actual data
} nrf_fragment_t;

void lidar_scan_task(void *pvParameters) {
    lidar_packet_t lidar_packet;

    while (1) {
        esp_err_t err = get_lidar_scan_data(lidar_packet.data, sizeof(lidar_packet.data), &lidar_packet.length);

        if (err == ESP_OK && lidar_packet.length > 0) {
            #ifdef DEBUG
                ESP_LOGI(TAG, "Lidar Scan Data (%u bytes) acquired. Sending to queue.", lidar_packet.length);
            #endif
            // Send the acquired LIDAR packet to the queue
            if (xQueueSend(lidar_data_queue, &lidar_packet, portMAX_DELAY) != pdPASS) {
                ESP_LOGE(TAG, "Failed to send LIDAR data to queue.");
            }
        } else if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error getting lidar scan data: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}

static void lidar_init_handler(void){
    ESP_LOGI(TAG, "Creating data queue for lidar data.");
    lidar_data_queue = xQueueCreate(5, sizeof(lidar_packet_t)); // Queue can hold 5 LIDAR packets
    if (lidar_data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create LIDAR data queue");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    lidar_init();
    
    // Create a FreeRTOS task for LIDAR scanning
    // Pin to core 0, as core 1 might be used by Wi-Fi or other critical tasks
    xTaskCreatePinnedToCore(lidar_scan_task, "LidarScanTask", 4096, NULL, 5, NULL, 0);
}

static void debugWheels(void){
    ESP_LOGI(TAG, "Motor A Forward, Motor B Forward");
    motorA_control(300, true);
    motorB_control(300, true);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Motor A Reverse, Motor B Reverse");
    motorA_control(300, false);
    motorB_control(300, false);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Motor A Stop, Motor B Stop");
    motorA_control(0, true);
    motorB_control(0, true);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Motor A Forward, Motor B Reverse");
    motorA_control(150, true);
    motorB_control(150, false);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Motor A Reverse, Motor B Forward");
    motorA_control(150, false);
    motorB_control(150, true);
    vTaskDelay(pdMS_TO_TICKS(2000));
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting Car OBC Main Application");
    #ifdef DEBUG
        ESP_LOGE(TAG, "===== Debug mode enabled =====");
    #endif

    // Inits:
    motor_init();
    nrf24_init();
    lidar_init_handler();

    ESP_LOGI(TAG, "All inits successful.");
    // Buffer for received motor commands
    motor_command_t received_command;
    // Holds the full LIDAR scan to be fragmented
    lidar_packet_t current_lidar_scan;

    // Main loop
    while (1) {
        // Motor Control Inputs
        #ifdef DEBUG //TODO: in final code remove this ifdef
            debugWheels();
        #else
            esp_err_t recv_ret = nrf24_receive((uint8_t *)&received_command, sizeof(motor_command_t));
            if (recv_ret == ESP_OK) {
                ESP_LOGI(TAG, "Received motor command: A_Speed=%u, A_Dir=%d, B_Speed=%u, B_Dir=%d",
                        received_command.motor_a_speed, received_command.motor_a_direction,
                        received_command.motor_b_speed, received_command.motor_b_direction);

                // Apply received motor commands
                motorA_control(received_command.motor_a_speed, received_command.motor_a_direction);
                motorB_control(received_command.motor_b_speed, received_command.motor_b_direction);
            } else if (recv_ret != ESP_ERR_NOT_FOUND) {
                ESP_LOGW(TAG, "Error receiving NRF24 data: %s", esp_err_to_name(recv_ret));
            }
        #endif

        // Transmission of LIDAR data
        // Check if there is new LIDAR data available in the queue
        if (xQueueReceive(lidar_data_queue, &current_lidar_scan, (TickType_t)0) == pdPASS) {
            #ifdef DEBUG
                ESP_LOGI(TAG, "Processing LIDAR data (%u bytes) for fragmentation and broadcast...", current_lidar_scan.length);
            #endif

            const size_t fragment_data_payload_size = 32 - sizeof(uint8_t) - sizeof(bool);
            size_t bytes_sent = 0;
            uint8_t fragment_idx = 0;

            while (bytes_sent < current_lidar_scan.length) {
                nrf_fragment_t nrf_fragment;
                nrf_fragment.fragment_id = fragment_idx;

                size_t bytes_to_copy = current_lidar_scan.length - bytes_sent;
                if (bytes_to_copy > fragment_data_payload_size) {
                    bytes_to_copy = fragment_data_payload_size;
                    nrf_fragment.is_last_fragment = false;
                } else {
                    nrf_fragment.is_last_fragment = true;
                }

                memcpy(nrf_fragment.fragment_data, current_lidar_scan.data + bytes_sent, bytes_to_copy);

                // Zero-fill
                if (bytes_to_copy < fragment_data_payload_size) {
                    memset(nrf_fragment.fragment_data + bytes_to_copy, 0, fragment_data_payload_size - bytes_to_copy);
                }

                #ifdef DEBUG
                    ESP_LOGD(TAG, "Sending fragment %u (last: %d, bytes: %zu/%zu)", 
                            nrf_fragment.fragment_id, nrf_fragment.is_last_fragment, bytes_to_copy, fragment_data_payload_size);
                #endif

                esp_err_t nrf_send_err = nrf24_send((const uint8_t *)&nrf_fragment, sizeof(nrf_fragment_t));
                if (nrf_send_err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to broadcast LIDAR fragment %u: %s", fragment_idx, esp_err_to_name(nrf_send_err));
                }

                bytes_sent += bytes_to_copy;
                fragment_idx++;
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            #ifdef DEBUG
                ESP_LOGI(TAG, "Finished broadcasting LIDAR scan in %u fragments.", fragment_idx);
            #endif
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
