#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lidar.h"
#include "bluetooth_comm.h"
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
    uint8_t magic;       // 0xA7
    uint8_t version;     // 0x01
    uint8_t scan_id;     // increments per full scan
    uint8_t fragment_id; // increments per fragment
    uint8_t payload_len; // 0..25
    uint16_t crc;        // CRC16-CCITT over header-with-zeroed-crc + payload
    uint8_t payload[25];
} __attribute__((packed)) rf_frame_t;

// CRC16-CCITT (poly 0x1021, init 0xFFFF, no reflect, no xorout)
static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static uint16_t compute_frame_crc(const rf_frame_t *fr) {
    rf_frame_t tmp = *fr;
    tmp.crc = 0;
    size_t len = offsetof(rf_frame_t, payload) + tmp.payload_len;
    return crc16_ccitt((const uint8_t *)&tmp, len);
}

void bluetooth_on_motor_command(const uint8_t *data, size_t length) {
    if (!data || length < sizeof(motor_command_t)) return;

    static motor_command_t last_cmd = {255, 0, 255, 0};
    motor_command_t cmd;
    memcpy(&cmd, data, sizeof(motor_command_t));

    if (memcmp(&cmd, &last_cmd, sizeof(cmd)) == 0) return;
    last_cmd = cmd;

    ESP_LOGI(TAG, "Motor cmd: A=%u dir=%d, B=%u dir=%d",
             cmd.motor_a_speed, cmd.motor_a_direction,
             cmd.motor_b_speed, cmd.motor_b_direction);

    motorA_control(cmd.motor_a_speed, cmd.motor_a_direction);
    motorB_control(cmd.motor_b_speed, cmd.motor_b_direction);
}

static lidar_packet_t lidar_scan_buffer;

void lidar_scan_task(void *pvParameters) {
    while (1) {
        esp_err_t err = get_lidar_scan_data(
            lidar_scan_buffer.data,
            sizeof(lidar_scan_buffer.data),
            &lidar_scan_buffer.length
        );

        if (err == ESP_OK && lidar_scan_buffer.length > 0) {
            #ifdef DEBUG
                ESP_LOGI(TAG, "Lidar Scan Data (%u bytes) acquired. Sending to queue.",
                         (unsigned)lidar_scan_buffer.length);
            #endif
            if (xQueueSend(lidar_data_queue, &lidar_scan_buffer, 0) != pdPASS) {
                // Drop oldest using the same buffer – no extra 2 kB on stack
                xQueueReceive(lidar_data_queue, &lidar_scan_buffer, 0);
                xQueueSend(lidar_data_queue, &lidar_scan_buffer, 0);
            }
        } else if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error getting lidar scan data: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
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
    // Pin to core 0, as core 1 might be used
    xTaskCreatePinnedToCore(lidar_scan_task, "LidarScanTask", 6144, NULL, 5, NULL, 0);
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting Car OBC Main Application");
    #ifdef DEBUG
        ESP_LOGE(TAG, "===== Debug mode enabled =====");
    #endif

    // Inits:
    motor_init();
    bluetooth_init();
    lidar_init_handler();

    ESP_LOGI(TAG, "All inits successful.");
    
    ESP_LOGI(TAG, "Waiting for Bluetooth connection...");
    while (!bluetooth_is_connected()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "Bluetooth connected. Starting data transmission.");

    // Holds the full LIDAR scan to be fragmented
    lidar_packet_t current_lidar_scan;
    static uint8_t scan_id = 0;

    uint16_t errorCount = 0;
    // Main loop
    while (1) {
        if (!bluetooth_is_connected()) {
            ESP_LOGW(TAG, "Bluetooth disconnected. Waiting for reconnection...");
            while (!bluetooth_is_connected()) {
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            ESP_LOGI(TAG, "Bluetooth reconnected.");
        }

        // Transmission of LIDAR data
        // Check if there is new LIDAR data available in the queue
        if (xQueueReceive(lidar_data_queue, &current_lidar_scan, 0) == pdPASS && errorCount <= 20) {
            #ifdef DEBUG
                ESP_LOGI(TAG,
                         "Processing LIDAR data (%u bytes) for framing and TX...", (unsigned)current_lidar_scan.length);
            #endif

            size_t bytes_sent = 0;
            uint8_t fragment_idx = 0;

            while (bytes_sent < current_lidar_scan.length && errorCount <= 20) {
                rf_frame_t f;
                memset(&f, 0, sizeof(f));

                f.magic = 0xA7;
                f.version = 0x01;
                f.scan_id = scan_id;
                f.fragment_id = fragment_idx;

                size_t remaining = current_lidar_scan.length - bytes_sent;
                size_t chunk = remaining > sizeof(f.payload) ? sizeof(f.payload)
                                                             : remaining;
                f.payload_len = (uint8_t)chunk;
                memcpy(f.payload,
                       current_lidar_scan.data + bytes_sent,
                       chunk);

                f.crc = compute_frame_crc(&f);

                esp_err_t bt_send_err = bluetooth_send((const uint8_t *)&f, sizeof(f));
                if (bt_send_err != ESP_OK) {
                    // Clear sticky state and back off a bit
                    vTaskDelay(pdMS_TO_TICKS(3));
                    errorCount += 1;
                } else {
                    // Small pace to keep receiver comfy
                    vTaskDelay(pdMS_TO_TICKS(2));
                }
                if (bt_send_err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to broadcast LIDAR fragment %u: %s",
                             fragment_idx, esp_err_to_name(bt_send_err));
                }

                bytes_sent += chunk;
                fragment_idx++;

                vTaskDelay(pdMS_TO_TICKS(1));
            }

            if (current_lidar_scan.length > 0) {
                scan_id++;
            }

            #ifdef DEBUG
                ESP_LOGI(TAG, "Finished broadcasting LIDAR scan in %u fragments.", fragment_idx);
            #endif
        }
        else if (errorCount >= 20) {
            motorA_control(0,false);
            motorB_control(0,false);
            ESP_LOGE(TAG, "Errored over 20 times entered failsafe state with 0 motor speed set.");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
