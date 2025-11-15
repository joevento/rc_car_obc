#include "nrf24_comm.h"
#include "rf24_wrapper.h"
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define NRF24_CE_PIN   21
#define NRF24_CSN_PIN  5
#define NRF24_CHANNEL  90
#define NRF24_PAYLOAD  32

static const char *TAG = "NRF24_COMM";

static const uint8_t TX_ADDRESS[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
static const uint8_t RX_ADDRESS[5] = {0xD2, 0xD2, 0xD2, 0xD2, 0xD2};

esp_err_t nrf24_init(void) {
    ESP_LOGI(TAG, "Initializing NRF24...");

    esp_err_t ret = rf24_init(NRF24_CE_PIN, NRF24_CSN_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "rf24_init failed: %d", ret);
        return ret;
    }
    rf24_stop_listening();

    vTaskDelay(pdMS_TO_TICKS(1000));
    rf24_set_channel(NRF24_CHANNEL);
    rf24_set_payload_size(NRF24_PAYLOAD);
    rf24_set_data_rate(RF24W_2MBPS);
    //rf24_set_power_level(RF24W_PA_MIN);
    rf24_set_retries(5, 15);

    // Use pipe 1 for receive, base pipe for transmit
    rf24_open_writing_pipe(TX_ADDRESS);
    rf24_open_reading_pipe(1, RX_ADDRESS);

    // Default to TX mode; use start_listening() when receiving
    rf24_stop_listening();

    rf24_print_details();
    ESP_LOGI(TAG, "NRF24 initialized successfully.");
    return ESP_OK;
}

esp_err_t nrf24_send(const uint8_t *data, size_t length) {
    if (!data || length == 0) return ESP_ERR_INVALID_ARG;

    rf24_stop_listening();

    ESP_LOGI(TAG, "Transmitting %d bytes...", (int)length);
    esp_err_t ret = rf24_write(data, length);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Transmission successful");
    } else {
        ESP_LOGW(TAG, "Transmission failed: %s", esp_err_to_name(ret));
    }

    rf24_start_listening();
    return ret;
}

esp_err_t nrf24_receive(uint8_t *data, size_t length) {
    if (!data || length == 0) return ESP_ERR_INVALID_ARG;

    rf24_start_listening();

    if (rf24_available() != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t ret = rf24_read(data, length);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Received %d bytes successfully", (int)length);
    } else {
        ESP_LOGW(TAG, "Read failed");
    }

    rf24_stop_listening();

    return ret;
}