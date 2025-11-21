#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nrf24_comm.h"
#include "rf24_wrapper.h"

#define NRF24_CE_PIN   21
#define NRF24_CSN_PIN  5
#define NRF24_CHANNEL  76
#define NRF24_PAYLOAD  32
#define NRF24_DATARATE RF24W_2MBPS
#define NRF24_POWER RF24W_PA_MAX
#define NRF24_RETRYDELAY 5
#define NRF24_RETRYCOUNT 15

static const char *TAG = "NRF24";

// Probably choose something better but these are fine for now ig
static const uint8_t TX_ADDRESS[5] = {'0', '0', '0', '0', '1'};

__attribute__((weak)) void nrf24_on_ack_payload(const uint8_t *data, size_t length) {
    (void)data;
    (void)length;
}

esp_err_t nrf24_init(void) {
    ESP_LOGI(TAG, "Initializing radio.");

    ESP_ERROR_CHECK(rf24_init(NRF24_CE_PIN, NRF24_CSN_PIN));
    rf24_stop_listening();

    vTaskDelay(pdMS_TO_TICKS(100));

    //! Keep in mind that these have to match exactly or shit hits the fan
    rf24_set_channel(NRF24_CHANNEL);
    rf24_set_payload_size(NRF24_PAYLOAD);
    rf24_set_data_rate(NRF24_DATARATE);
    rf24_set_power_level(NRF24_POWER);
    rf24_set_retries(NRF24_RETRYDELAY, NRF24_RETRYCOUNT);

    // Enable dynamic payloads and ACK payloads for bidirectional control in TX-only mode
    rf24_enable_dynamic_payloads();
    rf24_enable_ack_payload();

    rf24_open_writing_pipe(TX_ADDRESS);

    // Keep this node in TX mode permanently
    rf24_stop_listening();

    #ifdef DEBUG
        rf24_print_details();
    #endif
    
    ESP_LOGI(TAG, "Radio initialized.");
    return ESP_OK;
}

// Send and harvest motor command from ACK payload if available
esp_err_t nrf24_send(const uint8_t *data, size_t length) {
    if (!data || length == 0) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = rf24_write(data, length);
    if (ret != ESP_OK) {
        rf24_flush_tx();
        return ret;
    }

    uint8_t ack_buf[32];
    if (rf24_is_ack_payload_available()) {
        if (rf24_read_ack_payload(ack_buf, sizeof(ack_buf)) == ESP_OK) {
            nrf24_on_ack_payload(ack_buf, sizeof(ack_buf));
        }
    }
    return ESP_OK;
}

esp_err_t nrf24_receive(uint8_t *data, size_t length) {
    // Not used in ACK payload approach; keep for compatibility if needed
    if (!data || length == 0) return ESP_ERR_INVALID_ARG;
    return ESP_ERR_NOT_SUPPORTED;
}