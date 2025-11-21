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
static const uint8_t RX_ADDRESS[5] = {'0', '0', '0', '0', '2'};

esp_err_t nrf24_init(void) {
    ESP_LOGI(TAG, "Initializing radio.");

    ESP_ERROR_CHECK(rf24_init(NRF24_CE_PIN, NRF24_CSN_PIN));
    rf24_stop_listening();

    vTaskDelay(pdMS_TO_TICKS(500));
    
    //! Keep in mind that these have to match exactly or shit hits the fan
    rf24_set_channel(NRF24_CHANNEL);
    rf24_set_payload_size(NRF24_PAYLOAD);
    rf24_set_data_rate(NRF24_DATARATE);
    rf24_set_power_level(NRF24_POWER);
    rf24_set_retries(NRF24_RETRYDELAY, NRF24_RETRYCOUNT);

    rf24_open_writing_pipe(TX_ADDRESS);
    rf24_open_reading_pipe(1, RX_ADDRESS);

    rf24_stop_listening();

    #ifdef DEBUG
        rf24_print_details();
    #endif
    
    ESP_LOGI(TAG, "Radio initialized.");
    return ESP_OK;
}

esp_err_t nrf24_send(const uint8_t *data, size_t length) {
    if (!data || length == 0) return ESP_ERR_INVALID_ARG;

    rf24_stop_listening();

    #ifdef DEBUG
        ESP_LOGI(TAG, "Transmitting %d bytes...", (int)length);
    #endif
    esp_err_t ret = rf24_write(data, length);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Transmission failed: %s", esp_err_to_name(ret));
    }
    #ifdef DEBUG
        ESP_LOGI(TAG, "Transmission successful.");
    #endif

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
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Receiving data failed.");
    }
    #ifdef DEBUG
        ESP_LOGI(TAG, "Received %d bytes successfully", (int)length);
    #endif

    rf24_stop_listening();

    return ret;
}