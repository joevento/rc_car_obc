#ifndef NRF24_COMM_H
#define NRF24_COMM_H

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize NRF24 module and configure required addresses and parameters.
 * 
 * @return ESP_OK if successful, error code otherwise.
 */
esp_err_t nrf24_init(void);

/**
 * Send a data buffer via NRF24.
 * 
 * @param data Pointer to the data buffer to send.
 * @param length Number of bytes to send.
 * @return ESP_OK if transmission succeeded, ESP_FAIL otherwise.
 */
esp_err_t nrf24_send(const uint8_t *data, size_t length);

/**
 * Receive data via NRF24.
 * 
 * @param data Pointer to buffer where received data will be stored.
 * @param length Buffer size (in bytes).
 * @return ESP_OK if data was received, ESP_ERR_NOT_FOUND if no data available.
 */
esp_err_t nrf24_receive(uint8_t *data, size_t length);

// Hook to deliver ACK payloads (motor commands) up to application
void nrf24_on_ack_payload(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif // NRF24_COMM_H