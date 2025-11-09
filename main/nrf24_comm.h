#ifndef NRF24_COMM_H_
#define NRF24_COMM_H_

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// NRF24L01 Configuration
typedef struct {
    // SPI Configuration
    spi_host_device_t spi_host;    // SPI host to use (SPI2_HOST or SPI3_HOST)
    int miso_gpio;                 // GPIO for MISO
    int mosi_gpio;                 // GPIO for MOSI
    int sclk_gpio;                 // GPIO for SCLK
    int ce_gpio;                   // GPIO for CE (Chip Enable)
    int csn_gpio;                  // GPIO for CSN (Chip Select Not)
    
    // Radio Configuration
    uint8_t channel;               // RF Channel (0-125)
    uint8_t payload_size;          // Size of payload (1-32 bytes)
    uint8_t tx_address[5];         // Transmit address (5 bytes)
    uint8_t rx_address[5];         // Receive address (5 bytes)
    
    // Internal state - don't modify directly
    spi_device_handle_t spi_handle;
    bool is_initialized;
    bool is_transmitting;
} nrf24_t;

// Data rate options
typedef enum {
    NRF24_DR_1MBPS = 0,
    NRF24_DR_2MBPS,
    NRF24_DR_250KBPS
} nrf24_datarate_t;

// Power amplifier level options
typedef enum {
    NRF24_PA_MIN = 0,
    NRF24_PA_LOW,
    NRF24_PA_HIGH,
    NRF24_PA_MAX
} nrf24_power_t;

/**
 * Initialize the NRF24L01 module
 * 
 * @param nrf24 Pointer to nrf24_t structure with required configuration
 * @return ESP_OK if successful, appropriate error code otherwise
 */
esp_err_t nrf24_init(nrf24_t *nrf24);

/**
 * Send data via NRF24L01
 * 
 * @param nrf24 Pointer to initialized nrf24_t structure
 * @param data Data to send (must be at least payload_size bytes)
 * @param timeout_ms Timeout in milliseconds to wait for completion
 * @return ESP_OK if successful, appropriate error code otherwise
 */
esp_err_t nrf24_send(nrf24_t *nrf24, const uint8_t *data, uint32_t timeout_ms);

/**
 * Receive data via NRF24L01
 * 
 * @param nrf24 Pointer to initialized nrf24_t structure
 * @param data Buffer to store received data (must be at least payload_size bytes)
 * @param timeout_ms Timeout in milliseconds to wait for data
 * @return ESP_OK if data received, ESP_ERR_TIMEOUT if timed out
 */
esp_err_t nrf24_receive(nrf24_t *nrf24, uint8_t *data, uint32_t timeout_ms);

/**
 * Set RF channel (frequency)
 * 
 * @param nrf24 Pointer to initialized nrf24_t structure
 * @param channel Channel number (0-125)
 * @return ESP_OK if successful
 */
esp_err_t nrf24_set_channel(nrf24_t *nrf24, uint8_t channel);

/**
 * Set data rate
 * 
 * @param nrf24 Pointer to initialized nrf24_t structure
 * @param rate Data rate (NRF24_DR_1MBPS, NRF24_DR_2MBPS, NRF24_DR_250KBPS)
 * @return ESP_OK if successful
 */
esp_err_t nrf24_set_datarate(nrf24_t *nrf24, nrf24_datarate_t rate);

/**
 * Set power amplifier level
 * 
 * @param nrf24 Pointer to initialized nrf24_t structure
 * @param level Power level (NRF24_PA_MIN, NRF24_PA_LOW, NRF24_PA_HIGH, NRF24_PA_MAX)
 * @return ESP_OK if successful
 */
esp_err_t nrf24_set_power(nrf24_t *nrf24, nrf24_power_t level);

/**
 * Print NRF24L01 status and configuration for debugging
 * 
 * @param nrf24 Pointer to initialized nrf24_t structure
 */
void nrf24_print_details(nrf24_t *nrf24);

#ifdef __cplusplus
}
#endif

#endif /* NRF24_COMM_H_ */
