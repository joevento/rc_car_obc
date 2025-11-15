#ifndef RF24_WRAPPER_H
#define RF24_WRAPPER_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RF24W_1MBPS,
    RF24W_2MBPS,
    RF24W_250KBPS
} rf24_datarate_t;

typedef enum {
    RF24W_PA_MIN,
    RF24W_PA_LOW,
    RF24W_PA_HIGH,
    RF24W_PA_MAX
} rf24_powerlevel_t;

// --- Initialization and Basic Control ---
esp_err_t rf24_init(int ce_pin, int csn_pin);
esp_err_t rf24_power_up(void);
esp_err_t rf24_power_down(void);
bool rf24_is_chip_connected(void);

// --- Configuration ---
esp_err_t rf24_set_channel(uint8_t channel);
esp_err_t rf24_set_payload_size(uint8_t size);
esp_err_t rf24_set_retries(uint8_t delay, uint8_t count);
esp_err_t rf24_set_data_rate(rf24_datarate_t rate);
esp_err_t rf24_set_power_level(rf24_powerlevel_t level);

// --- Pipes ---
esp_err_t rf24_open_writing_pipe(const uint8_t *address);
esp_err_t rf24_open_reading_pipe(uint8_t number, const uint8_t *address);

// --- Data Transmission ---
esp_err_t rf24_write(const uint8_t *data, size_t length);
esp_err_t rf24_available(void);
esp_err_t rf24_read(uint8_t *data, size_t length);

// --- Listening Mode ---
esp_err_t rf24_start_listening(void);
esp_err_t rf24_stop_listening(void);

// --- Maintenance ---
esp_err_t rf24_flush_tx(void);
esp_err_t rf24_flush_rx(void);

// --- Debugging ---
void rf24_print_details(void);

#ifdef __cplusplus
}
#endif

#endif  // RF24_WRAPPER_H
