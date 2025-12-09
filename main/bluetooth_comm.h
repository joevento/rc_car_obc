#ifndef BLUETOOTH_COMM_H
#define BLUETOOTH_COMM_H

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bluetooth_init(void);
esp_err_t bluetooth_send(const uint8_t *data, size_t length);
bool bluetooth_is_connected(void);
void bluetooth_on_motor_command(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif
