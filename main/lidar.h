#ifndef RPLIDAR_H
#define RPLIDAR_H

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t lidar_init(void);
esp_err_t get_lidar_scan_data(uint8_t *buffer, size_t buffer_size,
                              size_t *read_len);

#ifdef __cplusplus
}
#endif

#endif // RPLIDAR_H