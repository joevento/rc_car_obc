#ifndef MOTOR_H
#define MOTOR_H

#include "esp_err.h"
#include <stdbool.h>

esp_err_t motor_init();
esp_err_t motorA_control(int speed, bool direction);
esp_err_t motorB_control(int speed, bool direction);

#endif