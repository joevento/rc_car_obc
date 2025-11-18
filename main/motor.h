#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>

void motor_init();
void motorA_control(int speed, bool direction);
void motorB_control(int speed, bool direction);

#endif