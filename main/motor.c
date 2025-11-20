/* 
Ai1 = 12
Ai2 = 14
PWMA = 34

Bi1 = 26
Bi2 = 25
PWMB = 35

standby = 27
*/

// motor.c
// components/motor/motor.c
#include "motor.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

// Motor A
#define AIN1_PIN 12
#define AIN2_PIN 14
#define PWMA_PIN 32

// Motor B
#define BIN1_PIN 26
#define BIN2_PIN 25
#define PWMB_PIN 33

// Standby pin
#define STBY_PIN 27

// PWM settings
const int freq = 30000;
const int resolution = 8;
const ledc_channel_t motorAChannel = LEDC_CHANNEL_0;
const ledc_channel_t motorBChannel = LEDC_CHANNEL_1;

esp_err_t motor_init() {
	// Configure GPIOs for motor control
	gpio_config_t io_conf = {};
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask =
		(1ULL << AIN1_PIN) | (1ULL << AIN2_PIN) | (1ULL << BIN1_PIN) |
		(1ULL << BIN2_PIN) | (1ULL << STBY_PIN);
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

	// Set standby pin high to enable the motor driver
	gpio_set_level(STBY_PIN, 1);

	// Configure LEDC for PWM
	ledc_timer_config_t ledc_timer = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.timer_num = LEDC_TIMER_0,
		.duty_resolution = resolution,
		.freq_hz = freq,
		.clk_cfg = LEDC_AUTO_CLK,
	};
	ledc_timer_config(&ledc_timer);

	ledc_channel_config_t ledc_channel_a = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = motorAChannel,
		.timer_sel = LEDC_TIMER_0,
		.intr_type = LEDC_INTR_DISABLE,
		.gpio_num = PWMA_PIN,
		.duty = 0,
		.hpoint = 0,
	};
	ledc_channel_config(&ledc_channel_a);

	ledc_channel_config_t ledc_channel_b = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = motorBChannel,
		.timer_sel = LEDC_TIMER_0,
		.intr_type = LEDC_INTR_DISABLE,
		.gpio_num = PWMB_PIN,
		.duty = 0,
		.hpoint = 0,
	};
	ledc_channel_config(&ledc_channel_b);
}

void motorA_control(int speed, bool direction) {

	ledc_set_duty(LEDC_LOW_SPEED_MODE, motorAChannel, speed);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, motorAChannel);

	if (direction) { // Forward
		gpio_set_level(AIN1_PIN, 1);
		gpio_set_level(AIN2_PIN, 0);
	} else { // Reverse
		gpio_set_level(AIN1_PIN, 0);
		gpio_set_level(AIN2_PIN, 1);
	}
}

void motorB_control(int speed, bool direction) {

	ledc_set_duty(LEDC_LOW_SPEED_MODE, motorBChannel, speed);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, motorBChannel);

	if (direction) { // Forward
		gpio_set_level(BIN1_PIN, 1);
		gpio_set_level(BIN2_PIN, 0);
	} else { // Reverse
		gpio_set_level(BIN1_PIN, 0);
		gpio_set_level(BIN2_PIN, 1);
	}
}
