#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "motor.h"

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

static const char *TAG = "Motor";

esp_err_t motor_init() {
	ESP_LOGI(TAG, "Initializing motor driver.");
	
    #ifdef DEBUG
        ESP_LOGI(TAG, "Init gpio for STBY.");
    #endif
	gpio_config_t io_conf = {};
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask =
		(1ULL << AIN1_PIN) | (1ULL << AIN2_PIN) | (1ULL << BIN1_PIN) |
		(1ULL << BIN2_PIN) | (1ULL << STBY_PIN);
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

	// Set standby pin high to enable the motor driver
	ESP_ERROR_CHECK(gpio_set_level(STBY_PIN, 1));
	ESP_ERROR_CHECK(gpio_config(&io_conf));
	#ifdef DEBUG
		ESP_LOGI(TAG, "STBY initialized");
	#endif

    #ifdef DEBUG
        ESP_LOGI(TAG, "Initializing motor PWM.");
    #endif
	// Configure LEDC timer for PWM
	ledc_timer_config_t ledc_timer = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.timer_num = LEDC_TIMER_0,
		.duty_resolution = resolution,
		.freq_hz = freq,
		.clk_cfg = LEDC_AUTO_CLK,
	};
	ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    #ifdef DEBUG
        ESP_LOGI(TAG, "Configuring motor A channel.");
    #endif
	ledc_channel_config_t ledc_channel_a = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = motorAChannel,
		.timer_sel = LEDC_TIMER_0,
		.intr_type = LEDC_INTR_DISABLE,
		.gpio_num = PWMA_PIN,
		.duty = 0,
		.hpoint = 0,
	};
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_a));

    #ifdef DEBUG
        ESP_LOGI(TAG, "Configuring motor B channel.");
    #endif
	ledc_channel_config_t ledc_channel_b = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = motorBChannel,
		.timer_sel = LEDC_TIMER_0,
		.intr_type = LEDC_INTR_DISABLE,
		.gpio_num = PWMB_PIN,
		.duty = 0,
		.hpoint = 0,
	};
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_b));
    #ifdef DEBUG
        ESP_LOGI(TAG, "Channels configured.");
    #endif

    ESP_LOGI(TAG, "Controller initialized.");
	return ESP_OK;
}

esp_err_t motorA_control(int speed, bool direction) {

	ledc_set_duty(LEDC_LOW_SPEED_MODE, motorAChannel, speed);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, motorAChannel);

	if (direction) { // Forward
		gpio_set_level(AIN1_PIN, 1);
		gpio_set_level(AIN2_PIN, 0);
	} else { // Reverse
		gpio_set_level(AIN1_PIN, 0);
		gpio_set_level(AIN2_PIN, 1);
	}

	return ESP_OK;
}

esp_err_t motorB_control(int speed, bool direction) {

	ledc_set_duty(LEDC_LOW_SPEED_MODE, motorBChannel, speed);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, motorBChannel);

	if (direction) { // Forward
		gpio_set_level(BIN1_PIN, 1);
		gpio_set_level(BIN2_PIN, 0);
	} else { // Reverse
		gpio_set_level(BIN1_PIN, 0);
		gpio_set_level(BIN2_PIN, 1);
	}

	return ESP_OK;
}
