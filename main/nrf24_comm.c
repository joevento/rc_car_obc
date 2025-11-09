#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

#define TAG "NRF24_TEST"

// NRF24L01 Register Map
#define NRF24_REG_CONFIG      0x00
#define NRF24_REG_STATUS      0x07
#define NRF24_REG_TX_ADDR     0x10

// NRF24L01 Command Set
#define NRF24_CMD_R_REGISTER  0x00
#define NRF24_CMD_W_REGISTER  0x20
#define NRF24_CMD_NOP         0xFF

// GPIO Pins
#define PIN_MISO    19
#define PIN_MOSI    23
#define PIN_SCLK    18
#define PIN_CE      21
#define PIN_CSN     5

// SPI Host
#define HOST_ID     SPI2_HOST

// Function prototypes
void ce_high(void);
void ce_low(void);
void csn_high(void);
void csn_low(void);
uint8_t spi_transfer_byte(spi_device_handle_t spi, uint8_t data);
void read_register(spi_device_handle_t spi, uint8_t reg, uint8_t* data, uint8_t len);
void write_register(spi_device_handle_t spi, uint8_t reg, const uint8_t* data, uint8_t len);
uint8_t get_status(spi_device_handle_t spi);

void app_main(void) {
    ESP_LOGI(TAG, "NRF24L01 Basic SPI Test");
    
    // Initialize GPIO pins
    ESP_LOGI(TAG, "Initializing GPIOs");
    gpio_reset_pin(PIN_CE);
    gpio_set_direction(PIN_CE, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_CE, 0);
    
    gpio_reset_pin(PIN_CSN);
    gpio_set_direction(PIN_CSN, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_CSN, 1);
    
    // Initialize SPI bus
    ESP_LOGI(TAG, "Initializing SPI bus");
    spi_bus_config_t bus_config = {
        .miso_io_num = PIN_MISO,
        .mosi_io_num = PIN_MOSI,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    
    esp_err_t ret = spi_bus_initialize(HOST_ID, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %d", ret);
        return;
    }
    ESP_LOGI(TAG, "SPI bus initialized");
    
    // Configure SPI device - try different settings
    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = 500000, // Super slow 1MHz for maximum stability
        .mode = 0,                 // SPI mode 0
        .queue_size = 1,
        .flags = 0,
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .spics_io_num = PIN_CSN,
    };
    
    spi_device_handle_t spi;
    ret = spi_bus_add_device(HOST_ID, &dev_config, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %d", ret);
        spi_bus_free(HOST_ID);
        return;
    }
    
    // Give the NRF24 time to power up fully
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // Read STATUS register (should be 0x0E after power-up)
    uint8_t status = get_status(spi);
    ESP_LOGI(TAG, "STATUS Register = 0x%02X (Expected 0x0E after power-up)", status);
    
    // Read CONFIG register (should be 0x08 after power-up)
    uint8_t config_val;
    read_register(spi, NRF24_REG_CONFIG, &config_val, 1);
    ESP_LOGI(TAG, "CONFIG Register = 0x%02X (Expected 0x08 after power-up)", config_val);
    
    // Try to write and read back a test value to CONFIG
    uint8_t test_config = 0x0C; // Enable CRC, 2-byte CRC
    ESP_LOGI(TAG, "Writing 0x%02X to CONFIG register", test_config);
    write_register(spi, NRF24_REG_CONFIG, &test_config, 1);
    
    // Read back CONFIG to verify
    read_register(spi, NRF24_REG_CONFIG, &config_val, 1);
    ESP_LOGI(TAG, "Read back CONFIG = 0x%02X (Expected 0x0C)", config_val);
    
    // Try to write and read back TX_ADDR
    uint8_t test_addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    ESP_LOGI(TAG, "Writing test address to TX_ADDR");
    write_register(spi, NRF24_REG_TX_ADDR, test_addr, 5);
    
    // Read back TX_ADDR
    uint8_t addr_read[5];
    read_register(spi, NRF24_REG_TX_ADDR, addr_read, 5);
    
    ESP_LOGI(TAG, "Read back TX_ADDR = 0x%02X%02X%02X%02X%02X (Expected 0xE7E7E7E7E7)",
             addr_read[0], addr_read[1], addr_read[2], addr_read[3], addr_read[4]);
    
    // Verify byte by byte
    bool addr_match = true;
    for (int i = 0; i < 5; i++) {
        if (addr_read[i] != test_addr[i]) {
            ESP_LOGE(TAG, "Address mismatch at byte %d: wrote 0x%02X, read 0x%02X",
                     i, test_addr[i], addr_read[i]);
            addr_match = false;
        }
    }
    
    if (addr_match) {
        ESP_LOGI(TAG, "Address verification PASSED!");
    } else {
        ESP_LOGE(TAG, "Address verification FAILED!");
    }
    
    // Check if we can communicate by comparing CONFIG and STATUS values
    if ((status == 0x0E || status == 0x0F) && (config_val == test_config)) {
        ESP_LOGI(TAG, "Basic SPI communication test PASSED!");
    } else {
        ESP_LOGE(TAG, "Basic SPI communication test FAILED!");
        ESP_LOGE(TAG, "This suggests a hardware issue or SPI configuration problem.");
    }
}

// CE control
void ce_high(void) {
    gpio_set_level(PIN_CE, 1);
    esp_rom_delay_us(15);
}

void ce_low(void) {
    gpio_set_level(PIN_CE, 0);
    esp_rom_delay_us(15);
}

// CSN control
void csn_high(void) {
    gpio_set_level(PIN_CSN, 1);
    esp_rom_delay_us(15);
}

void csn_low(void) {
    gpio_set_level(PIN_CSN, 0);
    esp_rom_delay_us(15);
}

// Basic SPI transfer
uint8_t spi_transfer_byte(spi_device_handle_t spi, uint8_t data) {
    uint8_t rx_data = 0;
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
        .rx_buffer = &rx_data,
        .flags = 0
    };
    
    esp_err_t ret = spi_device_transmit(spi, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transfer failed: %d", ret);
    }
    
    return rx_data;
}

// Get status register
uint8_t get_status(spi_device_handle_t spi) {
    csn_low();
    uint8_t status = spi_transfer_byte(spi, NRF24_CMD_NOP);
    csn_high();
    return status;
}

// Read register
void read_register(spi_device_handle_t spi, uint8_t reg, uint8_t* data, uint8_t len) {
    csn_low();
    spi_transfer_byte(spi, NRF24_CMD_R_REGISTER | (reg & 0x1F));
    
    for (uint8_t i = 0; i < len; i++) {
        data[i] = spi_transfer_byte(spi, 0xFF); // Send dummy byte to read
    }
    
    csn_high();
}

// Write register
void write_register(spi_device_handle_t spi, uint8_t reg, const uint8_t* data, uint8_t len) {
    csn_low();
    spi_transfer_byte(spi, NRF24_CMD_W_REGISTER | (reg & 0x1F));
    
    for (uint8_t i = 0; i < len; i++) {
        spi_transfer_byte(spi, data[i]);
    }
    
    csn_high();
}