idf_component_register(SRCS "nrf24_comm.c" "motor.c" "lidar.c" "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_spi esp_driver_uart esp_driver_gpio esp_driver_ledc)