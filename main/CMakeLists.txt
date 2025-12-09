# Allow enabling/disabling debug mode easily
option(ENABLE_DEBUG "Enable debugging mode" OFF)

idf_component_register(SRCS "bluetooth_comm.c" "motor.c" "lidar.c" "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_spi esp_driver_uart esp_driver_gpio esp_driver_ledc nvs_flash bt)

# Define DEBUG only if ENABLE_DEBUG is ON
if(ENABLE_DEBUG)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DEBUG)
endif()