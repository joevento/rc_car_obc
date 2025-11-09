set(component_srcs "mirf.c")

idf_component_register(SRCS "${component_srcs}"
                       REQUIRES esp_driver_spi esp_driver_gpio
                       INCLUDE_DIRS ".")