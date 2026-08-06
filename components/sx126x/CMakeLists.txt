idf_component_register(
    SRCS "sx126x.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_driver_spi esp_driver_gpio
)
