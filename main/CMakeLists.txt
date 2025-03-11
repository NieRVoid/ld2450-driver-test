idf_component_register(
    SRCS "ld2450_example.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash driver esp_timer ld2450-driver console
    PRIV_REQUIRES spi_flash
)
