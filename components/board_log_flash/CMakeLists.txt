idf_component_register(
    SRCS "src/board_log_flash.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_partition freertos
    PRIV_REQUIRES esp_hw_support
)
