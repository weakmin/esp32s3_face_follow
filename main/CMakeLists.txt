idf_component_register(
    SRCS "app_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp32-camera esp-dl esp_driver_ledc
)
