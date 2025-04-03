idf_component_register(
    SRCS
        "wifi_watch.c"
        "app/app_wifi.c"
        "app/app_display.c"
    INCLUDE_DIRS
        "."
    
    REQUIRES
        driver esp_lcd spiffs esp_timer esp_psram esp_wifi nvs_flash esp_adc
)

