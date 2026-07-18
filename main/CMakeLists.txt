idf_component_register(
    SRCS
        "wifi_watch.c"
        "app/app_boot.c"
        "app/app_wifi.c"
        "app/app_alert_state.c"
        "app/app_alert.c"
        "app/app_weather.c"
        "app/app_display.c"
    INCLUDE_DIRS
        "."
    EMBED_TXTFILES
        "certs/alerts_globalsign_root_ca.pem"

    REQUIRES
        driver esp_lcd spiffs esp_timer esp_psram esp_wifi nvs_flash esp_adc
        esp_http_client mbedtls cjson
)

