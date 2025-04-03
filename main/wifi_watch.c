#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include <freertos/FreeRTOS.h>
#include <esp_timer.h>
#include <esp_log.h>
#include "app.h"
#include "app_config.h"

#define TAG "GP1287-APP"


static void IRAM_ATTR clock_timer_callback(void* arg)
{
    time_t now;
    time(&now);
    set_time(now);
}

void app_main()
{
    setenv("TZ", "EET-2EEST,M3.5.0/3,M10.5.0/4", 1);
    tzset();

    setup_display();
    
    esp_timer_create_args_t clock_timer_config = TIME_UPDATE_TIMER_CONFIG;
    esp_timer_handle_t display_update_timer = NULL;    
    esp_timer_create(&clock_timer_config, &display_update_timer);
    esp_timer_start_periodic(display_update_timer, TIME_UPDATE_INTERVAL);

    bool wifi_ok = init_wifi();
    if (wifi_ok)
    {
        ESP_LOGI(TAG, ">> WiFi initialized successfully");
        start_sntp_service();
    }
    else
    {
        ESP_LOGE(TAG, ">> WiFi initialization failed");
        return;
    }
}
