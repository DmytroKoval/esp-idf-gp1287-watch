#pragma once

#include <freertos/queue.h>

#ifndef __APP_H__
#define __APP_H__
#ifdef __cplusplus
extern "C" {
#endif

#define TIME_UPDATE_TIMER_CONFIG { \
    .callback = clock_timer_callback, \
    .arg = (void *)NULL, \
    .dispatch_method = ESP_TIMER_TASK, \
    .name = "periodic", \
    .skip_unhandled_events = false \
}

bool init_wifi();
void start_sntp_service();

esp_err_t display_on();
esp_err_t display_off();
esp_err_t display_set_brightness(uint16_t brightness);
esp_err_t setup_display();
void set_time(time_t t);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // __APP_H__