#pragma once

#include <stdbool.h>

typedef struct
{
    bool valid;
    int temperature_c;
    int condition_code;
    bool is_night;
} app_weather_snapshot_t;

void start_weather_service(void);
app_weather_snapshot_t app_weather_get_snapshot(void);
