#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool replace_weather;
    bool show_triangle;
    int64_t detected_at;
} app_alert_view_t;

typedef struct
{
    bool active;
    int64_t detected_at;
    int64_t detected_at_us;
    int64_t cleared_at_us;
} app_alert_state_t;

void app_alert_state_init(app_alert_state_t *state);
void app_alert_state_apply(app_alert_state_t *state, bool active,
                           int64_t detected_at, int64_t now_us);
app_alert_view_t app_alert_state_view(const app_alert_state_t *state,
                                      int64_t now_us);
