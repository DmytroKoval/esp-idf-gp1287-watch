#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    APP_STARTUP_WIFI,
    APP_STARTUP_TIME,
    APP_STARTUP_WEATHER,
    APP_STARTUP_ALERTS,
    APP_STARTUP_PHASE_COUNT,
} app_startup_phase_t;

typedef enum
{
    APP_STARTUP_PENDING,
    APP_STARTUP_OK,
    APP_STARTUP_ERROR,
} app_startup_status_t;

typedef struct
{
    app_startup_status_t status[APP_STARTUP_PHASE_COUNT];
    int64_t completed_at_us;
} app_startup_state_t;

void app_startup_state_init(app_startup_state_t *state);
void app_startup_state_set(app_startup_state_t *state, app_startup_phase_t phase,
                           app_startup_status_t status, int64_t now_us);
app_startup_status_t app_startup_state_get(const app_startup_state_t *state,
                                           app_startup_phase_t phase);
bool app_startup_state_should_show(const app_startup_state_t *state,
                                   int64_t now_us);
