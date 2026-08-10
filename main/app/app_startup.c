#include "app_startup.h"

#include "freertos/FreeRTOS.h"
#include "esp_timer.h"

static app_startup_state_t startup_state;
static portMUX_TYPE startup_lock = portMUX_INITIALIZER_UNLOCKED;

void app_startup_init(void)
{
    portENTER_CRITICAL(&startup_lock);
    app_startup_state_init(&startup_state);
    portEXIT_CRITICAL(&startup_lock);
}

void app_startup_set(app_startup_phase_t phase, app_startup_status_t status)
{
    portENTER_CRITICAL(&startup_lock);
    app_startup_state_set(&startup_state, phase, status, esp_timer_get_time());
    portEXIT_CRITICAL(&startup_lock);
}

app_startup_state_t app_startup_get(void)
{
    app_startup_state_t state;
    portENTER_CRITICAL(&startup_lock);
    state = startup_state;
    portEXIT_CRITICAL(&startup_lock);
    return state;
}

bool app_startup_should_show(void)
{
    const app_startup_state_t state = app_startup_get();
    return app_startup_state_should_show(&state, esp_timer_get_time());
}
