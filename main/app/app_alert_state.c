#include "app_alert_state.h"

#define ALERT_START_BLINK_US (30LL * 1000 * 1000)
#define ALERT_CLEAR_BLINK_US (10LL * 1000 * 1000)

static bool is_blink_on(int64_t elapsed_us, int64_t half_period_us)
{
    if (elapsed_us < 0)
    {
        return true;
    }

    return ((elapsed_us / half_period_us) % 2) == 0;
}

void app_alert_state_init(app_alert_state_t *state)
{
    *state = (app_alert_state_t){0};
}

void app_alert_state_apply(app_alert_state_t *state, bool active,
                           int64_t detected_at, int64_t now_us)
{
    if (active && !state->active)
    {
        state->active = true;
        state->detected_at = detected_at;
        state->detected_at_us = now_us;
        state->cleared_at_us = 0;
        return;
    }

    if (!active && state->active)
    {
        state->active = false;
        state->cleared_at_us = now_us;
    }
}

app_alert_view_t app_alert_state_view(const app_alert_state_t *state,
                                      int64_t now_us)
{
    app_alert_view_t view = {
        .detected_at = state->detected_at,
    };

    if (state->active)
    {
        const int64_t elapsed_us = now_us - state->detected_at_us;
        view.replace_weather = true;
        view.show_triangle = elapsed_us >= ALERT_START_BLINK_US
                                 || is_blink_on(elapsed_us, 500000);
        return view;
    }

    if (state->cleared_at_us == 0)
    {
        return view;
    }

    const int64_t elapsed_us = now_us - state->cleared_at_us;
    if (elapsed_us >= ALERT_CLEAR_BLINK_US)
    {
        return view;
    }

    view.replace_weather = true;
    view.show_triangle = is_blink_on(elapsed_us, 250000);
    return view;
}
