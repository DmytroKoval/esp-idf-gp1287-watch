#include "app_startup_state.h"

#define STARTUP_HOLD_US (2LL * 1000 * 1000)

static bool all_phases_finished(const app_startup_state_t *state)
{
    for (int phase = 0; phase < APP_STARTUP_PHASE_COUNT; ++phase)
    {
        if (state->status[phase] == APP_STARTUP_PENDING)
        {
            return false;
        }
    }

    return true;
}

void app_startup_state_init(app_startup_state_t *state)
{
    *state = (app_startup_state_t){0};
}

void app_startup_state_set(app_startup_state_t *state, app_startup_phase_t phase,
                           app_startup_status_t status, int64_t now_us)
{
    state->status[phase] = status;
    if (state->completed_at_us == 0 && all_phases_finished(state))
    {
        state->completed_at_us = now_us;
    }
}

app_startup_status_t app_startup_state_get(const app_startup_state_t *state,
                                           app_startup_phase_t phase)
{
    return state->status[phase];
}

bool app_startup_state_should_show(const app_startup_state_t *state,
                                   int64_t now_us)
{
    return state->completed_at_us == 0
           || now_us - state->completed_at_us < STARTUP_HOLD_US;
}
