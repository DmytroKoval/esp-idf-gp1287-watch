#include <assert.h>
#include <stdint.h>

#include "app_startup_state.h"

static void test_startup_holds_until_all_phases_finish(void)
{
    app_startup_state_t state;
    app_startup_state_init(&state);

    for (int phase = 0; phase < APP_STARTUP_PHASE_COUNT; ++phase)
    {
        assert(app_startup_state_get(&state, phase) == APP_STARTUP_PENDING);
    }
    assert(app_startup_state_should_show(&state, 0));

    app_startup_state_set(&state, APP_STARTUP_WIFI, APP_STARTUP_OK, 100);
    app_startup_state_set(&state, APP_STARTUP_TIME, APP_STARTUP_OK, 200);
    app_startup_state_set(&state, APP_STARTUP_WEATHER, APP_STARTUP_ERROR, 300);
    assert(app_startup_state_should_show(&state, 1000000));

    app_startup_state_set(&state, APP_STARTUP_ALERTS, APP_STARTUP_OK, 400);
    assert(app_startup_state_should_show(&state, 2000399));
    assert(!app_startup_state_should_show(&state, 2000400));
}

int main(void)
{
    test_startup_holds_until_all_phases_finish();
    return 0;
}
