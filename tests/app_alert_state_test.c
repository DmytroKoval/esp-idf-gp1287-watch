#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "app_alert_state.h"

static void assert_view(app_alert_view_t view, bool replace_weather,
                        bool show_triangle, int64_t detected_at)
{
    assert(view.replace_weather == replace_weather);
    assert(view.show_triangle == show_triangle);
    assert(view.detected_at == detected_at);
}

static void test_active_and_clear_transitions(void)
{
    app_alert_state_t state;
    app_alert_state_init(&state);

    assert_view(app_alert_state_view(&state, 0), false, false, 0);

    app_alert_state_apply(&state, true, 100, 1000000);
    assert_view(app_alert_state_view(&state, 1000000), true, true, 100);
    assert_view(app_alert_state_view(&state, 1500000), true, false, 100);
    assert_view(app_alert_state_view(&state, 2000000), true, true, 100);
    assert_view(app_alert_state_view(&state, 31000000), true, true, 100);

    app_alert_state_apply(&state, false, 130, 31000000);
    assert_view(app_alert_state_view(&state, 31000000), true, true, 100);
    assert_view(app_alert_state_view(&state, 31250000), true, false, 100);
    assert_view(app_alert_state_view(&state, 41000000), false, false, 100);
}

static void test_reactivation_restarts_detected_time(void)
{
    app_alert_state_t state;
    app_alert_state_init(&state);

    app_alert_state_apply(&state, true, 100, 1000000);
    app_alert_state_apply(&state, false, 130, 31000000);
    app_alert_state_apply(&state, true, 131, 31250000);

    assert_view(app_alert_state_view(&state, 31250000), true, true, 131);
}

int main(void)
{
    test_active_and_clear_transitions();
    test_reactivation_restarts_detected_time();
    return 0;
}
