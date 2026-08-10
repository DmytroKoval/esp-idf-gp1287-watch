#pragma once

#include <stdbool.h>

#include "app_startup_state.h"

void app_startup_init(void);
void app_startup_set(app_startup_phase_t phase, app_startup_status_t status);
app_startup_state_t app_startup_get(void);
bool app_startup_should_show(void);
