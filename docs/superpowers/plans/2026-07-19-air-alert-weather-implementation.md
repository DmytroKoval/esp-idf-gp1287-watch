# Air Alert and Weather Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Show current weather normally and replace it with the designed air-alert indicator for Dnipropetrovsk oblast.

**Architecture:** A small pure state module owns alert transitions and blink timing. An ESP-IDF alert task polls the compact alerts.in.ua endpoint once a minute, while a separate weather task reuses the existing OpenWeather pattern for Kamyanske and refreshes hourly. The display reads immutable snapshots and never waits for HTTP.

**Tech Stack:** ESP-IDF 6, FreeRTOS, `esp_http_client`, certificate bundle, cJSON, U8g2.

---

### Task 1: Deterministic alert state

**Files:**
- Create: `main/app/app_alert_state.h`
- Create: `main/app/app_alert_state.c`
- Create: `tests/app_alert_state_test.c`

- [ ] **Step 1: Write a host test for transitions**

```c
assert_view(app_alert_state_view(&state, 0), false, false);
app_alert_state_apply(&state, true, 100, 1000000);
assert_view(app_alert_state_view(&state, 110, 1500000), true, false);
assert_view(app_alert_state_view(&state, 110, 2000000), true, true);
app_alert_state_apply(&state, false, 140, 31000000);
assert_view(app_alert_state_view(&state, 141, 31250000), true, true);
assert_view(app_alert_state_view(&state, 151, 41000000), false, false);
```

- [ ] **Step 2: Compile and run the failing test**

Run: `gcc -std=c11 -Wall -Wextra tests/app_alert_state_test.c main/app/app_alert_state.c -o build/app_alert_state_test && build/app_alert_state_test`

Expected: compilation fails because the state module does not exist.

- [ ] **Step 3: Add the state machine**

Define `app_alert_state_t` with `active`, `detected_at`, `detected_at_us`, and `cleared_at_us`. `app_alert_state_apply()` stores the first detection time on `false → true` and the clear time on `true → false`. `app_alert_state_view()` returns `replace_weather`, `show_triangle`, and `detected_at`.

Use a 500 ms phase for 1 Hz and 250 ms phase for 2 Hz. Keep the triangle visible after 30 seconds of an active alert and hide the full alert view 10 seconds after clear.

- [ ] **Step 4: Compile and run the passing test**

Run the command from Step 2.

Expected: process exits with code 0.

- [ ] **Step 5: Commit**

```text
feat: add deterministic air alert state
```

### Task 2: alerts.in.ua polling service

**Files:**
- Create: `main/app/app_alert.h`
- Create: `main/app/app_alert.c`
- Modify: `main/Kconfig.projbuild`
- Modify: `main/CMakeLists.txt`
- Modify: `main/app.h`
- Modify: `main/wifi_watch.c`

- [ ] **Step 1: Add the token configuration**

Add `ALERTS_API_TOKEN` as an empty string under an `Air Alert Configuration` menu. The token is passed only as `Authorization: Bearer <token>` and is never logged.

- [ ] **Step 2: Implement one bounded HTTPS request**

Use `https://api.alerts.in.ua/v1/iot/active_air_raid_alerts/9.json`, `esp_crt_bundle_attach`, a 5 second timeout, and a small response buffer. Accept only `A`, `P`, or `N`; return an error for any other HTTP status or response. Do not parse JSON or allocate response memory.

- [ ] **Step 3: Run the alert task**

Start a FreeRTOS task after successful Wi-Fi and SNTP startup. It fetches once immediately, then waits 60 seconds between requests. It calls the state module with `time(NULL)` and `esp_timer_get_time()`. Failed requests retain the prior state.

- [ ] **Step 4: Expose a display snapshot**

Return the state view under a short critical section so the display task never races the polling task.

- [ ] **Step 5: Commit**

```text
feat: poll Dnipropetrovsk air alerts
```

### Task 3: Narrow weather service for Kamyanske

**Files:**
- Create: `main/app/app_weather.h`
- Create: `main/app/app_weather.c`
- Modify: `main/Kconfig.projbuild`
- Modify: `main/CMakeLists.txt`
- Modify: `main/app.h`
- Modify: `main/wifi_watch.c`

- [ ] **Step 1: Add local OpenWeather key configuration**

Add `WEATHER_API_KEY` as an empty string. Do not provide or commit a key.

- [ ] **Step 2: Request only the current Kamyanske weather**

Reuse the existing coordinates `48.5168,34.6069` and HTTPS certificate-bundle configuration. Read a bounded response, parse `current.temp` and `current.weather[0].id` with cJSON, and keep the last valid temperature and condition code on failure.

- [ ] **Step 3: Refresh without blocking the display**

Fetch once after Wi-Fi/SNTP is ready and then once per hour. Expose the last weather snapshot through `app_weather_get_current()`.

- [ ] **Step 4: Commit**

```text
feat: add Kamyanske weather service
```

### Task 4: Render the approved screen

**Files:**
- Modify: `main/app/app_display.c`

- [ ] **Step 1: Draw the normal main screen**

Draw the left 129×46 clock/seconds frame, full date at `(144, 46)`, then render the 32 px weather glyph and temperature with the chosen ProFont29/ProFont12 positions.

- [ ] **Step 2: Draw the alert override**

When `replace_weather` is true, replace only the weather area with the exact triangle: `(172,0)`, `(189,34)`, `(155,34)`, ellipse `(172,30,3,3)`, and inner triangle `(172,24)`, `(176,11)`, `(168,11)`. Draw `HH:MM` at `(202,16)` and centered `Hh MMm` at baseline `28`. Hide only the triangle during its off phase.

- [ ] **Step 3: Confirm 1-bit geometry locally**

Build the firmware after activating `C:\Espressif\v6.0\esp-idf\export.ps1`. Flashing is not part of this task.

- [ ] **Step 4: Commit**

```text
feat: render alert and weather states
```

### Task 5: Full verification

**Files:**
- Modify: `docs/superpowers/specs/2026-07-19-air-alert-indicator-design.md` only if verification finds a specification defect.

- [ ] **Step 1: Run the host state test**

Expected: all transition assertions pass.

- [ ] **Step 2: Build from an activated ESP-IDF shell**

Run: `. C:\Espressif\v6.0\esp-idf\export.ps1; idf.py build`

Expected: build completes successfully.

- [ ] **Step 3: Inspect final diff and status**

Run: `git diff --check HEAD~4..HEAD` and `git status --short`.

Expected: no whitespace errors in new changes and no unintended tracked files.
