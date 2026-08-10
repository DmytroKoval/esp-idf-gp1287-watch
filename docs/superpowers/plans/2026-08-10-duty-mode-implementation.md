# Duty Mode and Startup Status Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the startup status screen and duty-mode calendar screen without
changing the alert priority or normal screen layout.

**Architecture:** A small `app_startup` module is the only cross-module state:
Wi-Fi, time, weather and alerts report their first result, while the display
reads an immutable snapshot. `app_display.c` owns the three render functions
and selects startup, main, or duty screen locally.

**Tech Stack:** ESP-IDF v6, FreeRTOS, esp_timer, U8g2, Kconfig.

---

### Task 1: Add testable startup-state logic

**Files:**

- Create: `main/app/app_startup_state.h`
- Create: `main/app/app_startup_state.c`
- Create: `tests/app_startup_state_test.c`

- [x] Write a focused failing test for the pending phases, a weather `err`,
  and the two-second hold after the fourth completion.
- [x] Build the test with the existing host-test command and verify it fails
  because the startup-state API does not exist.
- [x] Implement a pure state type with `pending`, `ok`, `err`, four phases,
  `app_startup_state_set()`, and `app_startup_state_should_show()`.
- [x] Re-run the test and verify it passes.

### Task 2: Wire startup ownership to the services

**Files:**

- Create: `main/app/app_startup.h`
- Create: `main/app/app_startup.c`
- Modify: `main/CMakeLists.txt`
- Modify: `main/wifi_watch.c`
- Modify: `main/app/app_wifi.c`
- Modify: `main/app/app_weather.c`
- Modify: `main/app/app_alert.c`
- Modify: `main/Kconfig.projbuild`

- [x] Add a lock-protected wrapper around the pure state and initialise all
  phases as pending before `init_wifi()`.
- [x] Set WiFi to `ok` after the IP event is accepted, Time to `ok` after the
  initial SNTP sync, and show each as `err` for two seconds before reboot on
  terminal failure.
- [x] Change the default `WIFI_MAXIMUM_RETRY` to `10`.
- [x] Make the first weather and alerts request report `ok` or `err` once;
  later periodic polling must not overwrite the startup result.
- [x] Add the new sources to the component and expose only the required public
  functions in `app_startup.h`.

### Task 3: Add the three display render paths

**Files:**

- Modify: `main/app/app_display.c`
- Modify: `main/Kconfig.projbuild`

- [x] Add `NIGHT_MODE_BRIGHTNESS_THRESHOLD`, range `0..600`, default `20`.
- [x] Add `draw_startup_screen()` with four P12 rows and right-aligned
  `...`, `ok`, or `err` status text; do not draw the clock there.
- [x] Extract the existing layout into `draw_main_screen()` unchanged.
- [x] Add `is_duty_mode()` and `draw_duty_screen()` with the accepted
  Sunday–Saturday calendar, XOR rounded selection box, and blinking P29 time.
- [x] Make `render_display()` choose startup first, then alert/main, then duty,
  then normal main.

### Task 4: Verify and integrate

**Files:**

- Modify: all files from Tasks 1–3 only as required by review.

- [x] Run the host state test.
- [x] Run `idf.py build` in the ESP-IDF PowerShell environment.
- [x] Inspect `git diff --check` and the final diff; do not include
  `.devcontainer/`.
- [x] Commit the feature only after the build passes.
- [ ] Flash and visually test only with a new explicit user approval.
