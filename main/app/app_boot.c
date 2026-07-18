#include <stdint.h>

#include <esp_attr.h>
#include <esp_log.h>
#include <esp_system.h>

#include "app.h"

#define TAG "BOOT"
#define BOOT_COUNTER_MAGIC 0x47503132UL

typedef struct
{
    uint32_t magic;
    uint32_t startup_count;
    uint32_t restart_count;
} app_boot_state_t;

RTC_NOINIT_ATTR static app_boot_state_t s_boot_state;

static const char *reset_reason_to_string(esp_reset_reason_t reason)
{
    switch (reason)
    {
    case ESP_RST_UNKNOWN:
        return "unknown";
    case ESP_RST_POWERON:
        return "poweron";
    case ESP_RST_EXT:
        return "external";
    case ESP_RST_SW:
        return "software";
    case ESP_RST_PANIC:
        return "panic";
    case ESP_RST_INT_WDT:
        return "interrupt_wdt";
    case ESP_RST_TASK_WDT:
        return "task_wdt";
    case ESP_RST_WDT:
        return "other_wdt";
    case ESP_RST_DEEPSLEEP:
        return "deepsleep";
    case ESP_RST_BROWNOUT:
        return "brownout";
    case ESP_RST_SDIO:
        return "sdio";
    case ESP_RST_USB:
        return "usb";
    case ESP_RST_JTAG:
        return "jtag";
    case ESP_RST_EFUSE:
        return "efuse";
    case ESP_RST_PWR_GLITCH:
        return "power_glitch";
    case ESP_RST_CPU_LOCKUP:
        return "cpu_lockup";
    default:
        return "unsupported";
    }
}

static bool is_cold_boot(esp_reset_reason_t reason)
{
    return reason == ESP_RST_POWERON || reason == ESP_RST_BROWNOUT;
}

void app_boot_log_startup(void)
{
    esp_reset_reason_t reset_reason = esp_reset_reason();

    if (s_boot_state.magic != BOOT_COUNTER_MAGIC || is_cold_boot(reset_reason))
    {
        s_boot_state.magic = BOOT_COUNTER_MAGIC;
        s_boot_state.startup_count = 1;
        s_boot_state.restart_count = 0;
    }
    else
    {
        s_boot_state.startup_count++;

        if (reset_reason != ESP_RST_DEEPSLEEP)
        {
            s_boot_state.restart_count++;
        }
    }

    ESP_LOGW(TAG,
             "Startup #%lu since power-on, warm restarts: %lu, reset reason: %s",
             (unsigned long)s_boot_state.startup_count,
             (unsigned long)s_boot_state.restart_count,
             reset_reason_to_string(reset_reason));
}

uint32_t app_boot_get_startup_count(void)
{
    if (s_boot_state.magic != BOOT_COUNTER_MAGIC)
    {
        return 0;
    }

    return s_boot_state.startup_count;
}