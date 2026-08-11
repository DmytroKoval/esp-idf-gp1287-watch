#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "soc/soc_caps.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <esp_err.h>
#include <esp_check.h>
#include <esp_log.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <esp_lcd_types.h>
#include "sdkconfig.h"

#include "u8g2.h"
#include "esp_lcd_panel_gp1287.h"
#include "../app_config.h"
#include "app_alert.h"
#include "app_startup.h"
#include "app_weather.h"

#define V_MAX 3400                                   // Maximum voltage for ADC in mV
#define DEFAULT_VREF 1100                            // Default reference voltage for ADC in mV
#define BRIGHTNESS_NO_OF_SAMPLES 10                  // Number of samples for averaging
#define BRIGHTNESS_READ_INTERVAL pdMS_TO_TICKS(250)  // Read interval in milliseconds
#define ADC_TASK_PRIORITY 1                          // ADC task priority
#define ADC_TASK_STACK_SIZE 1024 * 4                 // Stack size for the ADC task

static char* TAG = "GP1287-DISPLAY";

static volatile bool initialized = false;

static u8g2_t u8g2;

static uint8_t *gbuf = NULL;

static esp_lcd_panel_handle_t display_handle = NULL;

static bool force_duty_mode = false;

static volatile uint16_t display_brightness;

TaskHandle_t adc_task_handle = NULL;

/// @brief brightness sensor ADC handle
/// @note This handle is used to read the brightness sensor value from the ADC channel.
static adc_oneshot_unit_handle_t brightness_adc_handle = NULL;

/// @brief brightness sensor adc calibration handle
/// @note This is a handle for the ADC calibration scheme, which is used to convert raw ADC values to voltage values.
static adc_cali_handle_t brightness_adc_cali_handle = NULL;

/// @brief 40ms (25Hz) display update timer
static esp_timer_handle_t display_refresh_timer = NULL;

static esp_err_t setup_GP1287(esp_lcd_panel_handle_t *panel_handle)
{
    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num = DISPLAY_SPI_SCLK,
        .mosi_io_num = DISPLAY_SPI_MOSI,
        .flags = SPICOMMON_BUSFLAG_MASTER,
        .max_transfer_sz = (256 * 128) >> 3,
    };

    esp_err_t ret = spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_RETURN_ON_ERROR(ret, TAG, "SPI bus init failed");

    ESP_LOGD(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = DISPLAY_SPI_CS,
        .dc_gpio_num = DISPLAY_SPI_DC,
        .pclk_hz = 4 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .flags = {.lsb_first = 1},
        .spi_mode = 3,
        .trans_queue_depth = 10,
    };
    esp_lcd_panel_io_handle_t lcd_io = NULL;
    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST, &io_config, &lcd_io);
    ESP_RETURN_ON_ERROR(ret, TAG, "SPI panel IO setup failed");

    gp1287_dev_config_t gp1287_config = {
        .filament_en_io_num = DISPLAY_FILEMENT_EN,
    };

    ESP_LOGD(TAG, "Install display driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = DISPLAY_RESET,
        .flags = {.reset_active_high = 0},
        .vendor_config = &gp1287_config,
        .bits_per_pixel = 1,
    };
    ret = esp_lcd_new_panel_gp1287(lcd_io, &panel_config, &(*panel_handle));
    ESP_RETURN_ON_ERROR(ret, TAG, "GP1287 driver setup failed");

    ret = esp_lcd_panel_reset(*panel_handle);
    ESP_RETURN_ON_ERROR(ret, TAG, "GP1287 init failed");

    ret = esp_lcd_panel_init(*panel_handle);
    ESP_RETURN_ON_ERROR(ret, TAG, "GP1287 reset failed");

    ret = esp_lcd_panel_set_brightness(*panel_handle, DEFAULT_BRIGHTNESS);
    ESP_RETURN_ON_ERROR(ret, TAG, "GP1287 brightness failed");

    return ESP_OK;
}

/// @brief u8g2 callback stub for SPI transfer (not used)
/// @param u8x8
/// @param msg
/// @param arg_int
/// @param arg_ptr
/// @return
static uint8_t u8g2_esp32_spi_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    return 0;
}

/// @brief u8g2 callback stub for GPIO and delay (not used)
/// @param u8x8
/// @param msg
/// @param arg_int
/// @param arg_ptr
/// @return
static uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    return 0;
}

static void draw_seconds_scale(int seconds)
{
    const int first_x = 8;
    const int y = 38;

    u8g2_DrawLine(&u8g2, 6, y - 2, 6, y + 2);
    u8g2_DrawLine(&u8g2, 7, y - 3, 9, y - 3);
    u8g2_DrawLine(&u8g2, 7, y + 3, 9, y + 3);
    u8g2_DrawLine(&u8g2, 126, y - 2, 126, y + 2);
    u8g2_DrawLine(&u8g2, 123, y - 3, 125, y - 3);
    u8g2_DrawLine(&u8g2, 123, y + 3, 125, y + 3);

    for (int second = 1; second <= 60; ++second)
    {
        const int x = first_x + (second - 1) * 2;
        if (second > seconds)
        {
            u8g2_DrawPixel(&u8g2, x, y);
            continue;
        }

        int top = y - 1;
        int bottom = y + 1;
        if (second % 5 == 0)
        {
            bottom = y + 2;
        }
        if (second % 10 == 0)
        {
            top = y - 2;
            bottom = y + 2;
        }
        u8g2_DrawLine(&u8g2, x, top, x, bottom);
    }
}

static uint8_t weather_glyph(const app_weather_snapshot_t *weather)
{
    if (weather->condition_code == 800)
    {
        return weather->is_night ? 'B' : 'E';
    }
    if (weather->condition_code == 801 || weather->condition_code == 802)
    {
        return 'A';
    }
    if ((weather->condition_code >= 200 && weather->condition_code < 600)
        || (weather->condition_code >= 600 && weather->condition_code < 700))
    {
        return 'C';
    }
    return '@';
}

static void draw_weather(void)
{
    const app_weather_snapshot_t weather = app_weather_get_snapshot();

    u8g2_SetFont(&u8g2, u8g2_font_open_iconic_weather_4x_t);
    if (weather.valid)
    {
        u8g2_DrawGlyph(&u8g2, 149, 32, weather_glyph(&weather));
    }
    else
    {
        u8g2_DrawGlyph(&u8g2, 149, 32, '@');
    }

    u8g2_SetFont(&u8g2, u8g2_font_profont29_mn);
    char temperature[8];
    if (weather.valid)
    {
        snprintf(temperature, sizeof(temperature), "%d", weather.temperature_c);
    }
    else
    {
        strcpy(temperature, "--");
    }

    u8g2_DrawStr(&u8g2, 194, 28, temperature);
    const u8g2_uint_t temperature_width = u8g2_GetStrWidth(&u8g2, temperature);
    u8g2_DrawGlyph(&u8g2, 194 + temperature_width + 1, 20, 0xB0);
}

static void draw_alert(const app_alert_view_t *alert, time_t current_time)
{
    struct tm detected_time;
    char start_time[8];
    const time_t detected_at = (time_t)alert->detected_at;
    localtime_r(&detected_at, &detected_time);
    strftime(start_time, sizeof(start_time), "%H:%M", &detected_time);

    time_t duration = current_time - (time_t)alert->detected_at;
    if (duration < 0)
    {
        duration = 0;
    }

    char duration_text[16];
    snprintf(duration_text, sizeof(duration_text), "%ldh %02ldm",
             (long)(duration / 3600), (long)((duration % 3600) / 60));

    u8g2_SetFont(&u8g2, u8g2_font_profont12_tr);
    const u8g2_uint_t start_width = u8g2_GetStrWidth(&u8g2, start_time);
    const u8g2_uint_t duration_width = u8g2_GetStrWidth(&u8g2, duration_text);

    if (alert->show_triangle)
    {
        u8g2_DrawTriangle(&u8g2, 172, 0, 189, 34, 155, 34);
        u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawBox(&u8g2, 170, 28, 5, 4);
        u8g2_DrawTriangle(&u8g2, 172, 24, 176, 11, 168, 11);
        u8g2_SetDrawColor(&u8g2, 1);
    }

    const int text_center = 202 + (int)start_width / 2;
    u8g2_DrawStr(&u8g2, 202, 16, start_time);
    u8g2_DrawStr(&u8g2, text_center - (int)duration_width / 2, 28, duration_text);
}

static void draw_startup_screen(void)
{
    static const char *phase_text[APP_STARTUP_PHASE_COUNT] = {
        "WiFi...",
        "Time...",
        "Weather...",
        "Alerts...",
    };
    static const int baseline[APP_STARTUP_PHASE_COUNT] = { 11, 22, 33, 44 };

    const app_startup_state_t startup = app_startup_get();
    u8g2_SetFont(&u8g2, u8g2_font_profont12_tr);

    for (int phase = 0; phase < APP_STARTUP_PHASE_COUNT; ++phase)
    {
        const app_startup_status_t status = app_startup_state_get(&startup, phase);
        const char *status_text = status == APP_STARTUP_OK ? "ok"
                                  : status == APP_STARTUP_ERROR ? "err" : "...";

        u8g2_DrawStr(&u8g2, 72, baseline[phase], phase_text[phase]);
        const u8g2_uint_t status_width = u8g2_GetStrWidth(&u8g2, status_text);
        u8g2_DrawStr(&u8g2, 184 - (int)status_width, baseline[phase], status_text);
    }
}

static bool is_duty_mode(const struct tm *timeinfo,
                         const app_alert_view_t *alert)
{
    if (force_duty_mode)
    {
        return true;
    }

    if (alert->replace_weather)
    {
        return false;
    }

    if (display_brightness <= CONFIG_NIGHT_MODE_BRIGHTNESS_THRESHOLD)
    {
        return true;
    }

    return timeinfo->tm_hour >= 23 || timeinfo->tm_hour < 6;
}

static void draw_duty_screen(const struct tm *timeinfo, const char *time_text)
{
    static const u8g2_uint_t calendar_baseline = 45;
    static const int day_x[] = { 64, 79, 93, 107, 121, 135, 151 };

    u8g2_SetFont(&u8g2, u8g2_font_profont29_mn);
    const u8g2_uint_t time_width = u8g2_GetStrWidth(&u8g2, time_text);
    u8g2_DrawRFrame(&u8g2, 68, 1, 121, 29, 1);
    u8g2_DrawStr(&u8g2, 128 - (int)time_width / 2, 26, time_text);

    struct tm sunday = *timeinfo;
    sunday.tm_mday -= sunday.tm_wday;
    sunday.tm_hour = 12;
    mktime(&sunday);

    u8g2_SetFont(&u8g2, u8g2_font_profont12_tr);
    for (int day_index = 0; day_index < 7; ++day_index)
    {
        struct tm day = sunday;
        day.tm_mday += day_index;
        mktime(&day);

        char day_text[3];
        snprintf(day_text, sizeof(day_text), "%2d", day.tm_mday);
        u8g2_DrawStr(&u8g2, day_x[day_index], calendar_baseline, day_text);
    }

    u8g2_SetDrawColor(&u8g2, 2);
    u8g2_DrawRBox(&u8g2, day_x[timeinfo->tm_wday] - 2, 35, 15, 12, 0);
    
    u8g2_SetDrawColor(&u8g2, 1);

    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawRFrame(&u8g2, 149, 35, 15, 12, 1);
    u8g2_DrawRFrame(&u8g2, 133, 35, 15, 12, 1);

    char month[4];
    strftime(month, sizeof(month), "%b", timeinfo);
    u8g2_DrawStr(&u8g2, 169, calendar_baseline, month);
}

static void draw_main_screen(const struct tm *timeinfo, const char *time_text,
                             const char *date_text, time_t current_time,
                             const app_alert_view_t *alert)
{
    u8g2_DrawRFrame(&u8g2, 2, 2, 129, 46, 2);
    u8g2_SetFont(&u8g2, u8g2_font_profont29_mn);
    const u8g2_uint_t time_width = u8g2_GetStrWidth(&u8g2, time_text);
    u8g2_DrawStr(&u8g2, 66 - (int)time_width / 2, 29, time_text);
    draw_seconds_scale(timeinfo->tm_sec);

    if (alert->replace_weather)
    {
        draw_alert(alert, current_time);
    }
    else
    {
        draw_weather();
    }

    u8g2_SetFont(&u8g2, u8g2_font_profont12_tr);
    u8g2_DrawStr(&u8g2, 144, 46, date_text);
}

static void render_display(time_t current_time)
{
    static bool show_colon = false;
    static bool second_frame = false;

    if (!initialized)
    {
        return;
    }

    struct tm timeinfo;
    char time_text[8];
    char date_text[32];

    localtime_r(&current_time, &timeinfo);
    strftime(time_text, sizeof(time_text), show_colon ? "%H:%M" : "%H %M", &timeinfo);
    strftime(date_text, sizeof(date_text), "%a, %b %d, %Y", &timeinfo);

    u8g2_ClearBuffer(&u8g2);
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_SetFontMode(&u8g2, 1);

    const app_alert_view_t alert = app_alert_get_view();
    if (app_startup_should_show())
    {
        draw_startup_screen();
    }
    else if (is_duty_mode(&timeinfo, &alert))
    {
        draw_duty_screen(&timeinfo, time_text);
    }
    else
    {
        draw_main_screen(&timeinfo, time_text, date_text, current_time, &alert);
    }

    if (second_frame)
    {
        show_colon = !show_colon;
    }
    second_frame = !second_frame;
}

static void refresh_display_cb(void *arg)
{
    if (!initialized)
    {
        return;
    }
    esp_lcd_panel_set_brightness(display_handle, display_brightness);
    esp_lcd_panel_draw_bitmap(display_handle, 0, 0, 256, 128, gbuf);
}

static void adc_task(void *pvParameters)
{
    static int current_index = 0;                               // Поточний індекс в буфері
    static uint32_t adc_values[BRIGHTNESS_NO_OF_SAMPLES] = {0}; // Буфер для зберігання останніх значень АЦП
    int adc_raw;
    int adc_mv;
    uint32_t disp_brightness = 0;
    while (1)
    {
        ESP_ERROR_CHECK(adc_oneshot_read(brightness_adc_handle, DISPLAY_BRIGHTNESS_ADC_CHANNEL, &adc_raw));
        adc_cali_raw_to_voltage(brightness_adc_cali_handle, adc_raw, &adc_mv);

        // 3400 mv when brightnes is 0
        int next_val = V_MAX - adc_mv; // higher reading means lower brightness

        if (next_val < MIN_BRIGHTNESS)
        {
            next_val = MIN_BRIGHTNESS;
        }

        adc_values[current_index] = next_val;
        current_index = (current_index + 1) % BRIGHTNESS_NO_OF_SAMPLES;

        // Обчислення середнього значення
        uint32_t sum = 0;
        for (int i = 0; i < BRIGHTNESS_NO_OF_SAMPLES; i++)
        {
            sum += adc_values[i];
        }
        disp_brightness = sum / (BRIGHTNESS_NO_OF_SAMPLES * 6);

        if (disp_brightness > MAX_BRIGHTNESS)
        {
            disp_brightness = MAX_BRIGHTNESS;
        }

        display_brightness = disp_brightness;

        vTaskDelay(BRIGHTNESS_READ_INTERVAL); // Затримка перед наступним зчитуванням
    }
}

static esp_err_t init_brightness_sensor()
{
    esp_err_t ret = ESP_OK;
    static bool calibrated = false;

    if (adc_task_handle != NULL) {
        // already initialized
        return ESP_OK;
    }

    adc_oneshot_unit_init_cfg_t brightness_adc_config = {
        .unit_id = DISPLAY_BRIGHTNESS_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    ret = adc_oneshot_new_unit(&brightness_adc_config, &brightness_adc_handle);
    ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "ADC unit init failed");

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    ret = adc_oneshot_config_channel(brightness_adc_handle, ADC_CHANNEL_4, &config);
    ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "ADC channel init failed");

    if (!calibrated)
    {
        ESP_LOGI(TAG, "Calibration scheme version is Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .chan = DISPLAY_BRIGHTNESS_ADC_CHANNEL,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &brightness_adc_cali_handle);
        ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "ADC calibration failed");
        calibrated = true;
    }

    BaseType_t result = xTaskCreate(adc_task, "ADC_Task", ADC_TASK_STACK_SIZE, NULL, ADC_TASK_PRIORITY, &adc_task_handle);
    if (result != pdPASS) {
        ret = ESP_FAIL;
        goto cleanup;
    }
    return ESP_OK;

cleanup:
    if (adc_task_handle != NULL) {
        vTaskDelete(adc_task_handle);
        adc_task_handle = NULL;
    }
    if (brightness_adc_cali_handle != NULL) {
        adc_cali_delete_scheme_curve_fitting(brightness_adc_cali_handle);
        brightness_adc_cali_handle = NULL;
        calibrated = false;
    }
    if (brightness_adc_handle != NULL) {
        adc_oneshot_del_unit(brightness_adc_handle);
        brightness_adc_handle = NULL;
    }
    return ret;
}

static void setup_timers(esp_timer_cb_t cb)
{
    if (display_refresh_timer != NULL) {
        // already created
        return;
    }
    esp_timer_create_args_t display_refresh_timer_config = {
        .callback = cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "display refresh",
        .skip_unhandled_events = false
    };

    esp_timer_create(&display_refresh_timer_config, &display_refresh_timer);
    // 25Hz display update
    esp_timer_start_periodic(display_refresh_timer, 40000);
}

/* public functions */

esp_err_t setup_display()
{
    esp_err_t ret = ESP_OK;
    const size_t buf_size = 256 * 16;

    if (initialized) {
        return ESP_OK;
    }

    // setup display
    ret = setup_GP1287(&display_handle);
    ESP_RETURN_ON_ERROR(ret, TAG, "GP1287 setup failed");
    ret = init_brightness_sensor();
    ESP_RETURN_ON_ERROR(ret, TAG, "Brightness sensor setup failed");

    //setup display buffer
    if (gbuf == NULL) {
        gbuf = (uint8_t *)heap_caps_calloc(1, buf_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    }
    ESP_RETURN_ON_FALSE(gbuf != NULL, ESP_FAIL, TAG, "no mem for gbuf %u", buf_size);

    // init graphics library
    u8g2_Setup_gp1287ai_256x50_f(&u8g2, U8G2_R0, u8g2_esp32_spi_byte_cb, u8g2_esp32_gpio_and_delay_cb);
    u8g2_InitDisplay(&u8g2);

    u8g2.tile_buf_ptr = gbuf;

    // setup display timers
    setup_timers(refresh_display_cb);
    initialized = true;
    return ret;
}

esp_err_t test_display(uint16_t brightness)
{
    // esp_lcd_panel_set_brightness(display_handle, 20);
    force_duty_mode = true;
    display_brightness = brightness;
    // memset(gbuf, 0xFF, 256 * 16);
    // esp_lcd_panel_draw_bitmap(display_handle, 0, 0, 256, 128, gbuf);
    return ESP_OK;
}

void set_time(time_t t)
{
    render_display(t);
}

esp_err_t display_on()
{
    return esp_lcd_panel_disp_on_off(display_handle, true);
}

esp_err_t display_off()
{
    return esp_lcd_panel_disp_on_off(display_handle, false);
}

esp_err_t display_set_brightness(uint16_t brightness)
{
    return esp_lcd_panel_set_brightness(display_handle, brightness);
}
