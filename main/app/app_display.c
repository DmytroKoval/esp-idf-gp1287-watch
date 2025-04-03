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

#include "u8g2.h"
#include "esp_lcd_panel_gp1287.h"
#include "../app_config.h"
#include "../app.h"

#define V_MAX 3400                                   // Maximum voltage for ADC in mV
#define DEFAULT_VREF 1100                            // Default reference voltage for ADC in mV
#define BRIGHTNESS_NO_OF_SAMPLES 10                  // Number of samples for averaging
#define BRIGHTNESS_READ_INTERVAL pdMS_TO_TICKS(250)  // Read interval in milliseconds
#define ADC_TASK_PRIORITY 1                          // ADC task priority
#define ADC_TASK_STACK_SIZE 1024 * 4                 // Stack size for the ADC task

static char* TAG = "GP1287-DISPLAY";

static bool initialized = false; // Flag to check if the display is initialized 

static u8g2_t u8g2;

static uint8_t *gbuf = NULL;

static esp_lcd_panel_handle_t display_handle = NULL;

static uint16_t display_brightness;

TaskHandle_t adc_task_handle = NULL;

/// @brief brightness sensor ADC handle
/// @note This handle is used to read the brightness sensor value from the ADC channel.
static adc_oneshot_unit_handle_t brightness_adc_handle = NULL;

/// @brief brightness sensor adc calibration handle
/// @note This is a handle for the ADC calibration scheme, which is used to convert raw ADC values to voltage values.
static adc_cali_handle_t brightness_adc_cali_handle = NULL;
static time_t now;

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

static void render_display()
{
    if (!initialized)
    {
        return;
    }
    struct tm timeinfo;
    char strftime_buf[64];
    char strfdate_buf[64];
    static bool sec = false;

    localtime_r(&now, &timeinfo);
    
    // format time
    if (sec)
    {
        strftime(strftime_buf, sizeof(strftime_buf), "%H:%M", &timeinfo);
    }
    else
    {
        strftime(strftime_buf, sizeof(strftime_buf), "%H %M", &timeinfo);
    }

    u8g2_ClearBuffer(&u8g2);

    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_SetFontMode(&u8g2, 1);
#if 1
    const u8g2_uint_t corner_size = 5;
    const u8g2_uint_t max_x = 255;
    const u8g2_uint_t max_y = 49;
    // corner-tl-h
    u8g2_DrawLine(&u8g2, 0, 0, corner_size, 0);

    // corner-tl-v
    u8g2_DrawLine(&u8g2, 0, 0, 0, corner_size);

    // corner-bl-h
    u8g2_DrawLine(&u8g2, 0, max_y, corner_size, max_y);

    // corner-bl-v
    u8g2_DrawLine(&u8g2, 0, max_y - corner_size, 0, max_y);

    // corner-br-h
    u8g2_DrawLine(&u8g2, max_x - corner_size, max_y, max_x, max_y);

    // corner-br-v
    u8g2_DrawLine(&u8g2, max_x, max_y - corner_size, max_x, max_x);

    // corner-tr-v
    u8g2_DrawLine(&u8g2, max_x, 0, max_x, corner_size);

    // corner-tr-h
    u8g2_DrawLine(&u8g2, max_x - corner_size, 0, max_x, 0);
#endif
    const int left = 67;
    // time_box
    u8g2_DrawRFrame(&u8g2, left + 3, 3, 121, 25, 2);

    // time
    u8g2_SetFont(&u8g2, u8g2_font_profont29_mn);
    u8g2_DrawStr(&u8g2, left + 25, 25, strftime_buf);

    // date
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_SetFont(&u8g2, u8g2_font_profont12_tf);
    // format date
    strftime(strfdate_buf, sizeof(strfdate_buf), "%a, %b %d, %Y", &timeinfo);
    u8g2_DrawStr(&u8g2, left + 13, 47, strfdate_buf);

    // seconds_bar
    u8g2_DrawRFrame(&u8g2, left + 3, 30, 121, 7, 1);
    u8g2_SetDrawColor(&u8g2, 0);
    // second-bar-mask-top
    u8g2_DrawLine(&u8g2, left + 8, 30, left + 118, 30);
    // second-bar-mask-bottom
    u8g2_DrawLine(&u8g2, left + 8, 36, left + 118, 36);


    // fill seconds bar
    u8g2_SetDrawColor(&u8g2, 1);
    uint8_t x, y0, y1;
    for (int i = 1; i <= timeinfo.tm_sec; i++)
    {
        x = left + 5 + (i - 1) * 2;
        y0 = 32;
        y1 = 34;
        if (i % 5 == 0)
        {
            y1 = 35;
        }
        if (i % 10 == 0)
        {
            y0 = 31;
            y1 = 35;
        }
        u8g2_DrawLine(&u8g2, x, y0, x, y1);
    }

#if SHOW_BRIGHTNESS_LEVEL
    // display brightness level
    char textbuf[32];
    sprintf(textbuf, "%d", display_brightness);
    u8g2_int_t w = u8g2_GetStrWidth(&u8g2, textbuf);
    u8g2_DrawStr(&u8g2, 256 - w - 5, 48, textbuf);
#endif
    sec = !sec;
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
    adc_oneshot_unit_init_cfg_t brightness_adc_config = {
        .unit_id = DISPLAY_BRIGHTNESS_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    ret = adc_oneshot_new_unit(&brightness_adc_config, &brightness_adc_handle);
    ESP_RETURN_ON_ERROR(ret, TAG, "ADC unit init failed");

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    ret = adc_oneshot_config_channel(brightness_adc_handle, ADC_CHANNEL_4, &config);
    ESP_RETURN_ON_ERROR(ret, TAG, "ADC channel init failed");

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
        ESP_RETURN_ON_ERROR(ret, TAG, "ADC calibration failed");
        calibrated = true;
    }
    BaseType_t result = xTaskCreate(adc_task, "ADC_Task", ADC_TASK_STACK_SIZE, NULL, ADC_TASK_PRIORITY, &adc_task_handle);
    return result == pdPASS ? ESP_OK : ESP_FAIL;
}

static void setup_timers(esp_timer_cb_t cb)
{
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

    // setup display
    ret = setup_GP1287(&display_handle);
    ESP_RETURN_ON_ERROR(ret, TAG, "GP1287 setup failed");
    ret = init_brightness_sensor();
    ESP_RETURN_ON_ERROR(ret, TAG, "Brightness sensor setup failed");

    //setup display buffer
    gbuf = (uint8_t *)heap_caps_calloc(1, buf_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
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

void set_time(time_t t)
{
    now = t;
    render_display();
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