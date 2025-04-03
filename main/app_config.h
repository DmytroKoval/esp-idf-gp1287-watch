#pragma once
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#ifdef __cplusplus
extern "C"
{
#endif

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define WIFI_MAXIMUM_RETRY 5
#define WIFI_SSID "AMBER_Network"
#define WIFI_PASS "001234500"

#define LCD_SPI_HOST SPI2_HOST
#define DISPLAY_SPI_DC GPIO_NUM_NC
#define DISPLAY_RESET GPIO_NUM_11
#define DISPLAY_SPI_MOSI GPIO_NUM_10
#define DISPLAY_SPI_CS GPIO_NUM_9
#define DISPLAY_SPI_SCLK GPIO_NUM_8
#define DISPLAY_FILEMENT_EN GPIO_NUM_7

/// @brief ADC channel for brightness sensor
#define DISPLAY_BRIGHTNESS_ADC_CHANNEL ADC_CHANNEL_4 

/// @brief ADC unit for brightness sensor
#define DISPLAY_BRIGHTNESS_ADC_UNIT ADC_UNIT_1       

/// @brief Maximum brightness value (0..1023)
#define MAX_BRIGHTNESS 600

/// @brief Minimum brightness value
#define MIN_BRIGHTNESS 4                             

/// @brief Default brightness value (min_brightness...max_brightness)
#define DEFAULT_BRIGHTNESS 20                        

/// @brief Render brightness level (0, 1)
#define SHOW_BRIGHTNESS_LEVEL 0

#define TIME_UPDATE_INTERVAL (500 * 1000)

#define SNTP_SYNC_INTERVAL (pdMS_TO_TICKS(60 * 60 * 1000))

#ifdef __cplusplus
}
#endif

#endif // APP_CONFIG_H