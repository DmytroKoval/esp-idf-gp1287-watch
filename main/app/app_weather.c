#include "app_weather.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "sdkconfig.h"

#define WEATHER_URL_FORMAT "https://api.openweathermap.org/data/3.0/onecall?lat=48.5168&lon=34.6069&exclude=minutely,hourly,daily,alerts&units=metric&appid=%s"
#define WEATHER_POLL_INTERVAL_MS (60 * 60 * 1000)
#define WEATHER_RESPONSE_SIZE 4096
#define WEATHER_TASK_STACK_SIZE 6144

static const char *TAG = "WEATHER";
static app_weather_snapshot_t weather_snapshot;
static portMUX_TYPE weather_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t weather_task_handle;

static esp_err_t fetch_weather(char *response, size_t response_size)
{
    char url[320];
    const int url_length = snprintf(url, sizeof(url), WEATHER_URL_FORMAT,
                                    CONFIG_WEATHER_API_KEY);
    if (url_length < 0 || url_length >= sizeof(url))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    const esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 5000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t result = esp_http_client_open(client, 0);
    if (result == ESP_OK)
    {
        esp_http_client_fetch_headers(client);
        if (esp_http_client_get_status_code(client) != 200)
        {
            result = ESP_FAIL;
        }
    }

    size_t response_length = 0;
    while (result == ESP_OK && response_length + 1 < response_size)
    {
        const int read = esp_http_client_read(client, response + response_length,
                                               response_size - response_length - 1);
        if (read < 0)
        {
            result = ESP_FAIL;
            break;
        }
        if (read == 0)
        {
            break;
        }
        response_length += (size_t)read;
    }

    response[response_length] = '\0';
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return result;
}

static bool parse_weather(const char *response, app_weather_snapshot_t *snapshot)
{
    cJSON *root = cJSON_Parse(response);
    if (root == NULL)
    {
        return false;
    }

    const cJSON *current = cJSON_GetObjectItemCaseSensitive(root, "current");
    if (!cJSON_IsObject(current))
    {
        cJSON_Delete(root);
        return false;
    }

    const cJSON *temperature = cJSON_GetObjectItemCaseSensitive(current, "temp");
    const cJSON *conditions = cJSON_GetObjectItemCaseSensitive(current, "weather");
    const cJSON *condition = cJSON_GetArrayItem(conditions, 0);
    const cJSON *condition_code = cJSON_GetObjectItemCaseSensitive(condition, "id");
    const cJSON *current_time = cJSON_GetObjectItemCaseSensitive(current, "dt");
    const cJSON *sunrise = cJSON_GetObjectItemCaseSensitive(current, "sunrise");
    const cJSON *sunset = cJSON_GetObjectItemCaseSensitive(current, "sunset");

    const bool valid = cJSON_IsArray(conditions) && cJSON_IsObject(condition)
                       && cJSON_IsNumber(temperature) && cJSON_IsNumber(condition_code)
                       && cJSON_IsNumber(current_time) && cJSON_IsNumber(sunrise)
                       && cJSON_IsNumber(sunset);
    if (valid)
    {
        const double value = temperature->valuedouble;
        snapshot->valid = true;
        snapshot->temperature_c = (int)(value + (value < 0 ? -0.5 : 0.5));
        snapshot->condition_code = condition_code->valueint;
        snapshot->is_night = current_time->valuedouble < sunrise->valuedouble
                             || current_time->valuedouble >= sunset->valuedouble;
    }

    cJSON_Delete(root);
    return valid;
}

static void refresh_weather(void)
{
    if (CONFIG_WEATHER_API_KEY[0] == '\0')
    {
        ESP_LOGW(TAG, "OpenWeather API key is not configured");
        return;
    }

    static char response[WEATHER_RESPONSE_SIZE];
    const esp_err_t result = fetch_weather(response, sizeof(response));
    app_weather_snapshot_t next_snapshot = {0};
    if (result != ESP_OK || !parse_weather(response, &next_snapshot))
    {
        ESP_LOGW(TAG, "Weather request failed; keeping current weather");
        return;
    }

    portENTER_CRITICAL(&weather_lock);
    weather_snapshot = next_snapshot;
    portEXIT_CRITICAL(&weather_lock);
}

static void weather_task(void *arg)
{
    while (true)
    {
        refresh_weather();
        vTaskDelay(pdMS_TO_TICKS(WEATHER_POLL_INTERVAL_MS));
    }
}

void start_weather_service(void)
{
    if (weather_task_handle != NULL)
    {
        return;
    }

    if (xTaskCreate(weather_task, "weather", WEATHER_TASK_STACK_SIZE, NULL, 5,
                    &weather_task_handle) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to start weather task");
        weather_task_handle = NULL;
    }
}

app_weather_snapshot_t app_weather_get_snapshot(void)
{
    app_weather_snapshot_t snapshot;
    portENTER_CRITICAL(&weather_lock);
    snapshot = weather_snapshot;
    portEXIT_CRITICAL(&weather_lock);
    return snapshot;
}
