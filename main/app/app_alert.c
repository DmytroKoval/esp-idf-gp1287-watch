#include "app_alert.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#define ALERTS_URL "https://api.alerts.in.ua/v1/iot/active_air_raid_alerts/42.json"
#define ALERTS_POLL_INTERVAL_MS 60000
#define ALERTS_TASK_STACK_SIZE 4096

static const char *TAG = "ALERTS";
static app_alert_state_t alert_state;
static portMUX_TYPE alert_state_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t alert_task_handle;

extern const char alerts_globalsign_root_ca_pem_start[]
    asm("_binary_alerts_globalsign_root_ca_pem_start");

static bool is_active_status(const char *response)
{
    while (isspace((unsigned char)*response))
    {
        ++response;
    }

    if (*response == '"')
    {
        ++response;
    }

    return response[0] == 'A' || response[0] == 'P';
}

static bool is_known_status(const char *response)
{
    while (isspace((unsigned char)*response))
    {
        ++response;
    }

    if (*response == '"')
    {
        ++response;
    }

    return response[0] == 'A' || response[0] == 'P' || response[0] == 'N';
}

static esp_err_t fetch_alert_status(char *response, size_t response_size, int *http_status)
{
    *http_status = -1;
    const esp_http_client_config_t config = {
        .url = ALERTS_URL,
        .method = HTTP_METHOD_GET,
        .cert_pem = alerts_globalsign_root_ca_pem_start,
        .timeout_ms = 5000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    char authorization[sizeof(CONFIG_ALERTS_API_TOKEN) + sizeof("Bearer ")];
    snprintf(authorization, sizeof(authorization), "Bearer %s", CONFIG_ALERTS_API_TOKEN);
    esp_http_client_set_header(client, "Authorization", authorization);

    esp_err_t result = esp_http_client_open(client, 0);
    if (result == ESP_OK)
    {
        esp_http_client_fetch_headers(client);
        *http_status = esp_http_client_get_status_code(client);
        if (*http_status != 200)
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

static void poll_alerts(void)
{
    if (CONFIG_ALERTS_API_TOKEN[0] == '\0')
    {
        ESP_LOGW(TAG, "alerts.in.ua token is not configured");
        return;
    }

    char response[8] = {0};
    int http_status = -1;
    const esp_err_t result = fetch_alert_status(response, sizeof(response), &http_status);
    if (result != ESP_OK || !is_known_status(response))
    {
        ESP_LOGW(TAG, "Alert status request failed: result=%s, HTTP=%d, status=%c",
                 esp_err_to_name(result), http_status,
                 response[0] == '\0' ? '-' : response[0]);
        return;
    }

    time_t detected_at;
    time(&detected_at);

    portENTER_CRITICAL(&alert_state_lock);
    app_alert_state_apply(&alert_state, is_active_status(response), detected_at,
                          esp_timer_get_time());
    portEXIT_CRITICAL(&alert_state_lock);
}

static void alert_task(void *arg)
{
    while (true)
    {
        poll_alerts();
        vTaskDelay(pdMS_TO_TICKS(ALERTS_POLL_INTERVAL_MS));
    }
}

void start_alert_service(void)
{
    if (alert_task_handle != NULL)
    {
        return;
    }

    app_alert_state_init(&alert_state);
    if (xTaskCreate(alert_task, "alerts", ALERTS_TASK_STACK_SIZE, NULL, 5,
                    &alert_task_handle) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to start alert task");
        alert_task_handle = NULL;
    }
}

app_alert_view_t app_alert_get_view(void)
{
    app_alert_state_t state;
    portENTER_CRITICAL(&alert_state_lock);
    state = alert_state;
    portEXIT_CRITICAL(&alert_state_lock);

    return app_alert_state_view(&state, esp_timer_get_time());
}
