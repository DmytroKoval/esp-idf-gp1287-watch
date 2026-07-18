#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_err.h>
#include <nvs_flash.h>
#include <esp_system.h>
#include <esp_sntp.h>
#include <esp_netif.h>
#include <esp_netif_sntp.h>
#include "app.h"
#include "app_config.h"

#define TAG "WIFI"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define SNTP_SYNC_RETRY_COUNT    15
#define SNTP_SYNC_WAIT_MS        2000
#define TIME_SYNC_TASK_STACK     (8 * 1024)
#define WIFI_RECONNECT_WAIT_MS   5000

static EventGroupHandle_t s_wifi_event_group = NULL;
static int s_retry_num = 0;
static TimerHandle_t time_sync_timer = NULL;
static volatile bool s_wifi_connected = false;

/* ── Event handler ──────────────────────────────────────────────── */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "STA started, connecting...");
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "STA connected to AP");
            break;
        case WIFI_EVENT_STA_STOP:
            ESP_LOGW(TAG, "STA stopped");
            s_wifi_connected = false;
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            s_wifi_connected = false;
            if (s_retry_num < CONFIG_WIFI_MAXIMUM_RETRY)
            {
                esp_wifi_connect();
                s_retry_num++;
                ESP_LOGW(TAG, "Reconnect attempt %d/%d", s_retry_num, CONFIG_WIFI_MAXIMUM_RETRY);
            }
            else
            {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                ESP_LOGE(TAG, "Connection failed after %d retries", CONFIG_WIFI_MAXIMUM_RETRY);
            }
            break;
        default:
            break;
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        s_wifi_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ── NVS ────────────────────────────────────────────────────────── */

static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "Erasing NVS flash...");
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "NVS erase failed");
        ret = nvs_flash_init();
    }
    return ret;
}

/* ── WiFi STA ───────────────────────────────────────────────────── */

static esp_err_t init_wifi_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop create failed");

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "WiFi init failed");

    ESP_RETURN_ON_ERROR(
        esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                            &wifi_event_handler, NULL, NULL),
        TAG, "WiFi event handler register failed");
    ESP_RETURN_ON_ERROR(
        esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                            &wifi_event_handler, NULL, NULL),
        TAG, "IP event handler register failed");

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set mode failed");
    esp_wifi_set_storage(WIFI_STORAGE_FLASH);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
            .bssid_set = false,
            .threshold.authmode = WIFI_AUTH_WPA2_WPA3_PSK,
        },
    };

    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), TAG, "set config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "WiFi start failed");

    ESP_LOGI(TAG, "STA init complete, waiting for connection...");
    return ESP_OK;
}

static bool wait_for_connection(void)
{
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);

    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "Connected to SSID: %s", CONFIG_WIFI_SSID);
        return true;
    }

    ESP_LOGE(TAG, "Failed to connect to SSID: %s", CONFIG_WIFI_SSID);
    return false;
}

/* ── SNTP ───────────────────────────────────────────────────────── */

static void time_sync_notification_cb(struct timeval *tv)
{
    time_t now = tv->tv_sec;
    struct tm timeinfo = {0};
    localtime_r(&now, &timeinfo);
    ESP_LOGI(TAG, "Time synced: %s", asctime(&timeinfo));
}

static esp_err_t init_sntp(void)
{
    esp_sntp_config_t sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    sntp_config.start = true;
    sntp_config.ip_event_to_renew = IP_EVENT_STA_GOT_IP;
    sntp_config.sync_cb = time_sync_notification_cb;

    ESP_RETURN_ON_ERROR(esp_netif_sntp_init(&sntp_config), TAG, "SNTP init failed");
    esp_netif_sntp_start();
    return ESP_OK;
}

static void sync_time(void)
{
    int retry = 0;
    while (esp_netif_sntp_sync_wait(SNTP_SYNC_WAIT_MS / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT
           && ++retry < SNTP_SYNC_RETRY_COUNT)
    {
        ESP_LOGI(TAG, "Waiting for time sync... (%d/%d)", retry, SNTP_SYNC_RETRY_COUNT);
    }

    time_t now = 0;
    struct tm timeinfo = {0};
    time(&now);
    localtime_r(&now, &timeinfo);

    if (timeinfo.tm_year < (2020 - 1900))
    {
        ESP_LOGE(TAG, "Time not set after %d retries. Restarting...", SNTP_SYNC_RETRY_COUNT);
        esp_restart();
    }

    ESP_LOGI(TAG, "System time: %s", asctime(&timeinfo));
}

/* ── Periodic time re-sync ──────────────────────────────────────── */

static void time_sync_task(void *arg)
{
    ESP_LOGI(TAG, "Periodic time sync task started");

    if (!s_wifi_connected)
    {
        ESP_LOGW(TAG, "WiFi not connected, requesting reconnect...");
        esp_wifi_connect();

        /* Wait for connection with a timeout instead of syncing blind */
        EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                               WIFI_CONNECTED_BIT,
                                               pdFALSE, pdFALSE,
                                               pdMS_TO_TICKS(WIFI_RECONNECT_WAIT_MS));
        if (!(bits & WIFI_CONNECTED_BIT))
        {
            ESP_LOGW(TAG, "WiFi reconnect timed out, skipping sync");
            vTaskDelete(NULL);
            return;
        }
    }

    sync_time();
    vTaskDelete(NULL);
}

static void time_sync_timer_cb(TimerHandle_t xTimer)
{
    xTaskCreate(time_sync_task, "time_sync", TIME_SYNC_TASK_STACK, NULL, 5, NULL);
}

/* ── Public API ─────────────────────────────────────────────────── */

bool init_wifi(void)
{
    ESP_ERROR_CHECK(init_nvs());

    esp_err_t ret = init_wifi_sta();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "WiFi STA init failed: %s", esp_err_to_name(ret));
        return false;
    }

    if (!wait_for_connection())
    {
        return false;
    }

    ret = init_sntp();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SNTP init failed: %s", esp_err_to_name(ret));
        return false;
    }

    sync_time();
    return true;
}

void start_sntp_service(void)
{
    if (time_sync_timer == NULL)
    {
        time_sync_timer = xTimerCreate("time_sync", SNTP_SYNC_INTERVAL,
                                       pdTRUE, NULL, time_sync_timer_cb);
        if (time_sync_timer == NULL)
        {
            ESP_LOGE(TAG, "Failed to create time sync timer");
            return;
        }
    }

    if (xTimerStart(time_sync_timer, 0) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to start time sync timer");
    }
}
