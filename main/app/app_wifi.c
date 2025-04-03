#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <limits.h>

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
#include <esp_timer.h>
#include <esp_netif.h>
#include <esp_netif_sntp.h>
#include "app.h"
#include "app_config.h"

#define TAG "GP1287-WIFI"

static EventGroupHandle_t s_wifi_event_group = NULL;
static int s_retry_num = 0;
static TimerHandle_t time_sync_timer = NULL;

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
            case WIFI_EVENT_WIFI_READY:
                ESP_LOGI(TAG, ">> WIFI_EVENT_WIFI_READY");
                break;
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, ">> WIFI_EVENT_STA_START");
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_STOP:
                ESP_LOGI(TAG, ">> WIFI_EVENT_STA_STOP");
                break;
            case WIFI_EVENT_STA_AUTHMODE_CHANGE:
                {
                    wifi_event_sta_authmode_change_t* event = (wifi_event_sta_authmode_change_t*)event_data;
                    ESP_LOGI(TAG, ">> WIFI_EVENT_STA_AUTHMODE_CHANGE old:%d, new:%d", event->old_mode, event->new_mode);
                }
                break;
            case WIFI_EVENT_STA_CONNECTED:
                {
                    /* code */
                }
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                {
                    if (s_retry_num < WIFI_MAXIMUM_RETRY)
                    {
                        esp_wifi_connect();
                        s_retry_num++;
                        ESP_LOGI(TAG, ">> retry to connect to the AP");
                    }
                    else
                    {
                        xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                    }
                    ESP_LOGI(TAG, ">> connect to the AP fail");
                }
                break;
            default:
                break;
        }
    }

    if (event_base == IP_EVENT)
    {
        switch (event_id)
        {
            case IP_EVENT_STA_GOT_IP:
                {
                    ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
                    ESP_LOGI(TAG, ">> got ip:" IPSTR, IP2STR(&event->ip_info.ip));
                    s_retry_num = 0;
                    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
                }
                break;

            default:
                break;
        }
    }
}

static void time_sync_task(void* arg)
{
    const int retry_count = 15;
    int retry = 0;
    esp_wifi_stop();
    esp_wifi_start();

    esp_wifi_connect();

    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_config_t sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    sntp_config.start = true; // start SNTP service explicitly (after connecting)
    // config.server_from_dhcp = true;             // accept NTP offers from DHCP server, if any (need to enable *before* connecting)
    // config.index_of_first_server = 1;           // updates from server num 1, leaving server 0 (from DHCP) intact
    // sntp_config.renew_servers_after_new_IP = true; // let esp-netif update configured SNTP server(s) after receiving DHCP lease
    // sntp_config.ip_event_to_renew = IP_EVENT_STA_GOT_IP;
    // sntp_config.sync_cb = time_sync_notification_cb; // only if we need the notification function
    esp_netif_sntp_init(&sntp_config);

    esp_netif_sntp_start();

    while (esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT && ++retry < retry_count)
    {
        ESP_LOGI(TAG, ">> Waiting for system time to be set... (%d/%d)", retry, retry_count);
    }
    if (retry == retry_count)
    {
        time_t now = 0;
        struct tm timeinfo = { 0 };
        time(&now);
        localtime_r(&now, &timeinfo);
        if (timeinfo.tm_year < (2020 - 1900))
        {
            ESP_LOGE(TAG, ">> Time not set. Restarting...");
            esp_restart();
        }
    }
    ESP_LOGI(TAG, ">> Time set.");
    esp_netif_sntp_deinit();

    vTaskDelete(NULL);
}

bool init_wifi()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &wifi_event_handler,
        NULL,
        &instance_got_ip));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    esp_wifi_set_storage(WIFI_STORAGE_FLASH);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .bssid_set = false,
            .threshold.authmode = WIFI_AUTH_WPA2_WPA3_PSK},
    };

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, ">> connected to ap SSID: %s", WIFI_SSID);
        
        //xTaskCreate(time_sync_task, "time_sync_task", 16 * 1024, NULL, 5, NULL);

        /*
            RSSI > -30 dBm	 Amazing
            RSSI < -55 dBm	 Very good signal
            RSSI < -67 dBm	 Fairly Good
            RSSI < -70 dBm	 Okay
            RSSI < -80 dBm	 Not good
            RSSI < -90 dBm	 Extremely weak signal (unusable)
        */
       return true;
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID: %s", WIFI_SSID);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    return false;
    // vTaskDelete(NULL);
}

static void IRAM_ATTR time_sync_timer_cb(TimerHandle_t xTimer)
{
    printf(">> Time sync timer callback\n");
    // ESP_LOGI(TAG, ">> Time sync timer callback start");
    xTaskCreate(time_sync_task, "time_sync_task", 16 * 1024, NULL, 5, NULL);
}

void start_sntp_service()
{
    ESP_LOGI(TAG, ">> Starting SNTP service...");

    time_sync_task(NULL);

    time_sync_timer = xTimerCreate("time_sync", SNTP_SYNC_INTERVAL, pdTRUE, NULL, time_sync_timer_cb);
    esp_err_t ret = xTimerStart(time_sync_timer, pdMS_TO_TICKS(10 * 1000));
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create time sync timer");
    }

}