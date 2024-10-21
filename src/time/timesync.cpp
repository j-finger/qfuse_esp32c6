// src/time/time.cpp
#include "freertos/FreeRTOS.h"    
#include "freertos/task.h"        
#include "time.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include <time.h>
#include <cstring>                
#include <cstdint>


static const char *TAG = "ESP_coms";

/* ----- TIME ----- */

// Set Timezone to AEST-10
void set_timezone(void)
{
    setenv("TZ", "AEST-10", 1);
    tzset();
}

// Initialize SNTP
void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");  // You can specify other NTP servers if needed
    esp_sntp_init();
}





void obtain_time(void)
{
    initialize_sntp();

    // Wait for time to be set
    time_t now = 0;
    struct tm timeinfo = {};
    int retry = 0;
    const int retry_count = 10;

    while (timeinfo.tm_year < (2023 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(pdMS_TO_TICKS(2000));
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (retry == retry_count) {
        ESP_LOGI(TAG, "Failed to get time over NTP.");
    } else {
        ESP_LOGI(TAG, "System time is set.");
    }
}

void print_current_time(void)
{
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time is: %s", strftime_buf);
}
