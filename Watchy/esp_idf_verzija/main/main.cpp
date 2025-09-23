#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

extern "C" {
    #include "rtc.h"
    #include <time.h>
}

extern "C" void app_main() {
    esp_err_t err = watchy_rtc_init();
    if (err != ESP_OK) {
        ESP_LOGE("MAIN", "RTC init failed: %s", esp_err_to_name(err));
    }

    struct tm now;
    esp_err_t gt = watchy_rtc_get_time(&now);
    if (gt != ESP_OK) {
        ESP_LOGE("MAIN", "RTC get time failed: %s", esp_err_to_name(gt));
    }
    ESP_LOGI("MAIN", "Time: %02d:%02d:%02d %02d/%02d/%04d",
             now.tm_hour, now.tm_min, now.tm_sec,
             now.tm_mday, now.tm_mon + 1, now.tm_year + 1900);
}
