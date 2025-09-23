#ifndef RTC_H
#define RTC_H

#include "esp_err.h"
#include "esp_types.h"
#include <time.h>  // Za struct tm

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t watchy_rtc_init(void);
esp_err_t watchy_rtc_set_time(struct tm *timeinfo);
esp_err_t watchy_rtc_get_time(struct tm *timeinfo);

#ifdef __cplusplus
}
#endif

#endif // RTC_H
