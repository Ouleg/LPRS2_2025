#include "rtc.h"  //                            <3  Bebiiiii <3 
#include "driver/i2c.h"
#include "esp_log.h"

// Ensure no C++ linkage block is opened in this C file

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQ_HZ 100000
#define PCF8563_ADDR 0x51

#define TAG "RTC"

static uint8_t bcd2dec(uint8_t val) { return ((val / 16 * 10) + (val % 16)); }
static uint8_t dec2bcd(uint8_t val) { return ((val / 10 * 16) + (val % 10)); }

esp_err_t watchy_rtc_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    ESP_LOGI(TAG, "RTC I2C initialized");
    return ESP_OK;
}

esp_err_t watchy_rtc_get_time(struct tm *timeinfo) {
    uint8_t reg = 0x02;
    uint8_t data[7] = {0};

    esp_err_t res = i2c_master_write_read_device(
        I2C_MASTER_NUM, PCF8563_ADDR, &reg, 1, data, 7, 1000 / portTICK_PERIOD_MS);

    if (res != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %d", res);
        return ESP_FAIL;
    }

    timeinfo->tm_sec  = bcd2dec(data[0] & 0x7F);
    timeinfo->tm_min  = bcd2dec(data[1] & 0x7F);
    timeinfo->tm_hour = bcd2dec(data[2] & 0x3F);
    timeinfo->tm_mday = bcd2dec(data[3] & 0x3F);
    timeinfo->tm_wday = bcd2dec(data[4] & 0x07);
    timeinfo->tm_mon  = bcd2dec(data[5] & 0x1F) - 1;
    timeinfo->tm_year = bcd2dec(data[6]) + 100;

    return ESP_OK;
}

esp_err_t watchy_rtc_set_time(struct tm *timeinfo) {
    uint8_t data[8];
    data[0] = 0x02;
    data[1] = dec2bcd(timeinfo->tm_sec);
    data[2] = dec2bcd(timeinfo->tm_min);
    data[3] = dec2bcd(timeinfo->tm_hour);
    data[4] = dec2bcd(timeinfo->tm_mday);
    data[5] = dec2bcd(timeinfo->tm_wday);
    data[6] = dec2bcd(timeinfo->tm_mon + 1);
    data[7] = dec2bcd(timeinfo->tm_year - 100);

    esp_err_t res = i2c_master_write_to_device(
        I2C_MASTER_NUM, PCF8563_ADDR, data, 8, 1000 / portTICK_PERIOD_MS);

    if (res != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %d", res);
    } else {
        ESP_LOGI(TAG, "Time set");
    }
    return ESP_OK;
}
