#include <stdio.h>
#include <inttypes.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ms5611.h>
#include "driver/i2c.h"


static const char *TAG = "ms5611-example";

SemaphoreHandle_t i2c_port_mutex;

void ms5611_test(void *pvParameters)
{

    ms5611_init(&i2c_port_mutex, MS5611_OSR_4096);

    float temperature;
    int32_t pressure;
    esp_err_t res;
    vTaskDelay(pdMS_TO_TICKS(500));
    while (1)
    {
        // we can change oversampling ratio at any time:
        // dev.osr = MS5637_OSR_256

        res = ms5611_get_sensor_data(&pressure, &temperature);
        if (res != ESP_OK)
        {
            ESP_LOGE(TAG, "Temperature/pressure reading failed: %d (%s)", res, esp_err_to_name(res));
            continue;
        }
        ESP_LOGI(TAG, "Pressure: %" PRIi32 " Pa, Temperature: %.2f C\n", pressure, temperature);
    }
}

void app_main()
{
    // Init i2cdev library
    //ESP_ERROR_CHECK(i2cdev_init());

    i2c_port_mutex = xSemaphoreCreateBinary();

    // int i2c_master_port = 0;
    i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = CONFIG_I2C_MASTER_SDA,         // select GPIO specific to your project
    .sda_pullup_en = GPIO_PULLUP_DISABLE,
    .scl_io_num = CONFIG_I2C_MASTER_SCL,         // select GPIO specific to your project
    .scl_pullup_en = GPIO_PULLUP_DISABLE,
    .master.clk_speed = CONFIG_I2C_MASTER_FREQUENCY,  // select frequency specific to your project
    .clk_flags = 0,                          // you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here
};
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));
    
    vTaskDelay(pdMS_TO_TICKS(500));

    xTaskCreatePinnedToCore(ms5611_test, "ms5611_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}


