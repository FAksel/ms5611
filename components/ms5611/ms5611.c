/*
 * Copyright (c) 2016 Bernhard Guillon <Bernhard.Guillon@begu.org>
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 */

#include "ms5611.h"
#include "driver/i2c.h"
#include <stddef.h>
#include <esp_system.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define I2C_PORT I2C_NUM_0
#define I2C_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY // 400 kHz
#define CONFIG_MS5611_ADDRESS MS5611_ADDR

#define CMD_CONVERT_D1 0x40
#define CMD_CONVERT_D2 0x50
#define CMD_ADC_READ 0x00
#define CMD_RESET 0x1E

#define PROM_ADDR_SENS 0xa2
#define PROM_ADDR_OFF 0xa4
#define PROM_ADDR_TCS 0xa6
#define PROM_ADDR_TCO 0xa8
#define PROM_ADDR_T_REF 0xaa
#define PROM_ADDR_TEMPSENS 0xac

#define CHECK(x)                \
    do                          \
    {                           \
        esp_err_t __;           \
        if ((__ = x) != ESP_OK) \
            return __;          \
    } while (0)
#define CHECK_ARG(VAL)                  \
    do                                  \
    {                                   \
        if (!(VAL))                     \
            return ESP_ERR_INVALID_ARG; \
    } while (0)

static const char *TAG = "ms5611";


static SemaphoreHandle_t *i2cPortMutex;
static ms5611_osr_t dev_osr;
static ms5611_config_data_t config_data = {0};

static inline esp_err_t send_command(uint8_t cmd)
{
    // return i2c_dev_write(&dev->i2c_dev, NULL, 0, &cmd, 1);
    return i2c_master_write_to_device(I2C_PORT, CONFIG_MS5611_ADDRESS, &cmd, 1, pdMS_TO_TICKS(5));
}

static inline esp_err_t read_reg(uint8_t reg, uint8_t *readBuf, size_t read_size)
{

    return i2c_master_write_read_device(I2C_PORT, CONFIG_MS5611_ADDRESS, &reg, 1, readBuf, read_size, pdMS_TO_TICKS(5));
}

// static inline esp_err_t read_reg(uint8_t reg, uint8_t *readBuf, size_t read_size)
// {

//     uint8_t dummy;
//     i2c_cmd_handle_t cmd;
//     cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (CONFIG_MS5611_ADDRESS << 1) | I2C_MASTER_WRITE, 1);
//     i2c_master_write(cmd, &reg, 1,1);
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (CONFIG_MS5611_ADDRESS << 1) | I2C_MASTER_READ, 1);
//     i2c_master_read(cmd,readBuf, read_size, I2C_MASTER_LAST_NACK);
//     //i2c_master_read_byte(cmd, &readBuf[2], I2C_MASTER_NACK);
//     i2c_master_stop(cmd);
//     esp_err_t status = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
//     if (status != 0) ESP_LOGE("I2C","%d:%s",status,esp_err_to_name(status));
//     i2c_cmd_link_delete(cmd);
//     return status;
//     //return i2c_master_write_read_device(I2C_PORT, CONFIG_MS5611_ADDRESS, &reg, 1, readBuf, read_size, pdMS_TO_TICKS(5));
// }


static inline uint16_t shuffle(uint8_t val1, uint8_t val2)
{
    uint16_t res = val1;

    res = (res << 8) | val2;
    return res;
    //return ((val & 0xff00) >> 8) | ((val & 0xff) << 8);
}

static inline esp_err_t read_prom()
{
    uint8_t tmp[2];

    // FIXME calculate CRC (AN502)

    CHECK(read_reg(PROM_ADDR_SENS, tmp, 2));
    config_data.sens = shuffle(tmp[0], tmp[1]);
    CHECK(read_reg(PROM_ADDR_OFF, tmp, 2));
    config_data.off = shuffle(tmp[0], tmp[1]);
    CHECK(read_reg(PROM_ADDR_TCS, tmp, 2));
    config_data.tcs = shuffle(tmp[0], tmp[1]);
    CHECK(read_reg(PROM_ADDR_TCO, tmp, 2));
    config_data.tco = shuffle(tmp[0], tmp[1]);
    CHECK(read_reg(PROM_ADDR_T_REF, tmp, 2));
    config_data.t_ref = shuffle(tmp[0], tmp[1]);
    CHECK(read_reg(PROM_ADDR_TEMPSENS, tmp, 2));
    config_data.tempsens = shuffle(tmp[0], tmp[1]);

    return ESP_OK;
}

static esp_err_t read_adc(uint32_t *result)
{
    uint8_t tmp[3];

    CHECK(read_reg(0, tmp, 3)); // reg 0x00
    *result = (tmp[0] << 16) | (tmp[1] << 8) | tmp[2];

    ESP_LOGI("ms5611", "para 0 to 2: %d, %d, %d", tmp[0], tmp[1], tmp[2]);
    //  If we are to fast the ADC will return 0 instead of the actual result
    return *result == 0 ? ESP_ERR_INVALID_RESPONSE : ESP_OK;
}

static void wait_conversion()
{
    uint8_t ms = 10;
    // switch (dev->osr)
    // {
    //     case MS5611_OSR_256: us = 500; break;   // 0.5ms
    //     case MS5611_OSR_512: us = 1100; break;  // 1.1ms
    //     case MS5611_OSR_1024: us = 2100; break; // 2.1ms
    //     case MS5611_OSR_2048: us = 4100; break; // 4.1ms
    //     case MS5611_OSR_4096: us = 8220; break; // 8.22ms
    // }
    // ets_delay_us(us);

    switch (dev_osr)
    {
    case MS5611_OSR_256: ms = 1; break; // 0.5ms
    case MS5611_OSR_512: ms = 2; break; // 1.1ms
    case MS5611_OSR_1024: ms = 3;break; // 2.1ms
    case MS5611_OSR_2048: ms = 5; break; // 4.1ms
    case MS5611_OSR_4096: ms = 10; break; // 8.22ms
    }
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static inline esp_err_t get_raw_temperature(uint32_t *result)
{
    CHECK(send_command(CMD_CONVERT_D2 + dev_osr));
    wait_conversion();
    CHECK(read_adc(result));

    return ESP_OK;
}

static inline esp_err_t get_raw_pressure(uint32_t *result)
{
    CHECK(send_command(CMD_CONVERT_D1 + dev_osr));
    wait_conversion();
    CHECK(read_adc(result));
    return ESP_OK;
}

static esp_err_t ms5611_reset()
{
    send_command(CMD_RESET);
    return ESP_OK;
}

/////////////////////////Public//////////////////////////////////////

esp_err_t ms5611_init(SemaphoreHandle_t *sem_handle, ms5611_osr_t osr)
{

    i2cPortMutex = sem_handle;
    dev_osr = osr;

    // First of all we need to reset the chip
    CHECK(ms5611_reset());
    // Wait a bit for the device to reset
    vTaskDelay(pdMS_TO_TICKS(10));
    // Get the config
    CHECK(read_prom());
    ESP_LOGI("ms5611", "Calibration data, C1 to C6: %u, %u, %u, %u, %u, %u", config_data.sens, config_data.off, config_data.tcs, config_data.tco, config_data.t_ref, config_data.tempsens);
    return ESP_OK;
}

esp_err_t ms5611_get_sensor_data(int32_t *pressure, float *temperature)
{

    // Second order temperature compensation see datasheet p8
    uint32_t raw_pressure = 0;
    uint32_t raw_temperature = 0;

    get_raw_pressure(&raw_pressure);
    get_raw_temperature(&raw_temperature);

    ESP_LOGI("ms5611", "raw P: %lu, raw T: %lu", raw_pressure, raw_temperature);

    // dT = D2 - T_ref = D2 - C5 * 2^8
    int32_t dt = raw_temperature - ((int32_t)config_data.t_ref << 8);
    // Actual temperature (-40...85C with 0.01 resolution)
    // TEMP = 20C +dT * TEMPSENSE =2000 + dT * C6 / 2^23
    int64_t temp = (2000 + (int64_t)dt * config_data.tempsens / 8388608);
    // Offset at actual temperature
    // OFF=OFF_t1 + TCO * dT = OFF_t1(C2) * 2^16 + (C4*dT)/2^7
    int64_t off = (int64_t)((int64_t)config_data.off * 65536) + (((int64_t)config_data.tco * dt) / 128);
    // Sensitivity at actual temperature
    // SENS=SENS_t1 + TCS *dT = SENS_t1(C1) *2^15 + (TCS(C3) *dT)/2^8
    int64_t sens = (int64_t)(((int64_t)config_data.sens) * 32768) + (((int64_t)config_data.tcs * dt) / 256);

    // Set defaults for temp >= 2000
    int64_t t_2 = 0;
    int64_t off_2 = 0;
    int64_t sens_2 = 0;
    int64_t help = 0;
    if (temp < 2000)
    {
        // Low temperature
        t_2 = ((dt * dt) >> 31); // T2 = dT^2/2^31
        help = (temp - 2000);
        help = 5 * help * help;
        off_2 = help >> 1;  // OFF_2  = 5 * (TEMP - 2000)^2/2^1
        sens_2 = help >> 2; // SENS_2 = 5 * (TEMP - 2000)^2/2^2
        if (temp < -1500)
        {
            // Very low temperature
            help = (temp + 1500);
            help = help * help;
            off_2 = off_2 + 7 * help;             // OFF_2  = OFF_2 + 7 * (TEMP + 1500)^2
            sens_2 = sens_2 + ((11 * help) >> 1); // SENS_2 = SENS_2 + 7 * (TEMP + 1500)^2/2^1
        }
    }

    temp = temp - t_2;
    off = off - off_2;
    sens = sens - sens_2;

    // Temperature compensated pressure (10...1200mbar with 0.01mbar resolution
    // P = digital pressure value  * SENS - OFF = (D1 * SENS/2^21 -OFF)/2^15
    *pressure = (int32_t)(((int64_t)raw_pressure * (sens / 0x200000) - off) / 32768);
    *temperature = (float)temp / 100.0;

    return ESP_OK;
}

