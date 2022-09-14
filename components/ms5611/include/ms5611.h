/*
 * Copyright (c) 2016 Bernhard Guillon <Bernhard.Guillon@begu.org>
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
*/

#ifndef __MS5611_H__
#define __MS5611_H__

#include <stdint.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>


#ifdef __cplusplus
extern "C" {
#endif

#define MS5611_ADDR     0x77

/**
 * Oversampling ratio
 */
typedef enum
{
    MS5611_OSR_256  = 0x00, //!< 256 samples per measurement
    MS5611_OSR_512  = 0x02, //!< 512 samples per measurement
    MS5611_OSR_1024 = 0x04, //!< 1024 samples per measurement
    MS5611_OSR_2048 = 0x06, //!< 2048 samples per measurement
    MS5611_OSR_4096 = 0x08  //!< 4096 samples per measurement
} ms5611_osr_t;

/**
 * Configuration data
 */
typedef struct
{
    uint16_t sens;       //!< C1 Pressure sensitivity                             | SENS_t1
    uint16_t off;        //!< C2 Pressure offset                                  | OFF_t1
    uint16_t tcs;        //!< C3 Temperature coefficient of pressure sensitivity  | TCS
    uint16_t tco;        //!< C4 Temperature coefficient of pressure offset       | TCO
    uint16_t t_ref;      //!< C5 Reference temperature                            | T_ref
    uint16_t tempsens;   //!< C6 Temperature coefficient of the temperature       | TEMPSENSE
} ms5611_config_data_t;

/**
 * @brief Init MS5611-01BA03
 *
 * Reset device and read calibration data
 *
 * @param sem i2c port protecton semaphore
 * @param osr Oversampling ratio
 * @return `ESP_OK` on success
 */
esp_err_t ms5611_init(SemaphoreHandle_t *sem_handle, ms5611_osr_t osr);

/**
 * @brief Measure pressure and temperature
 *
 * @param[out] pressure Pressure, Pa
 * @param[out] temperature Temperature, degrees Celsius
 * @return `ESP_OK` on success
 */
esp_err_t ms5611_get_sensor_data(int32_t *pressure, float *temperature);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __MS5611_H__ */