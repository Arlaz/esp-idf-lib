/**
 * @file sht85.h
 * @defgroup sht85 sht85
 * @{
 *
 * ESP-IDF driver for Sensirion SHT85 digital temperature and humidity sensor
 *
 * Forked from <https://github.com/gschorcht/sht85-esp-idf>
 *
 * Copyright (c) 2017 Gunar Schorcht <https://github.com/gschorcht>\n
 * Copyright (C) 2019 Ruslan V. Uss <https://github.com/UncleRus>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __SHT85_H__
#define __SHT85_H__

#include <esp_err.h>
#include <i2cdev.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SHT85_I2C_ADDR 0x44

#define SHT85_RAW_DATA_SIZE 6

typedef uint8_t sht85_raw_data_t[SHT85_RAW_DATA_SIZE];

/**
 * Possible measurement modes
 */
typedef enum {
    SHT85_SINGLE_SHOT = 0,  //!< one single measurement
    SHT85_PERIODIC_05MPS,   //!< periodic with 0.5 measurements per second (mps)
    SHT85_PERIODIC_1MPS,    //!< periodic with   1 measurements per second (mps)
    SHT85_PERIODIC_2MPS,    //!< periodic with   2 measurements per second (mps)
    SHT85_PERIODIC_4MPS,    //!< periodic with   4 measurements per second (mps)
    SHT85_PERIODIC_10MPS    //!< periodic with  10 measurements per second (mps)
} sht85_mode_t;

/**
 * Possible repeatability modes
 */
typedef enum { SHT85_HIGH = 0, SHT85_MEDIUM, SHT85_LOW } sht85_repeat_t;

/**
 * Device descriptor
 */
typedef struct {
    i2c_dev_t i2c_dev;  //!< I2C device descriptor

    sht85_mode_t mode;             //!< used measurement mode
    sht85_repeat_t repeatability;  //!< used repeatability

    bool meas_started;         //!< indicates whether measurement started
    uint64_t meas_start_time;  //!< measurement start time in us
    bool meas_first;           //!< first measurement in periodic mode
} sht85_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev       Device descriptor
 * @param port      I2C port
 * @param cfg       i2c bus config
 * @return          `ESP_OK` on success
 */
esp_err_t sht85_init_desc(sht85_t* dev, i2c_port_t port, i2c_config_t* cfg);

/**
 * @brief Free device descriptor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t sht85_free_desc(sht85_t* dev);

/**
 * @brief Initialize sensor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t sht85_init(sht85_t* dev);

/**
 * @brief Enable/disable heater
 *
 * @param dev       Device descriptor
 * @param enable    True to enable, false to disable
 * @return          `ESP_OK` on success
 */
esp_err_t sht85_set_heater(sht85_t* dev, bool enable);

/**
 * @brief High level measurement function
 *
 * For convenience this function comprises all three steps to perform
 * one measurement in only one function:
 *
 * 1. Starts a measurement in single shot mode with high reliability
 * 2. Waits using `vTaskDelay()` until measurement results are available
 * 3. Returns the results in kind of floating point sensor values
 *
 * This function is the easiest way to use the sensor. It is most suitable
 * for users that don't want to have the control on sensor details.
 *
 * Please note: The function delays the calling task up to 30 ms to wait for
 * the  the measurement results. This might lead to problems when the function
 * is called from a software timer callback function.
 *
 * @param dev         Device descriptor
 * @param temperature Temperature in degree Celsius
 * @param humidity    Humidity in percent
 * @return            `ESP_OK` on success
 */
esp_err_t sht85_measure(sht85_t* dev, float* temperature, float* humidity);

/**
 * @brief Get the duration of a measurement in RTOS ticks.
 *
 * The function returns the duration in RTOS ticks required by the sensor to
 * perform a measurement for the given repeatability. Once a measurement is
 * started with function *sht85_start_measurement* the user task can use this
 * duration in RTOS ticks directly to wait with function *vTaskDelay* until
 * the measurement results can be fetched.
 *
 * Please note: The duration only depends on repeatability level. Therefore,
 * it can be considered as constant for a repeatability.
 *
 * @param repeat    Repeatability, see type *sht85_repeat_t*
 * @return          Measurement duration given in RTOS ticks
 */
uint8_t sht85_get_measurement_duration(sht85_repeat_t repeat);

/**
 * @brief Start the measurement in single shot or periodic mode
 *
 * The function starts the measurement either in *single shot mode*
 * (exactly one measurement) or *periodic mode* (periodic measurements)
 * with given repeatability.
 *
 * In the *single shot mode*, this function has to be called for each
 * measurement. The measurement duration has to be waited every time
 * before the results can be fetched.
 *
 * In the *periodic mode*, this function has to be called only once. Also
 * the measurement duration has to be waited only once until the first
 * results are available. After this first measurement, the sensor then
 * automatically performs all subsequent measurements. The rate of periodic
 * measurements can be 10, 4, 2, 1 or 0.5 measurements per second (mps).
 *
 * Please note: Due to inaccuracies in timing of the sensor, the user task
 * should fetch the results at a lower rate. The rate of the periodic
 * measurements is defined by the parameter *mode*.
 *
 * @param dev       Device descriptor
 * @param mode      Measurement mode, see type *sht85_mode_t*
 * @param repeat    Repeatability, see type *sht85_repeat_t*
 * @return          `ESP_OK` on success
 */
esp_err_t sht85_start_measurement(sht85_t* dev, sht85_mode_t mode, sht85_repeat_t repeat);

/**
 * @brief Stop the periodic mode measurements
 *
 * The function stops the measurements  in *periodic mode*
 * (periodic measurements) and the sensor returns in *single shot mode*
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t sht85_stop_periodic_measurement(sht85_t* dev);

/**
 * @brief Read measurement results from sensor as raw data
 *
 * The function read measurement results from the sensor, checks the CRC
 * checksum and stores them in the byte array as following.
 *
 *      data[0] = Temperature MSB
 *      data[1] = Temperature LSB
 *      data[2] = Temperature CRC
 *      data[3] = Pressure MSB
 *      data[4] = Pressure LSB
 *      data[2] = Pressure CRC
 *
 * In case that there are no new data that can be read, the function fails.
 *
 * @param dev       Device descriptor
 * @param raw_data  Byte array in which raw data are stored
 * @return          `ESP_OK` on success
 */
esp_err_t sht85_get_raw_data(sht85_t* dev, sht85_raw_data_t raw_data);

/**
 * @brief Computes sensor values from raw data
 *
 * @param raw_data    Byte array that contains raw data
 * @param temperature Temperature in degree Celsius
 * @param humidity    Humidity in percent
 * @return            `ESP_OK` on success
 */
esp_err_t sht85_compute_values(sht85_raw_data_t raw_data, float* temperature, float* humidity);

/**
 * @brief Get measurement results in form of sensor values
 *
 * The function combines function *sht85_read_raw_data* and function
 * *sht85_compute_values* to get the measurement results.
 *
 * In case that there are no results that can be read, the function fails.
 *
 * @param dev         Device descriptor
 * @param temperature Temperature in degree Celsius
 * @param humidity    Humidity in percent
 * @return            `ESP_OK` on success
 */
esp_err_t sht85_get_results(sht85_t* dev, float* temperature, float* humidity);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __SHT85_H__ */
