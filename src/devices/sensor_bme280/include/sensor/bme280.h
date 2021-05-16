#ifndef _BME280_H_
#define _BME280_H_

#include <stdint.h>

#include "system/callbacks.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float temperature;
    float pressure;
    float humidity;
} bme280_reading_t;

typedef enum {
    BME280_OK = 0,
    BME280_READ_ERROR = SYSTEM_CALLBACK_READ_ERROR,
    BME280_WRITE_ERROR = SYSTEM_CALLBACK_WRITE_ERROR,
    BME280_DEVICE_NOT_FOUND = -100,
    BME280_CALIBRATION_DATA_NOT_FOUND = -101,
    BME280_FORCED_READ_NOT_ALLOWED = -102
} bme280_error_t;

typedef enum {
    BME280_MODE_SLEEP = 0,
    BME280_MODE_FORCED = 1,
    BME280_MODE_NORMAL = 3
} bme280_mode_t;

typedef enum {
    BME280_SAMPLING_OFF = 0,
    BME280_SAMPLING_X1 = 1,
    BME280_SAMPLING_X2 = 2,
    BME280_SAMPLING_X4 = 3,
    BME280_SAMPLING_X8 = 4,
    BME280_SAMPLING_X16 = 5
} bme280_sampling_t;

typedef enum {
    BME280_FILTER_COEFF_OFF = 0,
    BME280_FILTER_COEFF_2 = 1,
    BME280_FILTER_COEFF_4 = 2,
    BME280_FILTER_COEFF_8 = 3,
    BME280_FILTER_COEFF_16 = 4
} bme280_filter_coeff_t;

typedef enum {
    BME280_STANDBY_TIME_0_5_MS = 0,
    BME280_STANDBY_TIME_62_5_MS = 1,
    BME280_STANDBY_TIME_125_MS = 2,
    BME280_STANDBY_TIME_250_MS = 3,
    BME280_STANDBY_TIME_500_MS = 4,
    BME280_STANDBY_TIME_1000_MS = 5,
    BME280_STANDBY_TIME_10_MS = 6,
    BME280_STANDBY_TIME_20_MS = 7
} bme280_standby_time_t;

typedef struct {
    uint16_t t1;
    int16_t t2;
    int16_t t3;

    uint16_t p1;
    int16_t p2;
    int16_t p3;
    int16_t p4;
    int16_t p5;
    int16_t p6;
    int16_t p7;
    int16_t p8;
    int16_t p9;

    uint8_t h1;
    int16_t h2;
    uint8_t h3;
    int16_t h4;
    int16_t h5;
    int8_t h6;
} bme280_calibration_data_t;

typedef struct {
    void *i2c;
    uint8_t i2c_address;
    bme280_sampling_t temperature_sampling;
    bme280_sampling_t pressure_sampling;
    bme280_sampling_t humidity_sampling;
    bme280_filter_coeff_t filter_coefficient;
    bme280_standby_time_t standby_time;
    bme280_mode_t mode;

    const system_callbacks_t *callbacks;

    bme280_calibration_data_t calibration_data;
} bme280_t;

void bme280_init_struct(
    bme280_t *bme280,
    void *i2c,
    uint8_t i2c_address,
    const system_callbacks_t *callbacks
);

bme280_error_t bme280_init(bme280_t *bme280);

bme280_error_t bme280_read(bme280_t *bme280, bme280_reading_t *reading);

bme280_error_t bme280_forced_read(
    bme280_t *bme280, bme280_reading_t *reading
);

uint32_t bme280_typical_measurement_time(const bme280_t *bme280);

uint32_t bme280_maximum_measurement_time(const bme280_t *bme280);

#ifdef __cplusplus
}
#endif

#endif // _BME280_H_

