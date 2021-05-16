#include "sensor/bme280.h"


typedef enum {
    REG_CALIB_DATA1 = 0x88,
    REG_CHIP_ID = 0xd0,
    REG_RESET = 0xe0,
    REG_CALIB_DATA2 = 0xe1,
    REG_CTRL_HUM = 0xf2,
    REG_STATUS = 0xf3,
    REG_CTRL_MEAS = 0xf4,
    REG_CONFIG = 0xf5,
    REG_DATA = 0xf7
} bme280_reg_t;


typedef enum {
    LEN_CALIB_DATA1 = 26,
    LEN_CALIB_DATA2 = 7,
    LEN_DATA = 8
} bme280_block_lengths_t;


typedef enum {
    CHIP_ID = 0x60,
    SOFT_RESET_COMMAND = 0xb6
} bme280_reg_constants_t;


typedef enum {
    STATUS_IMAGE_UPDATE_BIT = 0x01,
    STATUS_MEASURING_BIT = 0x08
} bme280_status_reg_t;


typedef enum {
    CTRL_HUM_OSRS_H_MASK = 0x07,
    CTRL_HUM_OSRS_H_POS = 0x00
} bme280_ctrl_hum_reg_t;


typedef enum {
    CTRL_MEAS_MODE_POS = 0x00,
    CTRL_MEAS_MODE_MASK = 0x03,
    CTRL_MEAS_OSRS_P_POS = 0x02,
    CTRL_MEAS_OSRS_T_POS = 0x05
} bme280_ctrl_meas_reg_t;


typedef enum {
    CONFIG_FILTER_MASK = 0x1c,
    CONFIG_FILTER_POS = 2,
    CONFIG_STANDBY_MASK = 0xe0,
    CONFIG_STANDBY_POS = 5
} bme280_config_reg_t;


typedef struct {
    uint32_t temperature;
    uint32_t pressure;
    uint16_t humidity;
} bme280_raw_data_t;


static bme280_error_t read_byte(
    bme280_t *bme280, uint8_t reg, uint8_t *byte
) {
    return bme280->callbacks->i2c_read(
        bme280->i2c, bme280->i2c_address, reg, byte, 1
    );
}


static bme280_error_t write_byte(
    bme280_t *bme280, uint8_t reg, uint8_t byte
) {
    return bme280->callbacks->i2c_write(
        bme280->i2c, bme280->i2c_address, reg, &byte, 1
    );
}


static bme280_error_t read_block(
    bme280_t *bme280, uint8_t reg, uint8_t *rx_data, uint8_t len
) {
    return bme280->callbacks->i2c_read(
        bme280->i2c, bme280->i2c_address, reg, rx_data, len
    );
}


static bme280_error_t check_chip_id(bme280_t *bme280) {
    for (uint8_t i = 1; i <= 5; ++i) {
        uint8_t chip_id;

        bme280_error_t rval = read_byte(bme280, REG_CHIP_ID, &chip_id);
        if (rval == BME280_OK && chip_id == CHIP_ID) {
            return BME280_OK;
        }

        bme280->callbacks->delay_ms(1);
    }

    return BME280_DEVICE_NOT_FOUND;
}


static bme280_error_t soft_reset(bme280_t *bme280) {
    return write_byte(bme280, REG_RESET, SOFT_RESET_COMMAND);
}


static bme280_error_t wait_for_calibration_data(bme280_t *bme280) {
    for (uint8_t i = 1; i <= 5; ++i) {
        bme280->callbacks->delay_ms(2);

        uint8_t status_reg;

        bme280_error_t rval = read_byte(bme280, REG_STATUS, &status_reg);
        if (rval != BME280_OK) {
            return rval;
        }

        if ((status_reg & STATUS_IMAGE_UPDATE_BIT) == 0) {
            return BME280_OK;
        }
    }

    return BME280_CALIBRATION_DATA_NOT_FOUND;
}


static uint16_t read_uint16_le(const uint8_t *data, uint8_t offset) {
    return (data[offset + 1] << 8) | data[offset];
}


static int16_t read_int16_le(const uint8_t *data, uint8_t offset) {
    return (data[offset + 1] << 8) | data[offset];
}


static bme280_error_t read_calibration_data(bme280_t *bme280) {
    bme280_calibration_data_t *cd = &bme280->calibration_data;
    uint8_t raw_cd[
        LEN_CALIB_DATA1 > LEN_CALIB_DATA2 ?
        LEN_CALIB_DATA1 : LEN_CALIB_DATA2
    ];

    bme280_error_t rval = read_block(
        bme280, REG_CALIB_DATA1, raw_cd, LEN_CALIB_DATA1
    );

    if (rval == BME280_OK) {
        cd->t1 = read_uint16_le(raw_cd, 0);
        cd->t2 = read_int16_le(raw_cd, 2);
        cd->t3 = read_int16_le(raw_cd, 4);

        cd->p1 = read_uint16_le(raw_cd, 6);
        cd->p2 = read_int16_le(raw_cd, 8);
        cd->p3 = read_int16_le(raw_cd, 10);
        cd->p4 = read_int16_le(raw_cd, 12);
        cd->p5 = read_int16_le(raw_cd, 14);
        cd->p6 = read_int16_le(raw_cd, 16);
        cd->p7 = read_int16_le(raw_cd, 18);
        cd->p8 = read_int16_le(raw_cd, 20);
        cd->p9 = read_int16_le(raw_cd, 22);

        cd->h1 = raw_cd[25];
    }

    if (rval == BME280_OK) {
        rval = read_block(
          bme280, REG_CALIB_DATA2, raw_cd, LEN_CALIB_DATA2
        );
    }

    if (rval == BME280_OK) {
        cd->h2 = read_int16_le(raw_cd, 0);
        cd->h3 = raw_cd[2];
        cd->h4 = ((int8_t) raw_cd[3] << 4) | (raw_cd[4] & 0x0f);
        cd->h5 = ((int8_t) raw_cd[5] << 4) | (raw_cd[4] >> 4);
        cd->h6 = (int8_t) raw_cd[6];
    }

    return rval;
}


static bme280_error_t configure_settings(bme280_t *bme280) {
    uint8_t config_reg;
    bme280_error_t rval = read_byte(bme280, REG_CONFIG, &config_reg);

    if (rval == BME280_OK) {
        config_reg =
            (config_reg & ~(CONFIG_STANDBY_MASK | CONFIG_FILTER_MASK)) |
            (bme280->standby_time << CONFIG_STANDBY_POS) |
            (bme280->filter_coefficient << CONFIG_FILTER_POS);
        rval = write_byte(bme280, REG_CONFIG, config_reg);
    }

    if (rval == BME280_OK) {
        uint8_t ctrl_hum_reg;
        rval = read_byte(bme280, REG_CTRL_HUM, &ctrl_hum_reg);

        if (rval == BME280_OK) {
            ctrl_hum_reg =
                (ctrl_hum_reg & ~CTRL_HUM_OSRS_H_MASK) |
                (bme280->humidity_sampling << CTRL_HUM_OSRS_H_POS);
            rval = write_byte(bme280, REG_CTRL_HUM, ctrl_hum_reg);
        }
    }

    if (rval == BME280_OK) {
        uint8_t mode = bme280->mode == BME280_MODE_FORCED ?
            BME280_MODE_SLEEP : BME280_MODE_NORMAL;
        uint8_t ctrl_meas_reg =
            (bme280->temperature_sampling << CTRL_MEAS_OSRS_T_POS) |
            (bme280->pressure_sampling << CTRL_MEAS_OSRS_P_POS) |
            (mode << CTRL_MEAS_MODE_POS);
        rval = write_byte(bme280, REG_CTRL_MEAS, ctrl_meas_reg);
    }

    return rval;
}


static bme280_error_t read_raw_data(
    bme280_t *bme280, bme280_raw_data_t *raw_data
) {
    uint8_t data[LEN_DATA];

    bme280_error_t rval = read_block(bme280, REG_DATA, data, LEN_DATA);

    if (rval == BME280_OK) {
        raw_data->pressure = data[0] << 12 | data[1] << 4 | data[2] >> 4;
        raw_data->temperature = data[3] << 12 | data[4] << 4 | data[5] >> 4;
        raw_data->humidity = data[6] << 8 | data[7];
    }

    return rval;
}


static float compensate_temperature(
    const bme280_calibration_data_t *c, uint32_t rawt
) {
    return (((float) rawt / 16384.0f - (float) c->t1 / 1024.0f) * (float) c->t2) +
            (((float) rawt / 131072.0f - (float) c->t1 / 8192.0f) *
             ((float) rawt / 131072.0f - (float) c->t1 / 8192.0f) * (float) c->t3);
}


static float compensate_pressure(
    const bme280_calibration_data_t *c, uint32_t rawp, float t_fine
) {
    float var1 = t_fine / 2.0f - 64000.0f;
    float var2 = var1 * var1 * (float) c->p6 / 32768.0f;
    var2 = var2 + var1 * (float) c->p5 * 2.0f;
    var2 = (var2 / 4.0f) + ((float) c->p4 * 65536.0f);
    var1 = ((float) c->p3 * var1 * var1 / 524288.0f + (float) c->p2 * var1) / 524288.0f;
    var1 = (1.0f + var1 / 32768.0f) * (float) c->p1;

    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }

    float p = 1048576.0f - (float) rawp;
    p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
    var1 = (float) c->p9 * p * p / 2147483648.0f;
    var2 = p * (float) c->p8 / 32768.0f;
    p = p + (var1 + var2 + (float) c->p7) / 16.0f;

    return p;
}


static float compensate_humidity(
    const bme280_calibration_data_t *c, uint32_t rawh, float t_fine
) {
    float h = t_fine - 76800.0f;

    h = ((float) rawh - ((float) c->h4 * 64.0f + (float) c->h5 / 16384.0f * h)) *
        ((float) c->h2 / 65536.0f * (1.0f + (float) c->h6 / 67108864.0f * h * (1.0f + (float) c->h3 / 67108864.0f * h)));

    h = h * (1.0f - (float) c->h1 * h / 524288.0f);

    return h;
}


static bme280_error_t trigger_forced_measurement(bme280_t *bme280) {
    uint8_t ctrl_meas_reg;
    bme280_error_t rval = read_byte(bme280, REG_CTRL_MEAS, &ctrl_meas_reg);

    if (rval == BME280_OK) {
        uint8_t mode =
            (ctrl_meas_reg & CTRL_MEAS_MODE_MASK) >> CTRL_MEAS_MODE_POS;

        if (mode != BME280_MODE_SLEEP) {
            rval = BME280_FORCED_READ_NOT_ALLOWED;
        }
    }

    if (rval == BME280_OK) {
        ctrl_meas_reg = (ctrl_meas_reg & ~CTRL_MEAS_MODE_MASK) | 
            (BME280_MODE_FORCED << CTRL_MEAS_MODE_POS);

        rval = write_byte(bme280, REG_CTRL_MEAS, ctrl_meas_reg);
    }

    return rval;
}


void bme280_init_struct(
    bme280_t *bme280,
    void *i2c,
    uint8_t i2c_address,
    const system_callbacks_t *callbacks
) {
    bme280->i2c = i2c;
    bme280->i2c_address = 0x76;

    bme280->callbacks = callbacks;

    bme280->humidity_sampling = BME280_SAMPLING_X1;
    bme280->pressure_sampling = BME280_SAMPLING_X1;
    bme280->temperature_sampling = BME280_SAMPLING_X1;
    bme280->filter_coefficient = BME280_FILTER_COEFF_OFF;
    bme280->standby_time = BME280_STANDBY_TIME_0_5_MS;
    bme280->mode = BME280_MODE_NORMAL;
}


bme280_error_t bme280_init(bme280_t *bme280) {
    bme280_error_t rval = check_chip_id(bme280);

    if (rval == BME280_OK) {
        rval = soft_reset(bme280);
    }

    if (rval == BME280_OK) {
        rval = wait_for_calibration_data(bme280);
    }

    if (rval == BME280_OK) {
        rval = read_calibration_data(bme280);
    }

    if (rval == BME280_OK) {
        rval = configure_settings(bme280);
    }

    // If normal mode was configured, wait for first reading to complete so
    // that it's available for caller immediately after bme280_init returns.
    if (rval == BME280_OK && bme280->mode == BME280_MODE_NORMAL) {
        bme280->callbacks->delay_ms(bme280_maximum_measurement_time(bme280));
    }

    return rval;
}


bme280_error_t bme280_read(bme280_t *bme280, bme280_reading_t *reading) {
    bme280_raw_data_t raw_data;

    bme280_error_t rval = read_raw_data(bme280, &raw_data);

    if (rval == BME280_OK) {
        // Temperature
        const float t_fine = compensate_temperature(
            &bme280->calibration_data, raw_data.temperature
        );
        float temperature = t_fine / 5120.0f;

        if (temperature < -40.0f) {
            temperature = -40.0f;
        } else if (temperature > 85.0f) {
            temperature = 85.0f;
        }

        reading->temperature = temperature;

        // Pressure
        float pressure = reading->pressure = compensate_pressure(
            &bme280->calibration_data, raw_data.pressure, t_fine
        );

        if (pressure < 30000.0f) {
            pressure = 30000.0f;
        } else if (pressure > 110000.0f) {
            pressure = 110000.0f;
        }

        reading->pressure = pressure;

        // Humidity
        float humidity = compensate_humidity(
            &bme280->calibration_data, raw_data.humidity, t_fine
        );

        if (humidity < 0.0f) {
            humidity = 0.0f;
        } else if (humidity > 100.0f) {
            humidity = 100.0f;
        }

        reading->humidity = humidity;
    }

    return rval;
}


bme280_error_t bme280_forced_read(
    bme280_t *bme280, bme280_reading_t *reading
) {
    bme280_error_t rval = trigger_forced_measurement(bme280);

    if (rval == BME280_OK) {
        bme280->callbacks->delay_ms(bme280_maximum_measurement_time(bme280));
        rval = bme280_read(bme280, reading);
    }

    return rval;
}


static const uint8_t samples[] = {0, 1, 2, 4, 8, 16};


uint32_t bme280_typical_measurement_time(const bme280_t *bme280) {
    const uint8_t temperature_samples = samples[bme280->temperature_sampling];
    const uint8_t pressure_samples = samples[bme280->pressure_sampling];
    const uint8_t humidity_samples = samples[bme280->humidity_sampling];

    const uint32_t typical_meas_time_us = 1000 +
        (temperature_samples == 0 ? 0 : 2000 * temperature_samples) +
        (pressure_samples == 0 ? 0 : 2000 * pressure_samples + 500) +
        (humidity_samples == 0 ? 0 : 2000 * humidity_samples + 500);

    const uint32_t typical_meas_time_ms =
        (typical_meas_time_us + 1000 - 1) / 1000;

    return typical_meas_time_ms;
}


uint32_t bme280_maximum_measurement_time(const bme280_t *bme280) {
    const uint8_t temperature_samples = samples[bme280->temperature_sampling];
    const uint8_t pressure_samples = samples[bme280->pressure_sampling];
    const uint8_t humidity_samples = samples[bme280->humidity_sampling];

    const uint32_t max_meas_time_us = 1250 +
        (temperature_samples == 0 ? 0 : 2300 * temperature_samples) +
        (pressure_samples == 0 ? 0 : 2300 * pressure_samples + 575) +
        (humidity_samples == 0 ? 0 : 2300 * humidity_samples + 575);

    const uint32_t max_meas_time_ms = (max_meas_time_us + 1000 - 1) / 1000;

    return max_meas_time_ms;
}

