#include <string.h>

#include "hardware/i2c.h"

#include "callbacks/blocking.h"

static system_callback_error_t pico_sdk_i2c_read(
    void *i2c,
    uint8_t i2c_addr,
    uint8_t reg,
    uint8_t *rx_data,
    uint8_t len
) {
    if (i2c_write_blocking(i2c, i2c_addr, &reg, 1, true) != 1) {
        return SYSTEM_CALLBACK_WRITE_ERROR;
    }

    if (i2c_read_blocking(i2c, i2c_addr, rx_data, len, false) != len) {
        return SYSTEM_CALLBACK_READ_ERROR;
    }

    return SYSTEM_CALLBACK_OK;
}

static system_callback_error_t pico_sdk_i2c_write(
    void *i2c,
    uint8_t i2c_addr,
    uint8_t reg,
    const uint8_t *tx_data,
    uint8_t len
) {
    uint8_t tx_len = len + 1;
    uint8_t tx_buf[tx_len];
    tx_buf[0]  = reg;
    memcpy(&tx_buf[1], tx_data, len);

    if (i2c_write_blocking(i2c, i2c_addr, tx_buf, tx_len, false) != tx_len) {
        return SYSTEM_CALLBACK_WRITE_ERROR;
    }

    return SYSTEM_CALLBACK_OK;
}

static void pico_sdk_delay_ms(uint32_t milliseconds) {
    sleep_ms(milliseconds);
}

const system_callbacks_t pico_callbacks_blocking = {
    .i2c_read = pico_sdk_i2c_read,
    .i2c_write = pico_sdk_i2c_write,
    .delay_ms = pico_sdk_delay_ms
};

