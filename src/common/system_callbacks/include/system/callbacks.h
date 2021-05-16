#ifndef _SYSTEM_CALLBACKS_H
#define _SYSTEM_CALLBACKS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SYSTEM_CALLBACK_OK = 0,
    SYSTEM_CALLBACK_READ_ERROR = -1,
    SYSTEM_CALLBACK_WRITE_ERROR = -2,
} system_callback_error_t;

typedef system_callback_error_t (*system_callback_i2c_read_t)(
    void *i2c,
    uint8_t i2c_addr,
    uint8_t reg,
    uint8_t *rx_data,
    uint8_t len
);

typedef system_callback_error_t (*system_callback_i2c_write_t)(
    void *i2c,
    uint8_t i2c_addr,
    uint8_t reg,
    const uint8_t *tx_data,
    uint8_t len
);

typedef void (*system_callback_delay_ms_t)(uint32_t milliseconds);

typedef struct {
    system_callback_i2c_read_t i2c_read;
    system_callback_i2c_write_t i2c_write;
    system_callback_delay_ms_t delay_ms;
} system_callbacks_t;

#ifdef __cplusplus
}
#endif

#endif // _SYSTEM_CALLBACKS_H

