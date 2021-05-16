#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "sensor/bme280.h"
#include "callbacks/blocking.h"

int main(void) {
    stdio_init_all();

    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    i2c_init(i2c_default, 400000);

    bme280_t bme280;

    bme280_init_struct(&bme280, i2c_default, 0x76, &pico_callbacks_blocking);
    bme280_init(&bme280);

    for (uint32_t i = 1; ; ++i) {
        bme280_reading_t r;
        bme280_read(&bme280, &r);

        printf(
            "\n%u Temperature: %.2f°C, Pressure: %.2f Pa, Humidity: %.2f%%",
            i, r.temperature, r.pressure, r.humidity
        );

        sleep_ms(200);
    }
}

