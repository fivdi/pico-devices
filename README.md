# pico-devices

An I2C driver for the BME280 humidity, pressure and temperature sensor with
examples for RP2040-based boards like the Raspberry Pi Pico.

## Contents

- [Installation](#installation)
- [Examples](#examples)
- [Usage](#usage)
- [API](#api)

## Installation

Assuming the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
has been successfully installed and the `PICO_SDK_PATH` environment variable
has been appropriately set, this repository can be cloned and built with the
following commands:

```
git clone https://github.com/fivdi/pico-devices.git
cd pico-devices
mkdir build
cd build
cmake ..
make
```

## Examples

Examples can be found in the [examples](examples/bme280/) directory.

After a successful build, the list of example applications that have been
built and are available for loading onto a RP2040-based board can be found by
running the following command from the build directory:

```
find examples -type f -name "*.uf2"
```

## Usage

The steps to perform to use a BME280 sensor are:

- Create a `bme280_t`
- Call `bme280_init_struct` to initialize the `bme280_t` struct with default
values
- Optional: If needed, modify the sampling, filtering, standby time or mode
settings in the `bme280_t` struct
- Call `bme280_init` to initialize the BME280 sensor
- Create a `bme280_reading_t`
- Call `bme280_read` to copy the sensor data from the BME280 sensor to the
`bme280_reading_t`

Here is a complete minimalistic example that reads and prints BME280 sensor
data every 200 milliseconds. The BME280 sensor is assumed to be at address
`0x76` on the default I2C bus.

```c
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
```

## API

- [Typedefs](#typedefs)
  - [bme280_t](#typedef-bme280_t)
  - [bme280_reading_t](#typedef-bme280_reading_t)
- [Enumerations](#enumerations)
  - [bme280_error_t](#enumeration-bme280_error_t)
  - [bme280_mode_t](#enumeration-bme280_mode_t)
  - [bme280_sampling_t](#enumeration-bme280_sampling_t)
  - [bme280_filter_coeff_t](#enumeration-bme280_filter_coeff_t)
  - [bme280_standby_time_t](#enumeration-bme280_standby_time_t)
- [Functions](#functions)
  - [bme280_init_struct](#bme280_init_struct)
  - [bme280_init](#bme280_init)
  - [bme280_read](#bme280_read)
  - [bme280_forced_read](#bme280_forced_read)
  - [bme280_typical_measurement_time](#bme280_typical_measurement_time)
  - [bme280_maximum_measurement_time](#bme280_maximum_measurement_time)

### Typedefs

#### Typedef bme280_t

Used to control and read data from a BME280 sensor. Contains the following
fields:

- **bme280_sampling_t temperature_sampling** - Controls sampling of
temperature data
- **bme280_sampling_t pressure_sampling** - Controls sampling of pressure data
- **bme280_sampling_t humidity_sampling** - Controls sampling of humidity data
- **bme280_filter_coeff_t filter_coefficient** - Controls speed of response to
the sensor inputs.
- **bme280_standby_time_t standby_time** - Controls the inactive standby
period in normal mode
- **bme280_mode_t mode** - Controls the operating mode of the sensor

#### Typedef bme280_reading_t

A BME280 sensor reading. Contains the following fields:

- **float temperature** - Temperature in degrees Celsius
- **float pressure** - Pressure in pascal
- **float humidity** - Relative humidity in percent

### Enumerations

#### Enumeration bme280_error_t

Function return values.

- **BME280_OK** - Success
- **BME280_READ_ERROR** - I2C read error
- **BME280_WRITE_ERROR** - I2C write error
- **BME280_DEVICE_NOT_FOUND** - BME280 sensor not found at specified address
- **BME280_CALIBRATION_DATA_NOT_FOUND** - Can't read calibration data from
BME280 sensor
- **BME280_FORCED_READ_NOT_ALLOWED** - Forced read not allowed as the BME280
sensor is configured to run in normal mode

#### Enumeration bme280_mode_t

Controls the operating mode of the sensor.

- **BME280_MODE_SLEEP** - Sleep mode
- **BME280_MODE_FORCED** - Forced mode
- **BME280_MODE_NORMAL** - Normal mode

#### Enumeration bme280_sampling_t

Controls the sampling of sensor data.

- **BME280_SAMPLING_OFF** - Sampling off
- **BME280_SAMPLING_X1** - Sampling × 1
- **BME280_SAMPLING_X2** - Sampling × 2
- **BME280_SAMPLING_X4** - Sampling × 4
- **BME280_SAMPLING_X8** - Sampling × 8
- **BME280_SAMPLING_X16** - Sampling × 16

#### Enumeration bme280_filter_coeff_t

The filter is used to slow down the response to the sensor inputs.

- **BME280_FILTER_COEFF_OFF** - Filter off
- **BME280_FILTER_COEFF_2** - Filter coefficient = 2
- **BME280_FILTER_COEFF_4** - Filter coefficient = 4
- **BME280_FILTER_COEFF_8** - Filter coefficient = 8
- **BME280_FILTER_COEFF_16** - Filter coefficient = 16

#### Enumeration bme280_standby_time_t

Controls the inactive standby period in normal mode.

- **BME280_STANDBY_TIME_0_5_MS** - 0.5 milliseconds
- **BME280_STANDBY_TIME_62_5_MS** - 62.5 milliseconds
- **BME280_STANDBY_TIME_125_MS** - 125 milliseconds
- **BME280_STANDBY_TIME_250_MS** - 250 milliseconds
- **BME280_STANDBY_TIME_500_MS** - 500 milliseconds
- **BME280_STANDBY_TIME_1000_MS** - 1000 milliseconds
- **BME280_STANDBY_TIME_10_MS** - 10 milliseconds
- **BME280_STANDBY_TIME_20_MS** - 20 milliseconds

### Functions

#### bme280_init_struct

```c
void bme280_init_struct(
    bme280_t *bme280,
    void *i2c,
    uint8_t i2c_address,
    const system_callbacks_t *callbacks
);
```

Initializes the bme280_t struct specified by `bme280`.

The BME280 sensor is assumed to be on the I2C bus specified by `i2c` at the
I2C address specified by `i2c_address`.

On RP2040-based boards, the value specified by `callbacks` should be
`&pico_callbacks_blocking`.

The sampling defaults for humidity, pressure and temperature are
`BME280_SAMPLING_X1` and the default filter coefficient is
`BME280_FILTER_COEFF_OFF`. The default operating mode is `BME280_MODE_NORMAL`
and the default standby period is `BME280_STANDBY_TIME_0_5_MS` for
0.5 milliseconds. The defaults can be modified after calling
`bme280_init_struct` and before calling `bme280_init`.

#### bme280_init

```c
bme280_error_t bme280_init(bme280_t *bme280);
```

Initializes the BME280 sensor.

#### bme280_read

```c
bme280_error_t bme280_read(
    bme280_t *bme280,
    bme280_reading_t *reading
);
```

Reads the sensor data of the last sensor measurement from the BME280 to the
`bme280_reading_t` specified by `reading`.

#### bme280_forced_read

```c
bme280_error_t bme280_forced_read(
    bme280_t *bme280,
    bme280_reading_t *reading
);
```

Triggers a forced measurement, waits for the BME280 to complete the forced
measurement and reads the sensor data from the BME280 to the
`bme280_reading_t` specified by `reading`.

#### bme280_typical_measurement_time

```c
uint32_t bme280_typical_measurement_time(const bme280_t *bme280);
```

Returns the typical measurement time in milliseconds.

The typical measurement time depends on the selected values for humidity,
pressure and temperature sampling.

If `BME280_SAMPLING_X1` (the default) is used for humidity, pressure and
temperature sampling, the typical measurement time is 8 milliseconds.

If `BME280_SAMPLING_X16` is used for humidity, pressure and temperature
sampling, the typical measurement time is 98 milliseconds.

#### bme280_maximum_measurement_time

```c
uint32_t bme280_maximum_measurement_time(const bme280_t *bme280);
```

Returns the maximum measurement time in milliseconds.

The maximum measurement time depends on the selected values for humidity,
pressure and temperature oversampling.

If `BME280_SAMPLING_X1` (the default) is used for humidity, pressure and
temperature oversampling, the maximum measurement time is 10 milliseconds.

If `BME280_SAMPLING_X16` is used for humidity, pressure and temperature
oversampling, the maximum measurement time is 113 milliseconds.

