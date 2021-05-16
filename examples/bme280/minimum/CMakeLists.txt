add_executable(minimum
    main.c
)

target_link_libraries(minimum
    pico_stdlib
    hardware_i2c
    sensor_bme280
    callbacks_blocking
)

pico_enable_stdio_usb(minimum 0)
pico_enable_stdio_uart(minimum 1)

pico_add_extra_outputs(minimum)

