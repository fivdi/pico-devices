add_executable(forced_read
    main.c
)

target_link_libraries(forced_read
    pico_stdlib
    hardware_i2c
    sensor_bme280
    callbacks_blocking
)

pico_enable_stdio_usb(forced_read 0)
pico_enable_stdio_uart(forced_read 1)

pico_add_extra_outputs(forced_read)

