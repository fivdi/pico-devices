add_library(sensor_bme280 INTERFACE)

target_include_directories(sensor_bme280 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_sources(sensor_bme280 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/bme280.c
)

target_link_libraries(sensor_bme280 INTERFACE
    system_callbacks_headers
)

