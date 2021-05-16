add_library(callbacks_blocking INTERFACE)

target_include_directories(callbacks_blocking INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_sources(callbacks_blocking INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/blocking.c
)

target_link_libraries(callbacks_blocking INTERFACE
    hardware_i2c
    system_callbacks_headers
)

