cmake_minimum_required(VERSION 3.24)

include(lib/pico-sdk/pico_sdk_init.cmake)

project(easy-do-tracker CXX C ASM)

pico_sdk_init()

add_executable(easy-do-tracker
        main.cpp
        lib/i2cdevlib/RP2040/I2Cdev/I2Cdev.cpp
        lib/i2cdevlib/RP2040/MPU6050/MPU6050.cpp
)

target_include_directories(easy-do-tracker PUBLIC
        lib/i2cdevlib/RP2040/I2Cdev
        lib/i2cdevlib/RP2040/MPU6050
)

target_link_libraries(easy-do-tracker pico_stdlib hardware_i2c)

pico_enable_stdio_usb(easy-do-tracker 1)

pico_add_extra_outputs(easy-do-tracker)