cmake_minimum_required(VERSION 3.13)

include(/usr/local/raspberrypi-pico-sdk/pico_sdk_init.cmake)

project(si5351)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(si5351
        si5351.cpp
	libsi5351.cpp
        )

# pull in common dependencies
target_link_libraries(si5351
	pico_stdlib
	pico_malloc
	hardware_i2c
	)

add_compile_options(
	-Wall
	-Wno-format
	-Wno-unused-function
	-Wno-maybe-uninitialized
	)

pico_enable_stdio_usb(si5351 1)
pico_enable_stdio_uart(si5351 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(si5351)
