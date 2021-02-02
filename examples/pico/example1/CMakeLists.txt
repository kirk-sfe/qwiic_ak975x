
# Setup the build environment
#

cmake_minimum_required(VERSION 3.12) 

include(pico_sdk_import.cmake)

project(ak975x_example1)

include(sparkfun_sdk_import.cmake)

# init the build environment
pico_sdk_init()

add_executable(ak975x_example1
   	ak975x_example1.cpp
)

# Pull in our (to be renamed) simple get you started dependencies
target_link_libraries(ak975x_example1 pico_stdlib sparkfun_qwiic_ak975x)

#enable USB output, disable uart output
pico_enable_stdio_usb(ak975x_example1 1)
pico_enable_stdio_uart(ak975x_example1 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(ak975x_example1)
