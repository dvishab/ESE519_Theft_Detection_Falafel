cmake_minimum_required(VERSION 3.13) # CMAKE (heading)
include(pico_sdk_import.cmake) # CMAKE -> PICO SDK (include)

project(project C CXX ASM) # CMAKE ("project")
pico_sdk_init() # PICO SDK (initialization)

include(example_auto_set_url.cmake)


add_executable(project) # CMAKE (target)
pico_generate_pio_header(project ${CMAKE_CURRENT_LIST_DIR}/i2c.pio)

#ico_generate_pio_header(project  ${CMAKE_CURRENT_LIST_DIR}/generated) # PICO SDK (PIO)

target_sources(project PRIVATE app.c i2c.pio pio_i2c.c) # CMAKE -> C (compiler)
target_link_libraries(project PRIVATE pico_stdlib hardware_pio hardware_i2c) # CMAKE -> C (linker)

pico_enable_stdio_usb(project 1) # PICO SDK (USB)
pico_enable_stdio_uart(project 0) # PICO SDK (USB)

pico_add_extra_outputs(project) # PICO SDK (UF2)
