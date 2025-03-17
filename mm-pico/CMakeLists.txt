cmake_minimum_required(VERSION 3.30)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

#set(PICO_SDK_FETCH_FROM_GIT on)

include(pico_sdk_import.cmake)

project(mm_pico)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(mm_pico
        main.cpp
        hub75.cpp
)

pico_generate_pio_header(mm_pico ${CMAKE_CURRENT_LIST_DIR}/hub75.pio)

pico_set_program_version(mm_pico "0.1")

pico_enable_stdio_uart(mm_pico 1)
pico_enable_stdio_usb(mm_pico 1)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(mm_pico pico_stdlib)

target_link_libraries(mm_pico
        pico_multicore
        hardware_i2c
        hardware_dma
        hardware_pio
        hardware_interp
        hardware_timer
)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(mm_pico)