cmake_minimum_required(VERSION 3.13)

# Inclui o SDK do Pico
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(extrusora_controller C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Inicializa o SDK do Pico
pico_sdk_init()

# Adiciona o executável
add_executable(extrusora_controller
    extrusora_controller.cpp
)

# Inclui bibliotecas do Pico
target_link_libraries(extrusora_controller
    pico_stdlib
    hardware_i2c
    hardware_adc
    hardware_pwm
    hardware_gpio
    pico_time
)

# Habilita saída USB e UART
pico_enable_stdio_usb(extrusora_controller 1)
pico_enable_stdio_uart(extrusora_controller 1)

# Cria arquivos de saída adicionais
pico_add_extra_outputs(extrusora_controller)
