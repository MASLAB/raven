cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_HAL_DRIVER 
	STM32G431xx
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../Core/Inc
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Inc
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/CMSIS/Device/ST/STM32G4xx/Include
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/CMSIS/Include
)

target_sources(stm32cubemx INTERFACE
    ../../Core/Src/main.c
    ../../Core/Src/stm32g4xx_it.c
    ../../Core/Src/stm32g4xx_hal_msp.c
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_crc.c
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_crc_ex.c
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c
    /mnt/data/Documents/MASLAB/2025/ubuntu-raven/firmware/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c
    ../../Core/Src/system_stm32g4xx.c
    ../../Core/Src/sysmem.c
    ../../Core/Src/syscalls.c
    ../../startup_stm32g431xx.s
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


