cmake_minimum_required(VERSION 3.15)

project(ADC_Sampling)
set(EXECUTABLE ${PROJECT_NAME}.elf)

enable_language(C ASM)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_LIBOPENCM3_DIR ${CMAKE_SOURCE_DIR}/../libopencm3)
# set(CMAKE_LIBOPENCM3_INCLUDE_DIR ${CMAKE_LIBOPENCM3_DIR}/include/libopencm3/stm32)

# Specify the device type
add_definitions(-DSTM32L1)
set(CMAKE_TARGET_DEVICE stm32/l1)

# Use toolchain
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/arm-gcc-toolchain.cmake)

# C flags
set(CMAKE_C_FLAGS "-Wall -Wextra -O2 -ffreestanding -nostartfiles -mthumb -mcpu=cortex-m3")

# Linker flags
#set(CMAKE_EXE_LINKER_FLAGS "-T ${CMAKE_SOURCE_DIR}/stm32l152re.ld -nostdlib -Wl,--gc-sections")


# ExternalProject: clone and build libopencm3
include(ExternalProject)
ExternalProject_Add(libopencm3_project
   SOURCE_DIR ${CMAKE_LIBOPENCM3_DIR}
   CONFIGURE_COMMAND ""
   BUILD_COMMAND make TARGETS=${CMAKE_TARGET_DEVICE}
   BUILD_IN_SOURCE 1
   INSTALL_COMMAND ""
)

# Add include dir
include_directories(${CMAKE_LIBOPENCM3_INCLUDE_DIR})

# Add lib dir for linking
# link_directories(${CMAKE_LIBOPENCM3_DIR}/lib)


# Executable target
add_executable(${EXECUTABLE} 
        ${CMAKE_SOURCE_DIR}/src/main.c
        #${CMAKE_SOURCE_DIR}/src/startup_stm32l1xx.c
        ${CMAKE_SOURCE_DIR}/drivers/ADC_drv.c
        ${CMAKE_SOURCE_DIR}/drivers/led_driver.c
        ${CMAKE_SOURCE_DIR}/drivers/systick.c
        ${CMAKE_SOURCE_DIR}/drivers/timer_drv.c
        ${CMAKE_SOURCE_DIR}/drivers/uart_drv.c
)

# Add header path to target
target_include_directories(${EXECUTABLE}  PRIVATE 
    ${CMAKE_SOURCE_DIR}/drivers 
    ${CMAKE_LIBOPENCM3_DIR}/include
)

# Ensure libopencm3 is built first
add_dependencies(${EXECUTABLE} libopencm3_project)


# Link against pre-built libopencm3.a
target_link_libraries(${EXECUTABLE}
    ${CMAKE_LIBOPENCM3_DIR}/lib/libopencm3_stm32l1.a
)

# Compiler options
target_compile_options(${EXECUTABLE} PRIVATE
        -mcpu=cortex-m3
        -mthumb

        -fdata-sections
        -ffunction-sections

        -Wall
        -O0
        -g3
)

# Linker options
target_link_options(${EXECUTABLE} PRIVATE
        -T ${CMAKE_SOURCE_DIR}/stm32l152re.ld
        -mcpu=cortex-m3
        -mthumb
        -specs=nano.specs
        -specs=nosys.specs
        -lc
        -lgcc
        -lm
        -lnosys
        -Wl,-Map=${PROJECT_NAME}.map,--cref
        -Wl,--gc-sections
        -Xlinker -print-memory-usage -Xlinker
)


# Optional: Print executable size as part of the post build process
add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND ${CMAKE_SIZE_UTIL} ${EXECUTABLE}
)

# Optional: generate .bin and .hex
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${EXECUTABLE} ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${EXECUTABLE} ${PROJECT_NAME}.hex
)

# Optional: Flash executable to target
add_custom_target(flash
    COMMAND st-flash write ${PROJECT_NAME}.bin 0x08000000
    DEPENDS ${PROJECT_NAME}
)