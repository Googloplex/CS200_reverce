cmake_minimum_required(VERSION 3.20)

# Clone firmware for Kpower HB200T (MCU LCP037BT32EU8 = LCM32F037, Cortex-M0).
# Built as an APPLICATION at 0x08001000: stock Kpower bootloader (0x08000000–0x08000FFF)
# stays in place and forwards interrupts via the vector table at 0x08001000.
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake" CACHE FILEPATH "")
project(hb200t_fw C ASM)

set(TARGET hb200t)
set(LD_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/ld/app.ld")

add_executable(${TARGET}.elf
    src/startup.c
    src/system.c
    src/board.c
    src/regs.c
    src/dxl.c
    src/sense.c
    src/motor.c
    src/main.c
)
target_include_directories(${TARGET}.elf PRIVATE vendor include src)
target_compile_definitions(${TARGET}.elf PRIVATE LCM32F037)
target_compile_options(${TARGET}.elf PRIVATE
    -mcpu=cortex-m0 -mthumb
    -Os -g3
    -std=gnu11 -Wall -Wextra -Wno-unused-parameter
    -ffunction-sections -fdata-sections -fno-common
    -fno-builtin-printf -Wdouble-promotion
)
target_link_options(${TARGET}.elf PRIVATE
    -mcpu=cortex-m0 -mthumb
    -nostartfiles --specs=nano.specs --specs=nosys.specs
    -Wl,--gc-sections -Wl,-Map=${TARGET}.map -Wl,--print-memory-usage
    -T ${LD_SCRIPT}
)
set_target_properties(${TARGET}.elf PROPERTIES LINK_DEPENDS ${LD_SCRIPT})

add_custom_command(TARGET ${TARGET}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET}.elf> ${TARGET}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${TARGET}.elf> ${TARGET}.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET}.elf>
    COMMENT "bin/hex + size"
)
