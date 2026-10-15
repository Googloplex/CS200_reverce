set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Путь к тулчейну: переменная окружения ARM_TOOLCHAIN_DIR, иначе PATH, иначе стандартная установка.
set(_tc_hints "$ENV{ARM_TOOLCHAIN_DIR}")
file(GLOB _tc_default "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/*/bin"
                      "C:/Program Files/Arm GNU Toolchain arm-none-eabi/*/bin")
list(APPEND _tc_hints ${_tc_default})

find_program(CMAKE_C_COMPILER   NAMES arm-none-eabi-gcc     HINTS ${_tc_hints} REQUIRED)
find_program(CMAKE_ASM_COMPILER NAMES arm-none-eabi-gcc     HINTS ${_tc_hints} REQUIRED)
find_program(CMAKE_OBJCOPY      NAMES arm-none-eabi-objcopy HINTS ${_tc_hints} REQUIRED)
find_program(CMAKE_OBJDUMP      NAMES arm-none-eabi-objdump HINTS ${_tc_hints} REQUIRED)
find_program(CMAKE_SIZE         NAMES arm-none-eabi-size    HINTS ${_tc_hints} REQUIRED)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
