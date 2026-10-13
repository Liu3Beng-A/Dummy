#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.20)

# specify cross compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(Ctrl-Step-STM32-fw C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

#Uncomment for hardware floating point
#add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
#add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
#add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

add_compile_options(-mcpu=cortex-m3 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0 -fstack-usage)

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

include_directories(
        Core/Inc
        Drivers/STM32F1xx_HAL_Driver/Inc
        Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Device/ST/STM32F1xx/Include
        Drivers/CMSIS/Include
        Ctrl
        Ctrl/Sensor
        Ctrl/Signal
        Ctrl/LowLevel
        Ctrl/MotorControl
        UserApp
        Port
)

add_definitions(-DUSE_HAL_DRIVER -D__MICROLIB -DSTM32F1 -DSTM32F1xx -DSTM32F103xB)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
        "startup/*.*"
        "Drivers/*.*"
        "Core/*.*"
        "Ctrl/*.*"
        "UserApp/*.*"
        "Port/*.*"
        )

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F103CBTx_FLASH.ld)

# The protection ISR/context implementation uses short scoped IRQ guards and
# has no throwing operations. Do not pull the C++ exception unwinder into the
# 95KB application area just for those guards. Existing sources retain their
# original compiler options.
set_source_files_properties(Ctrl/Motor/stall_protection.cpp PROPERTIES
        COMPILE_OPTIONS "-fno-exceptions;-fno-rtti")

# Stream preparation/protocol run outside 20kHz. These nonthrowing sources use
# size optimization so the immutable 95KiB application/config boundary is kept.
# Preserve motor.cpp, motion_planner.cpp and protection hot-path optimization.
set_source_files_properties(
        Ctrl/Motor/stream_curve.cpp Ctrl/Motor/stream_runtime.cpp Ctrl/Motor/stream_controller.cpp
        UserApp/protocols/interface_can.cpp UserApp/main.cpp
        PROPERTIES COMPILE_OPTIONS "-Os;-fno-exceptions;-fno-rtti")

# Audited 2026-10-12: Ctrl/UserApp/Port contain no throw/catch, dynamic allocation,
# typeid or dynamic_cast. Their static C++ device objects cannot unwind. Removing
# the exception personality tables recovers application Flash without changing
# the Ofast optimization of the 20kHz control functions.
add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>" "$<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>")
set_source_files_properties(Core/Src/can.c Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_can.c
        PROPERTIES COMPILE_OPTIONS "-Os")

include("${CMAKE_SOURCE_DIR}/../cmake/firmware_artifacts.cmake")

# Define motor types and their corresponding MOTOR_TYPE values
set(MOTOR_TARGETS motor-35 motor-42 motor-57 motor-gripper)
set(MOTOR_TYPES 35 42 57 8)

list(LENGTH MOTOR_TARGETS NUM_TARGETS)
math(EXPR NUM_TARGETS "${NUM_TARGETS} - 1")

foreach(i RANGE 0 ${NUM_TARGETS})
    list(GET MOTOR_TARGETS ${i} TARGET_NAME)
    list(GET MOTOR_TYPES ${i} MOTOR_TYPE_VAL)
    

    # Create target-specific compile options
    set(TARGET_COMPILE_DEFS -DMOTOR_TYPE=${MOTOR_TYPE_VAL})

    add_executable(${TARGET_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})
    target_compile_definitions(${TARGET_NAME}.elf PRIVATE ${TARGET_COMPILE_DEFS})
    target_compile_definitions(${TARGET_NAME}.elf PRIVATE FW_TARGET_TYPE=${MOTOR_TYPE_VAL})
    set_property(TARGET ${TARGET_NAME}.elf APPEND PROPERTY LINK_DEPENDS "${LINKER_SCRIPT}")
    # Target-scope link options so each motor-N.elf gets its own
    # -T linker script, -mcpu, -Wl,-Map=... (was directory-scope add_link_options,
    # which accumulated across the foreach loop and made the linker overwrite
    # motor-35.map with motor-42's contents, etc.)
    target_link_options(${TARGET_NAME}.elf PRIVATE
        -mcpu=cortex-m3 -mthumb -mthumb-interwork
        -T ${LINKER_SCRIPT}
        -Wl,-gc-sections,--print-memory-usage)

    firmware_export_artifacts(${TARGET_NAME}.elf)
endforeach()

include("${CMAKE_SOURCE_DIR}/../cmake/firmware_identity.cmake")
firmware_configure_identity(TARGETS
        motor-35.elf motor-42.elf motor-57.elf motor-gripper.elf)
