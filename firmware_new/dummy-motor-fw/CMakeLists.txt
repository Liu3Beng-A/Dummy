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
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

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

file(GLOB_RECURSE SOURCES
        "startup/*.*"
        "Drivers/*.*"
        "Core/*.*"
        "Ctrl/*.*"
        "UserApp/*.*"
        "Port/*.*"
        )

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F103CBTx_FLASH.ld)

add_link_options(-mcpu=cortex-m3 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})

# Define motor types and their corresponding MOTOR_TYPE values
set(MOTOR_TARGETS motor-35 motor-42 motor-57 motor-gripper)
set(MOTOR_TYPES 35 42 57 8)

list(LENGTH MOTOR_TARGETS NUM_TARGETS)
math(EXPR NUM_TARGETS "${NUM_TARGETS} - 1")

foreach(i RANGE 0 ${NUM_TARGETS})
    list(GET MOTOR_TARGETS ${i} TARGET_NAME)
    list(GET MOTOR_TYPES ${i} MOTOR_TYPE_VAL)
    
    set(TARGET_ELF ${PROJECT_BINARY_DIR}/${TARGET_NAME}.elf)
    set(TARGET_HEX ${PROJECT_BINARY_DIR}/${TARGET_NAME}.hex)
    set(TARGET_BIN ${PROJECT_BINARY_DIR}/${TARGET_NAME}.bin)
    set(TARGET_MAP ${PROJECT_BINARY_DIR}/${TARGET_NAME}.map)
    
    # Create target-specific compile options
    set(TARGET_COMPILE_DEFS -DMOTOR_TYPE=${MOTOR_TYPE_VAL})
    
    add_executable(${TARGET_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})
    target_compile_definitions(${TARGET_NAME}.elf PRIVATE ${TARGET_COMPILE_DEFS})
    
    add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${TARGET_MAP})
    
    add_custom_command(TARGET ${TARGET_NAME}.elf POST_BUILD
            COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${TARGET_NAME}.elf> ${TARGET_HEX}
            COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${TARGET_NAME}.elf> ${TARGET_BIN}
            COMMENT "Building ${TARGET_HEX}
Building ${TARGET_BIN}")
endforeach()
