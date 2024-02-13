#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.27)

# specify cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(buzzcam C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

#Uncomment for hardware floating point
add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

add_compile_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

# Enable assembler files preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

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

include_directories(FATFS/Target /Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.11.3.rel1.macos64_1.1.100.202310310803 FATFS/App Core/Inc STM32_WPAN/App Drivers/STM32WBxx_HAL_Driver/Inc Drivers/STM32WBxx_HAL_Driver/Inc/Legacy Utilities/lpm/tiny_lpm Middlewares/Third_Party/FatFs/src Middlewares/ST/STM32_WPAN Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci Middlewares/ST/STM32_WPAN/utilities Middlewares/ST/STM32_WPAN/ble/core Middlewares/ST/STM32_WPAN/ble/core/auto Middlewares/ST/STM32_WPAN/ble/core/template Drivers/CMSIS/Device/ST/STM32WBxx/Include Middlewares/ST/STM32_WPAN/ble Drivers/CMSIS/Include Middlewares/Third_Party/FreeRTOS/Source/include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F protobuf Middlewares/STM32_AdafruitSensor Middlewares/bsec_2_2_0_0/algo/normal_version/inc Middlewares/STM32_BME680 Middlewares/bsec_2_2_0_0 USB_Device/App USB_Device/Target Middlewares/ST/STM32_USB_Device_Library/Core/Inc Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc Middlewares/ST/STM32_WPAN/thread/openthread/core/openthread_api Middlewares/ST/STM32_WPAN/thread/openthread/stack/include/openthread Middlewares/ST/STM32_WPAN/thread/openthread/stack/include Middlewares/ST/STM32_WPAN/thread/openthread/stack/src/core Middlewares/ST/STM32_WPAN/thread/openthread/stack/third_party/mbedtls/repo/include Middlewares/ST/STM32_WPAN/thread/openthread/stack/include/openthread/platform tflac)

add_definitions(-DCFG_FULL_LOW_POWER=1 -DDYNAMIC_MODE -DTHREAD_WB -DOPENTHREAD_CONFIG_FILE=<openthread_api_config_ftd.h> -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB5Mxx)

file(GLOB_RECURSE SOURCES "Core/*.*" "Drivers/*.*" "FATFS/*.*" "Middlewares/*.*" "Middlewares/bsec_2_2_0_0/algo/*.*" "Middlewares/bsec_2_2_0_0/config/*.*" "STM32_WPAN/*.*" "USB_Device/*.*" "Utilities/*.*" "protobuf/*.*")

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32WB5MMGHX_FLASH.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
