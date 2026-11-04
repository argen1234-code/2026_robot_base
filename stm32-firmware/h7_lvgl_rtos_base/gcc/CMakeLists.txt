cmake_minimum_required(VERSION 3.20)

set(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")
include(${CMAKE_CURRENT_LIST_DIR}/sources.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/includes.cmake)

project(stm32h743_firmware C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(firmware.elf ${FIRMWARE_SOURCES})

target_include_directories(firmware.elf PRIVATE ${FIRMWARE_INCLUDES})

target_compile_definitions(firmware.elf PRIVATE
  USE_HAL_DRIVER
  STM32H743xx
)

target_compile_options(firmware.elf PRIVATE
  -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=softfp
  -O1 -g3
  -ffunction-sections -fdata-sections -fno-common
  -Wall -Wno-unused-parameter -Wno-missing-field-initializers
)

target_link_libraries(firmware.elf PRIVATE
  ${PROJECT_ROOT}/Drivers/Hardware/NanoEdgeAI/libneai.a
)

target_link_options(firmware.elf PRIVATE
  -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=softfp
  -T${CMAKE_CURRENT_LIST_DIR}/STM32H743IITx_FLASH.ld
  -Wl,--gc-sections
  -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/firmware.map
  --specs=nano.specs
  --specs=nosys.specs
)

add_custom_command(TARGET firmware.elf POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:firmware.elf> ${CMAKE_CURRENT_BINARY_DIR}/firmware.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:firmware.elf> ${CMAKE_CURRENT_BINARY_DIR}/firmware.bin
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:firmware.elf>
)
