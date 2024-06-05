cmake_minimum_required(VERSION 3.20)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/toolchain.cmake)

project(stmproj)

# The executable target
add_executable(stmproj src/main.c
                    src/user_functions.c 
                    src/spi1.c
)

target_include_directories(stmproj 
PRIVATE ${CMAKE_SOURCE_DIR}/drivers/CMSIS/include
PRIVATE ${CMAKE_SOURCE_DIR}/drivers/CMSIS/device
PRIVATE ${CMAKE_SOURCE_DIR}/startup
PRIVATE ${CMAKE_SOURCE_DIR}/include
                            )

# Post-build command to generate elf file from executable
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
COMMAND arm-none-eabi-objcopy -O elf32-littlearm ${PROJECT_NAME} "stmproj.elf")

# Post-build command to generate bin file from executable
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
COMMAND arm-none-eabi-objcopy -O binary ${PROJECT_NAME} "stmproj.bin")

# Post-build command to display the size of the elf file.
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
COMMAND arm-none-eabi-size --format=berkeley "stmproj.elf")

# Option for flashing MCU with OpenOCD
option(FLASH "Flashing ${PROJECT_NAME}.bin with OpenOCD" OFF)

set(openocd_scripts_path /usr/local/share/openocd/scripts)


set(openocd_flags
    -f ${openocd_scripts_path}/interface/stlink.cfg 
    -f ${openocd_scripts_path}/target/stm32f4x.cfg
    -c "program ${PROJECT_NAME}.elf verify reset exit "
    # -c "program test_from_cubeide.elf verify reset exit"
    -l openocdlog
)

# -D FLASH=ON
if(FLASH)
message("run make to flash")
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND openocd ${openocd_flags} )

endif()         