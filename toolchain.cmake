set(CMAKE_SYSTEM_NAME  Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

# Path to compiler.
set(compiler_path /home/roboros/gcc-arm-none-eabi-10.3-2021.10)

# Compiler paths
# set(CMAKE_CXX_COMPILER "${compiler_path}/bin/arm-none-eabi-g++")
set(CMAKE_C_COMPILER "${compiler_path}/bin/arm-none-eabi-gcc")
# Compiler flags
set(compiler_flags
    "-mcpu=cortex-m4 -mthumb -g -O0 "
)
set(linker_flags
    "-specs=nosys.specs -T /home/roboros/stm32_projects/stm32_crosscomp/startup/stm32f410rbtx.ld -nostdlib -ffreestanding"
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${compiler_flags}")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${compiler_flags}")
set(CMAKE_EXE_LINKER_FLAGS "${linker_flags}")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)