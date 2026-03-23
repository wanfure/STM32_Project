cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)



# ====================== 核心配置：CLT编译器路径 + 浮点数支持 ======================
# 1. 指定STM32CubeCLT自带的ARM GCC编译器完整路径（Windows必加.exe）
# 注意：请确认你的CLT安装路径是否为以下默认路径，若不是则修改！
set(CMAKE_C_COMPILER "D:/Program Files/STM32CubeCLT_1.18.0/GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe")
set(CMAKE_ASM_COMPILER "D:/Program Files/STM32CubeCLT_1.18.0/GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe")

# 2. 编译选项：开启浮点数格式化 + 适配MCU内核
add_compile_options(
        -mthumb                      # 必加：ARM Thumb指令集（STM32专用）
        -mcpu=cortex-m4              # 【必改】根据你的MCU调整：F1=cm3，F4=cm4，F7=cm7
        # 无FPU（如STM32F103）保留注释；有FPU（如STM32F407）取消注释下面2行
        # -mfpu=fpv4-sp-d16
        # -mfloat-abi=hard
        -u _printf_float             # 核心：开启sprintf/printf浮点数格式化支持
)

# 3. 链接选项：必须与编译选项匹配，否则运行时浮点数格式化失效
add_link_options(
        -mthumb
        -mcpu=cortex-m4             # 同上，匹配MCU内核
        # 无FPU则注释下面2行
        # -mfpu=fpv4-sp-d16
        # -mfloat-abi=hard
        -u _printf_float             # 链接阶段强制加载浮点数格式化库
)
# ===============================================================================





# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME MaiLun)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here

        Core/APP/chassis.c

        Core/BSP/Encoder.c
        Core/BSP/PID.c

        Core/Hardware/Motor.c
        Core/Hardware/elrs.c

)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths

        Core/APP
        Core/BSP
        Core/Hardware

)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
