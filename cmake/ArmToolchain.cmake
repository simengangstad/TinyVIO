# -------------------------- Toolchain --------------------------

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(ARM_TOOLCHAIN_VERSION 11.3.1) 

# First find the folder for the compiler
if(MINGW OR CYGWIN OR WIN32)
    set(UTIL_SEARCH_CMD where)
elseif(UNIX OR APPLE)
    set(UTIL_SEARCH_CMD which)
endif()

execute_process(
  COMMAND ${UTIL_SEARCH_CMD} arm-none-eabi-gcc
  OUTPUT_VARIABLE ARM_GCC_BINARY_PATH 
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if (ARM_GCC_BINARY_PATH STREQUAL "")
    message(FATAL_ERROR "ARM toolchain was not found on path, you need to download it and add the binaries to path")
endif()

# Where the binaries for the compiler is located
get_filename_component(ARM_TOOLCHAIN_BINARY_DIRECTORY ${ARM_GCC_BINARY_PATH} DIRECTORY)

# Top level directory for the toolchain
set(ARM_TOOLCHAIN_DIRECTORY
    ${ARM_TOOLCHAIN_BINARY_DIRECTORY}/..
)


# Main include directory
set(ARM_TOOLCHAIN_INCLUDE_DIR 
    ${ARM_TOOLCHAIN_DIRECTORY}/share/arm-none-eabi/arm-none-eabi/include
)

# C++ include directory, this is mostly used for LSP
set(ARM_TOOLCHAIN_CXX_INCLUDE_DIR 
    ${ARM_TOOLCHAIN_DIRECTORY}/share/arm-none-eabi/arm-none-eabi/include/c++/${ARM_TOOLCHAIN_VERSION}
)

# Floating point include directory, this is mostly used for LSP
set(ARM_TOOLCHAIN_CXX_FP_INCLUDE_DIR 
    ${ARM_TOOLCHAIN_DIRECTORY}/share/arm-none-eabi/arm-none-eabi/include/c++/${ARM_TOOLCHAIN_VERSION}/arm-none-eabi/thumb/v7e-m+fp/hard
)

set(CMAKE_C_COMPILER    ${ARM_TOOLCHAIN_BINARY_DIRECTORY}/arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER  ${ARM_TOOLCHAIN_BINARY_DIRECTORY}/arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER  ${ARM_TOOLCHAIN_BINARY_DIRECTORY}/arm-none-eabi-g++)
set(CMAKE_OBJCOPY       ${ARM_TOOLCHAIN_BINARY_DIRECTORY}/arm-none-eabi-objcopy)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)


# ---------------------- Definitions & flags ----------------------

set(TOOLCHAIN_INSTRUCTION_SPECIFICATION
    -mfpu=fpv5-d16
    -mfloat-abi=hard
    -mthumb
)

set(TOOLCHAIN_COMPILE_DEFINITIONS
    CPU_MIMXRT1176DVMAA
    __USE_CMSIS 
    __USE_SHMEM
    __MCUXPRESSO

    USE_SDRAM
    CACHE_MODE_WRITE_THROUGH=1
    FSL_SDK_ENABLE_DRIVER_CACHE_CONTROL=1

    $<$<CONFIG:Debug>:DEBUG>
    $<$<CONFIG:Release>:NDEBUG>
)

set(TOOLCHAIN_COMPILE_OPTIONS
    ${TOOLCHAIN_INSTRUCTION_SPECIFICATION}

    -Werror
    -Wall 
    -Wextra 
    -Wpedantic 
    -Wshadow 
    -Wno-vla 

    -fno-common
    -ffunction-sections
    -fdata-sections
    -fno-exceptions
    -fmerge-constants
    -fstack-usage
    -ffreestanding
    
    # Need to add this here as CMake won't add it with target_include_directories
    # for some reason. Want this here so that the compile_commands.json contains 
    # the standard library headers as well
    -I${ARM_TOOLCHAIN_INCLUDE_DIR} 
    -I${ARM_TOOLCHAIN_CXX_INCLUDE_DIR} 
    -I${ARM_TOOLCHAIN_CXX_FP_INCLUDE_DIR}

    $<$<CONFIG:Debug>:-O0>
    $<$<CONFIG:Debug>:-g>
    $<$<CONFIG:Release>:-O3>
    $<$<CONFIG:Release>:-Ofast>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-volatile>
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-register>
)

set(TOOLCHAIN_LINK_OPTIONS
    ${TOOLCHAIN_INSTRUCTION_SPECIFICATION}

    -specs=nosys.specs
    -nostdlib
    -nostartfiles

    -Wl,--print-memory-usage
    -Wl,--gc-section
    -Wl,--sort-section=alignment
    -Wl,--cref

    $<$<CONFIG:Debug>:-O0>
    $<$<CONFIG:Debug>:-g>
    $<$<CONFIG:Release>:-O3>
    $<$<CONFIG:Release>:-Ofast>
)
