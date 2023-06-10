cmake_minimum_required(VERSION 3.20)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ------------------------- Toolchain -------------------------

set(DEFAULT_BUILD_TYPE "Release")

if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
    set(CMAKE_BUILD_TYPE  "${DEFAULT_BUILD_TYPE}")
endif()

include(cmake/ArmToolchain.cmake)

# ------------------------- Dependencies ----------------------

include(cmake/FetchDependencies.cmake)

# ------------------------------ SDK --------------------------

set(SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/nxp)
set(CMSIS_PATH ${SDK_PATH}/CMSIS)
set(DEVICE_PATH ${SDK_PATH}/devices/MIMXRT1176)
set(DRIVER_PATH ${DEVICE_PATH}/drivers)
set(MIDDLEWARE_PATH ${SDK_PATH}/middleware)
set(COMPONENT_PATH ${SDK_PATH}/components)

set(CMAKE_MODULE_PATH 
    ${CMAKE_MODULE_PATH} 
    ${CMSIS_PATH}/Core/Include 
    ${DEVICE_PATH} 
    ${DEVICE_PATH}/utilities
    ${DEVICE_PATH}/utilities/incbin
    ${DRIVER_PATH} 
    ${DRIVER_PATH}/cm4
    ${DRIVER_PATH}/cm7
    ${MIDDLEWARE_PATH}
    ${MIDDLEWARE_PATH}/multicore 
    ${MIDDLEWARE_PATH}/fatfs
    ${MIDDLEWARE_PATH}/sdmmc
    ${MIDDLEWARE_PATH}/lwip
    ${COMPONENT_PATH}/osa
    ${COMPONENT_PATH}/lists
    ${COMPONENT_PATH}/phy
    ${COMPONENT_PATH}/phy/device/phyrtl8211f
    ${COMPONENT_PATH}/phy/device/phyksz8081
    ${COMPONENT_PATH}/phy/mdio/enet
    ${COMPONENT_PATH}/silicon_id
    ${COMPONENT_PATH}/silicon_id/socs/imxrt1170
)

# ---------------------- Common variables ----------------------

project(TinyVIO)

enable_language(C CXX ASM)

set(CM7_TARGET core0.elf)
set(CM4_TARGET core1.elf)

add_compile_options(${TOOLCHAIN_COMPILE_OPTIONS})

add_compile_definitions(
    ${TOOLCHAIN_COMPILE_DEFINITIONS}

    # Defined when depencies are fetched
    ${EIGEN_FLAGS}
    EIGEN_NO_IO
    ${ETL_FLAGS}
)

# ------------------------- CM4 Target -------------------------

file(GLOB CM4_SOURCES 
    src/*.cpp
    src/core/*.cpp
    src/core/boot/startup_mimxrt1176_cm4.cpp
    src/drivers/*.cpp
    src/util/*.cpp
)

list(REMOVE_ITEM CM4_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main_cm7.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/data_interface.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/util/profile.cpp
)

# This is needed for the SDK
set(MCUX_SDK_PROJECT_NAME ${CM4_TARGET})

add_executable(${CM4_TARGET} ${CM4_SOURCES})

target_compile_definitions(${CM4_TARGET} PRIVATE
    -DCPU_MIMXRT1176DVMAA_cm4
    -DMULTICORE_SLAVE
    -DSKIP_SEMC_INIT
)

# Device 
include(device_MIMXRT1176_CMSIS_MIMXRT1176_cm4)
include(device_MIMXRT1176_system_MIMXRT1176_cm4)


# Drivers
include(driver_anatop_ai_MIMXRT1176_cm4)
include(driver_cache_lmem_MIMXRT1176_cm4)
include(driver_clock_MIMXRT1176_cm4)
include(driver_common_MIMXRT1176_cm4)
include(driver_dcdc_soc_MIMXRT1176_cm4)
include(driver_igpio_MIMXRT1176_cm4)
include(driver_lpuart_MIMXRT1176_cm4)
include(driver_mu_MIMXRT1176_cm4)
include(driver_pmu_1_MIMXRT1176_cm4)

# Middleware
set(CONFIG_USE_middleware_multicore_rpmsg_lite_bm_MIMXRT1176_cm4 true)
set(CONFIG_USE_middleware_baremetal_MIMXRT1176_cm4 true)
include(middleware_multicore_mcmgr_MIMXRT1176_cm4)
include(middleware_multicore_mcmgr_imxrt1170_MIMXRT1176_cm4)
include(middleware_multicore_rpmsg_lite_MIMXRT1176_cm4)
include(middleware_multicore_rpmsg_lite_imxrt1170_bm_MIMXRT1176_cm4)

target_include_directories(${CM4_TARGET} PRIVATE 
    src
    src/core
    src/core/device
    src/core/device/cm4
    src/drivers
    src/util
)

target_compile_options(${CM4_TARGET} PRIVATE
    -mcpu=cortex-m4
)

# Remove warnings from compiled units of the SDK. This will still propagate errors
# when used in source files in the project, but not when the SDK files are compiled
# themselves
get_target_property(SDK_SOURCES_NO_WARNINGS ${MCUX_SDK_PROJECT_NAME} SOURCES)
list(FILTER SDK_SOURCES_NO_WARNINGS INCLUDE REGEX "\\${SDK_PATH}")
set_source_files_properties(${SDK_SOURCES_NO_WARNINGS} PROPERTIES COMPILE_FLAGS 
    "${COMPILE_FLAGS} -w"
)

target_link_options(${CM4_TARGET} PRIVATE
    -T${CMAKE_SOURCE_DIR}/linker/mimxrt1170_cm4.ld
    -mcpu=cortex-m4
    -Wl,-Map=${CM4_TARGET}.map
    ${TOOLCHAIN_LINK_OPTIONS}
)

add_custom_command(TARGET ${CM4_TARGET}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${CM4_TARGET} core1_image.bin
)

# Dummy target which keeps track of changes in CORE1 so that CORE0 is re-built 
# after a change in CORE1
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/cm4_is_built
  COMMAND ${CMAKE_COMMAND} -E touch  ${CMAKE_CURRENT_BINARY_DIR}/cm4_is_built
  DEPENDS $<TARGET_FILE:${CM4_TARGET}>
)


# ------------------------- CM7 Target -------------------------

file(GLOB CM7_SOURCES 
    src/*.cpp
    src/core/*.cpp
    src/core/boot/*.cpp
    src/core/boot/*.c
    src/drivers/*.cpp
    src/math/*.cpp
    src/net/*.cpp
    src/util/*.cpp
    src/vio/*.cpp
    src/vio/backend/*.cpp
    src/vio/feature/*.cpp
    src/vio/frontend/*.cpp
    src/vio/sensors/*.cpp
)

list(REMOVE_ITEM CM7_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main_cm4.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/core/boot/startup_mimxrt1176_cm4.cpp
)

# This is needed for the SDK
set(MCUX_SDK_PROJECT_NAME ${CM7_TARGET})

add_executable(${CM7_TARGET} ${CM7_SOURCES})
add_dependencies(${CM7_TARGET} ${CM4_TARGET})

target_compile_definitions(${CM7_TARGET} PRIVATE
    -DCPU_MIMXRT1176DVMAA_cm7
    -DMULTICORE_MASTER

    -DUSE_LWIP
    -DLWIP_DISABLE_PBUF_POOL_SIZE_SANITY_CHECKS=1
    -DFSL_FEATURE_PHYKSZ8081_USE_RMII50M_MODE
    -DENET_RXBUFF_NUM=20
    -DENET_RXBD_NUM=9
    -DLWIP_ENET_FLEXIBLE_CONFIGURATION
    -DLWIP_TIMEVAL_PRIVATE=0

    -DXIP_EXTERNAL_FLASH=1
    -DXIP_BOOT_HEADER_ENABLE=1
    -DXIP_BOOT_HEADER_DCD_ENABLE=1
)

# CMSIS
include(device_MIMXRT1176_CMSIS_MIMXRT1176_cm7)
include(device_MIMXRT1176_system_MIMXRT1176_cm7)

# Device 
include(utility_incbin_MIMXRT1176_cm7)

# Drivers
include(driver_anatop_ai_MIMXRT1176_cm7)
include(driver_cache_armv7_m7_MIMXRT1176_cm7)
include(driver_clock_MIMXRT1176_cm7)
include(driver_common_MIMXRT1176_cm7)
include(driver_dcdc_soc_MIMXRT1176_cm7)
include(driver_igpio_MIMXRT1176_cm7)
include(driver_lpuart_MIMXRT1176_cm7)
include(driver_mu_MIMXRT1176_cm7)
include(driver_pmu_1_MIMXRT1176_cm7)
include(driver_usdhc_MIMXRT1176_cm7)

# --- Middleware ---

# Multicore
set(CONFIG_USE_middleware_multicore_rpmsg_lite_bm_MIMXRT1176_cm7 true)
set(CONFIG_USE_middleware_baremetal_MIMXRT1176_cm7 true)
include(middleware_multicore_mcmgr_MIMXRT1176_cm7)
include(middleware_multicore_mcmgr_imxrt1170_MIMXRT1176_cm7)
include(middleware_multicore_rpmsg_lite_MIMXRT1176_cm7)
include(middleware_multicore_rpmsg_lite_imxrt1170_bm_MIMXRT1176_cm7)

# LWIP
set(CONFIG_USE_middleware_lwip_enet_ethernetif_MIMXRT1176_cm7 true)
set(CONFIG_USE_driver_enet_MIMXRT1176_cm7 true)
set(CONFIG_USE_driver_common_MIMXRT1176_cm7 true)
include(driver_phy-device-rtl8211f_MIMXRT1176_cm7)
include(driver_phy-device-ksz8081_MIMXRT1176_cm7)
include(middleware_lwip_MIMXRT1176_cm7)
include(middleware_lwip_enet_ethernetif_MIMXRT1176_cm7)
include(driver_phy-common_MIMXRT1176_cm7)
include(driver_enet_MIMXRT1176_cm7)
include(component_silicon_id_MIMXRT1176_cm7)
include(component_silicon_id_rt1170_MIMXRT1176_cm7)

target_include_directories(${CM7_TARGET} PRIVATE
    src
    src/core
    src/core/boot
    src/core/device
    src/core/device/cm7
    src/drivers
    src/math
    src/net
    src/util
    src/vio
    src/vio/backend
    src/vio/feature
    src/vio/frontend
    src/vio/sensors
)

target_compile_options(${CM7_TARGET} PRIVATE
    -mcpu=cortex-m7
)

# Remove warnings from compiled units of the SDK. This will still propagate errors
# when used in source files in the project, but not when the SDK files are compiled
# themselves
get_target_property(SDK_SOURCES_NO_WARNINGS ${MCUX_SDK_PROJECT_NAME} SOURCES)
list(FILTER SDK_SOURCES_NO_WARNINGS INCLUDE REGEX 
    "\\${SDK_PATH}|\\${CMAKE_CURRENT_SOURCE_DIR}/src/core/boot/flexspi_boot.c"
)
set_source_files_properties(${SDK_SOURCES_NO_WARNINGS} PROPERTIES COMPILE_FLAGS 
    "${COMPILE_FLAGS} -w"
)

# If the CM4 core's code changes, we want to re-build the CM7 core's related parts 
# and for the CM4 core's binary
set_source_files_properties(
    src/core/device/cm7/inc_core1_bin.S 
    src/main_cm7.cpp
    PROPERTIES OBJECT_DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/cm4_is_built
)

target_link_options(${CM7_TARGET} PRIVATE
    -T${CMAKE_SOURCE_DIR}/linker/mimxrt1170_cm7.ld
    -mcpu=cortex-m7
    -Wl,-Map=${CM7_TARGET}.map
    ${TOOLCHAIN_LINK_OPTIONS}
)

target_link_libraries(${CM7_TARGET} PRIVATE
    etl::etl
    Eigen3::Eigen
)

add_custom_command(TARGET ${CM7_TARGET}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${CM7_TARGET} ${CM7_TARGET}.hex
)

# --------------------- Flashing & Debugging ---------------------


set(FLASH_SCRIPT ../scripts/jlink/flash.jlinkscript)
set(DEBUG_SCRIPT ../scripts/jlink/debug.jlinkscript)
set(DEVICE_M7 MIMXRT1176XXXA_M7) 
set(DEVICE_M4 MIMXRT1176XXXA_M4) 
set(SN 725439210)

add_custom_target(flash
    DEPENDS ${CM7_TARGET} 
    COMMAND JLinkExe -nogui 1 -if SWD -speed 32000 -commanderscript ${FLASH_SCRIPT} -device ${DEVICE_M7} -SelectEmuBySn ${SN}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

add_custom_target(gdbcore0
    DEPENDS ${CM7_TARGET} 
    COMMAND JLinkGDBServer -nogui 1 -jlinkscriptfile ${DEBUG_SCRIPT} -device ${DEVICE_M7} -if SWD -speed 32000
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

add_custom_target(gdbcore1
    DEPENDS ${CM7_TARGET} 
    COMMAND JLinkGDBServer -nogui 1 -jlinkscriptfile ${DEBUG_SCRIPT} -device ${DEVICE_M4} -if SWD -speed 32000
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
