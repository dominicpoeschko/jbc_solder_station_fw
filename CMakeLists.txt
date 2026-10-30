cmake_minimum_required(VERSION 3.28)

# Kvasir_SDK: KVASIR_ROOT (variable or environment), else the checkout next to this repository, else fetched; the chip
# package and kvasir_devices the same way (kvasir_locate: Kvasir_SDK cmake/kvasir_locate.cmake)
if(NOT KVASIR_ROOT AND DEFINED ENV{KVASIR_ROOT})
    set(KVASIR_ROOT "$ENV{KVASIR_ROOT}")
endif()
if(NOT KVASIR_ROOT AND EXISTS "${CMAKE_CURRENT_LIST_DIR}/../Kvasir_SDK/cmake/kvasir_locate.cmake")
    get_filename_component(KVASIR_ROOT "${CMAKE_CURRENT_LIST_DIR}/../Kvasir_SDK" ABSOLUTE)
elseif(NOT KVASIR_ROOT)
    include(FetchContent)
    FetchContent_Declare(
        kvasir_sdk
        GIT_REPOSITORY https://github.com/kvasir-io/Kvasir_SDK.git
        GIT_TAG master
        GIT_SHALLOW TRUE
        GIT_SUBMODULES "" SOURCE_SUBDIR fetch-only)
    FetchContent_MakeAvailable(kvasir_sdk)
    set(KVASIR_ROOT "${kvasir_sdk_SOURCE_DIR}")
endif()
include("${KVASIR_ROOT}/cmake/kvasir_locate.cmake")
kvasir_locate(USE_ENVIRONMENT SDK_SUBMODULES ALL CHIP chip_atsamd21 DEVICES)

include(${KVASIR_ROOT}/cmake/kvasir.cmake)

project(
    "jbc_solder_station_fw"
    VERSION 0.1.0
    DESCRIPTION "JBC solder station firmware using Kvasir SDK")

include(${KVASIR_ROOT}/cmake/util.cmake)

# Two variants, debug and release, both built for size.
kvasir_executable_variants(
    ""
    SOURCES
    src/main.cpp
    LIBRARIES
    kvasir::devices
    VARIANTS
    debug
    release
    OPTIMIZATION
    size
    DEBUG_OPTIMIZATION
    size)
