cmake_minimum_required(VERSION 3.28)

if(NOT KVASIR_ROOT)
    if(DEFINED ENV{KVASIR_ROOT})
        set(KVASIR_ROOT $ENV{KVASIR_ROOT})
    else()
        include(FetchContent)
        FetchContent_Declare(
            Kvasir_SDK
            GIT_REPOSITORY https://github.com/kvasir-io/Kvasir_SDK.git
            GIT_TAG master)
        FetchContent_Declare(
            chip_atsamd21
            GIT_REPOSITORY https://github.com/kvasir-io/chip_atsamd21.git
            GIT_TAG master)
        FetchContent_Declare(
            kvasir_devices
            GIT_REPOSITORY https://github.com/kvasir-io/kvasir_devices.git
            GIT_TAG master)
        FetchContent_MakeAvailable(Kvasir_SDK chip_atsamd21 kvasir_devices)
        set(KVASIR_ROOT ${kvasir_sdk_SOURCE_DIR})
        set(CHIP_ROOT ${chip_atsamd21_SOURCE_DIR})
        set(KVASIR_DEVICES_ROOT ${kvasir_devices_SOURCE_DIR})
    endif()
endif()

include(${KVASIR_ROOT}/cmake/kvasir.cmake)

project(
    "jbc_solder_station_fw"
    VERSION 0.1.0
    DESCRIPTION "JBC solder station firmware using Kvasir SDK")

include(${KVASIR_ROOT}/cmake/util.cmake)

add_executable(debug src/main.cpp)
target_configure_kvasir(debug OPTIMIZATION_STRATEGY size USE_LOG)
target_link_libraries(debug kvasir::devices)

add_executable(release src/main.cpp)
target_configure_kvasir(release OPTIMIZATION_STRATEGY size)
target_link_libraries(release kvasir::devices)
