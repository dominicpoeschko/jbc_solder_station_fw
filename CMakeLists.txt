cmake_minimum_required(VERSION 3.19)

include(kvasir/cmake/kvasir.cmake)

project("JBC Solder Station" VERSION 0.1.0)

include(kvasir/cmake/util.cmake)
add_subdirectory(kvasir_devices)

add_executable(debug src/main.cpp)
target_configure_kvasir(debug OPTIMIZATION_STRATEGY size USE_LOG)
target_link_libraries(debug kvasir::devices)

add_executable(release src/main.cpp)
target_configure_kvasir(release OPTIMIZATION_STRATEGY size)
target_link_libraries(release kvasir::devices)
