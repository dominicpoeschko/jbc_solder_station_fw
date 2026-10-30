# jbc_tui: the station's remote in a terminal (RTXUI: HTML templates and CSS), a host program built on its own (`just
# tui`), not part of the firmware build. It talks src/remote/Protocol.hpp to the socket uc_log_printer opens for the
# firmware's "control" duplex channel, or to SERCOM1's UART on the debug pins.
#
# jbc_fake is the station for testing it without one: the firmware's command handler and a simulated tip behind a unix
# socket. The end-to-end test (e2e_test.sh) runs jbc_tui --dump against it, and renders the screen headless.
#
# cmake -S tui -B build_tui -G Ninja && cmake --build build_tui && ctest --test-dir build_tui --output-on-failure
cmake_minimum_required(VERSION 3.28)

project(jbc_tui LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include(FetchContent)

# aglio: the sibling checkout if it is there, as tests/ takes it; its own CMake brings glaze.
if(NOT DEFINED FETCHCONTENT_SOURCE_DIR_AGLIO AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../../aglio/CMakeLists.txt)
    set(FETCHCONTENT_SOURCE_DIR_AGLIO ${CMAKE_CURRENT_SOURCE_DIR}/../../aglio)
endif()
set(AGLIO_USE_ENCHANTUM
    OFF
    CACHE BOOL "" FORCE)
FetchContent_Declare(
    aglio
    GIT_REPOSITORY https://github.com/dominicpoeschko/aglio.git
    GIT_TAG master
    SYSTEM)
FetchContent_MakeAvailable(aglio)

# RTXUI's first release (psu_hmi's pin), always fetched into this tree's own _deps (never another tree's copy).
FetchContent_Declare(
    rtxui
    GIT_REPOSITORY https://github.com/ArthurSonzogni/RTXUI
    GIT_TAG v0.1.1
    SYSTEM)
FetchContent_MakeAvailable(rtxui)
# RTXUI's own sources, not ours (v0.1.1 has one library target, `rtxui`)
target_compile_options(rtxui PRIVATE -w)
set_target_properties(rtxui PROPERTIES COMPILE_WARNING_AS_ERROR OFF)

set(JBC_HOST_WARNINGS -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow)
set(JBC_HOST_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../host
                      ${CMAKE_CURRENT_SOURCE_DIR}/../src)

add_executable(jbc_tui main.cpp)
target_include_directories(jbc_tui PRIVATE ${JBC_HOST_INCLUDES})
target_link_libraries(jbc_tui PRIVATE aglio::aglio rtxui)
# Where `just log` puts the printer's unix sockets: the tui looks there for <target>/duplex.0.sock when not told
# otherwise ($JBC_RTT_LOG_DIR overrides it).
cmake_path(SET JBC_RTT_LOG_DIR NORMALIZE "${CMAKE_CURRENT_SOURCE_DIR}/../build/rtt_log")
target_compile_definitions(jbc_tui PRIVATE TUI_RTT_LOG_DIR="${JBC_RTT_LOG_DIR}")
target_compile_options(jbc_tui PRIVATE ${JBC_HOST_WARNINGS})

add_executable(jbc_fake fake_station.cpp)
target_include_directories(jbc_fake PRIVATE ${JBC_HOST_INCLUDES})
target_link_libraries(jbc_fake PRIVATE aglio::aglio)
target_compile_options(jbc_fake PRIVATE ${JBC_HOST_WARNINGS})

enable_testing()
add_test(NAME tui_e2e COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/e2e_test.sh $<TARGET_FILE:jbc_fake> $<TARGET_FILE:jbc_tui>
                              ${CMAKE_CURRENT_BINARY_DIR}/e2e)
set_tests_properties(tui_e2e PROPERTIES TIMEOUT 120)
