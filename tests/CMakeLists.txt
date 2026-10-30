# The host tests of the remote protocol (src/remote/): framing, the command handler and its ranges, and the remote's
# model (tui/Model.hpp, host/Frames.hpp). They build from a lone checkout - aglio comes from the sibling checkout in
# kvasir_work when there is one, else from GitHub.
#
# * clang: cmake -S tests -B build/tests-clang -G Ninja -DCMAKE_CXX_COMPILER=clang++
# * gcc:   cmake -S tests -B build/tests-gcc -G Ninja -DCMAKE_CXX_COMPILER=g++
# * then:  cmake --build build/tests-<c> && ctest --test-dir build/tests-<c> --output-on-failure
cmake_minimum_required(VERSION 3.28)

project(jbc_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

include(FetchContent)
if(NOT DEFINED FETCHCONTENT_SOURCE_DIR_AGLIO AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../../aglio/CMakeLists.txt)
    set(FETCHCONTENT_SOURCE_DIR_AGLIO ${CMAKE_CURRENT_SOURCE_DIR}/../../aglio)
endif()
# Without enchantum (enum names in aglio's printing, not needed here) one dependency less to fetch.
set(AGLIO_USE_ENCHANTUM
    OFF
    CACHE BOOL "" FORCE)
FetchContent_Declare(
    aglio
    GIT_REPOSITORY https://github.com/dominicpoeschko/aglio.git
    GIT_TAG master
    SYSTEM)
FetchContent_MakeAvailable(aglio)

enable_testing()

# The firmware's language flags (no exceptions, no RTTI) and warning set: clang -Weverything with ASan/UBSan, gcc -Wall
# -Wextra -Wconversion.
function(jbc_test name)
    add_executable(${name} ${name}.cpp)
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../src)
    target_link_libraries(${name} PRIVATE aglio::aglio)
    target_compile_options(${name} PRIVATE -fno-exceptions -fno-rtti)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(
            ${name}
            PRIVATE -Weverything
                    -Wno-c++98-compat
                    -Wno-c++98-compat-pedantic
                    -Wno-c++20-compat
                    -Wno-pre-c2x-compat
                    -Wno-padded
                    -Wno-switch-default
                    -Wno-covered-switch-default
                    -Wno-unsafe-buffer-usage
                    -Wno-exit-time-destructors
                    -Wno-global-constructors
                    -Wno-unknown-warning-option
                    -Wno-nrvo
                    -Wno-lifetime-safety-intra-tu-suggestions
                    -Wno-lifetime-safety-intra-tu-constructor-suggestions
                    -fsanitize=address,undefined
                    -fno-sanitize-recover=all
                    -fno-omit-frame-pointer)
        target_link_options(${name} PRIVATE -fsanitize=address,undefined)
    else()
        target_compile_options(${name} PRIVATE -Wall -Wextra -pedantic -Wconversion -Wsign-conversion)
    endif()
    add_test(NAME ${name} COMMAND ${name})
    set_tests_properties(${name} PROPERTIES ENVIRONMENT "UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1")
endfunction()

jbc_test(protocol_test)
jbc_test(handler_test)
# jbc_tui's model and graphs (tui/Model.hpp, tui/Plot.hpp) and the host's frame reader (host/Frames.hpp): no RTXUI, no
# sockets.
jbc_test(tui_model_test)
target_include_directories(tui_model_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../host
                                                  ${CMAKE_CURRENT_SOURCE_DIR}/../tui)
