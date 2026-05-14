cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/TASTE-SAMX-RTEMS-Runtime/toolchains/samv71.cmake)
project(TASTE-SAMV71-Drivers VERSION 1.0.0 LANGUAGES C)

set(CMAKE_C_STANDARD 99)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/build/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/build/lib)
set(TESTS_OUTPUT_PATH ${CMAKE_BINARY_DIR}/build/tests)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

option(WITH_CLANGT_TIDY "Enable clang-tidy static code analyzer" OFF)

include(MessageColors)
include(LoggingHelper)

add_library(common_build_options INTERFACE)
include(CommonBuildOptions)
include(Sanitizers)

if(WITH_CLANGT_TIDY)
    message("Checking for clang-tidy")
    find_program(CLANG_TIDY_EXE clang-tidy
        NAMES clang-tidy
        clang-tidy-19
        clang-tidy-20
        clang-tidy-21
        clang-tidy-22
        clang-tidy-23
        REQUIRED)
    if(CLANG_TIDY_EXE MATCHES "-NOTFOUND")
        message(FATAL_ERROR "Clang-tidy not found")
    else()
        message("Clang-tidy executable: ${CLANG_TIDY_EXE}")
    endif()
endif()

option(TASTE_SAMV71_DRIVERS_WARNINGS_AS_ERRORS
       "Treat compiler warnings as errors"
       TRUE)

if(TASTE_SAMV71_DRIVERS_WARNINGS_AS_ERRORS)
    log_option_enabled("warnings as errors")
    set(CLANG_WARNINGS -Werror)
endif()

set(CLANG_WARNINGS ${CLANG_WARNINGS}
                   -Wall
                   -Wextra
                   -Wpedantic
                   -Wshadow
                   -Wcast-align
                   -Wunused
                   -Wsign-conversion
                   -Wnull-dereference
                   -Wdouble-promotion
                   -Wformat=2)

set(GCC_WARNINGS ${CLANG_WARNINGS}
                 -Wimplicit-function-declaration
                 -Wmisleading-indentation)

if(NOT WITH_CLANGT_TIDY)
    set(GCC_WARNINGS ${GCC_WARNINGS}
        -Wduplicated-branches
        -Wduplicated-cond
        -Wlogical-op)
endif()

log_info(${CMAKE_C_COMPILER_ID})

if(CMAKE_C_COMPILER_ID STREQUAL "Clang")
    set(PROJECT_WARNINGS ${CLANG_WARNINGS})
elseif(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    set(PROJECT_WARNINGS ${GCC_WARNINGS})
endif()

target_compile_options(common_build_options INTERFACE ${PROJECT_WARNINGS})


add_library(bsp_build_options INTERFACE)
target_compile_definitions(bsp_build_options
    INTERFACE   USE_UART_IO
                LOW_LEVEL_IO_UART_ID=4)

include(CppCheck)
include(ClangFormat)
include(Doxygen)

if(WITH_CLANGT_TIDY)
    set(CMAKE_CXX_CLANG_TIDY clang-tidy; --use-color; --config-file=${CMAKE_CURRENT_LIST_DIR}/.clang-tidy; --header-filter=.;)
    set(CMAKE_C_CLANG_TIDY clang-tidy; --use-color; --config-file=${CMAKE_CURRENT_LIST_DIR}/.clang-tidy; --header-filter=.;)
endif()

include_directories(SYSTEM
  /opt/rtems/arm-rtems6/atsamv/lib/include
  /opt/rtems/arm-rtems6/include)

add_subdirectory(TASTE-Runtime-Common/src/RuntimeMocks)
add_subdirectory(TASTE-Runtime-Common/src/RuntimeCommon)
add_subdirectory(TASTE-Runtime-Common/src/Packetizer)
add_subdirectory(TASTE-Runtime-Common/src/Broker)
add_subdirectory(TASTE-Runtime-Common/src/Escaper)
add_subdirectory(TASTE-SAMX-RTEMS-Runtime/arm-bsp/src/Startup)
add_subdirectory(TASTE-SAMX-RTEMS-Runtime/arm-bsp/src/Nvic)
add_subdirectory(TASTE-SAMX-RTEMS-Runtime/arm-bsp/src/Wdt)
add_subdirectory(TASTE-SAMX-RTEMS-Runtime/arm-bsp/src/Fpu)
add_subdirectory(TASTE-SAMX-RTEMS-Runtime/arm-bsp/src/Sdramc)
add_subdirectory(TASTE-SAMX-RTEMS-Runtime/arm-bsp/src/Scb)
add_subdirectory(TASTE-SAMX-RTEMS-Runtime/arm-bsp/src/Rstc)
add_subdirectory(TASTE-SAMX-RTEMS-Runtime/arm-bsp/src/Pio)
add_subdirectory(TASTE-SAMX-RTEMS-Runtime/arm-bsp/src/Mpu)
add_subdirectory(TASTE-SAMX-RTEMS-Runtime/arm-bsp/src/Systick)
add_subdirectory(TASTE-SAMX-RTEMS-Runtime/arm-bsp/src/Pmc)
add_subdirectory(TASTE-SAMX-RTEMS-Runtime/arm-bsp/src/Tic)
add_subdirectory(TASTE-SAMX-RTEMS-Runtime/arm-bsp/src/Uart)
add_subdirectory(TASTE-SAMX-RTEMS-Runtime/arm-bsp/src/Utils)
add_subdirectory(TASTE-SAMX-RTEMS-Runtime/src)
add_subdirectory(src)
