cmake_minimum_required(VERSION 3.30 FATAL_ERROR)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/TASTE-SAMX-RTEMS-Runtime/toolchains/samv71.cmake)

project(TASTE-SAMV71-Drivers VERSION 1.0.0 LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/build/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/build/lib)
set(TESTS_OUTPUT_PATH ${CMAKE_BINARY_DIR}/build/tests)
option(TOOLS_ENABLE_CPPCHECK "Enable Cppcheck" OFF)
option(TASTE_SAMV71_RTEMS_DRIVERS_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" TRUE)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

include(MessageColors)
include(LoggingHelper)
include(ClangFormat)
include(CommonBuildOptions)

if(TOOLS_ENABLE_CPPCHECK)
    find_program(CPPCHECK_EXE cppcheck REQUIRED)
    if(CPPCHECK_EXE MATCHES "-NOTFOUND")
        log_program_missing("cppcheck")
    else()
        log_option_enabled("cppcheck")
    endif()
else()
    log_option_disabled("cppcheck")
endif()

log_info(${CMAKE_C_COMPILER_ID})

set(PROJECT_WARNINGS
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wcast-align
    -Wunused
    -Wsign-conversion
    -Wnull-dereference
    -Wdouble-promotion
    -Wformat=2
    -Wimplicit-function-declaration
    -Wmisleading-indentation)

if(TASTE_SAMV71_RTEMS_DRIVERS_WARNINGS_AS_ERRORS)
    log_option_enabled("warnings as errors")
    list(APPEND PROJECT_WARNINGS -Werror)
endif()

add_library(drivers_common_build_options INTERFACE)
target_compile_options(drivers_common_build_options INTERFACE ${PROJECT_WARNINGS})

if(TOOLS_ENABLE_CPPCHECK)
    set(CMAKE_C_CPPCHECK ${CPPCHECK_EXE}; --error-exitcode=1; --check-level=exhaustive; --language=c; --std=c99; --inline-suppr; --enable=all; --suppress=unmatchedSuppression; --suppress=missingIncludeSystem; --suppress=unusedFunction; --suppress=staticFunction; --suppress=checkersReport;)
endif()

include(FetchContent)
set(TARGET_SAMV71Q21 ON)
FetchContent_Declare(taste-samx-rtems-runtime SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/TASTE-SAMX-RTEMS-Runtime" SYSTEM EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(taste-samx-rtems-runtime)
set(TASTE_RUNTIME_COMMON_BUILD_TESTS OFF)
FetchContent_Declare(taste-runtime-common SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/TASTE-Runtime-Common" SYSTEM EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(taste-runtime-common)

set_target_properties(Samv71Nvic PROPERTIES C_CPPCHECK "" CXX_CPPCHECK "" SKIP_LINTING ON)
set_target_properties(Samv71Wdt PROPERTIES C_CPPCHECK "" CXX_CPPCHECK "" SKIP_LINTING ON)
set_target_properties(Samv71Fpu PROPERTIES C_CPPCHECK "" CXX_CPPCHECK "" SKIP_LINTING ON)
set_target_properties(Samv71Sdramc PROPERTIES C_CPPCHECK "" CXX_CPPCHECK "" SKIP_LINTING ON)
set_target_properties(Samv71Scb PROPERTIES C_CPPCHECK "" CXX_CPPCHECK "" SKIP_LINTING ON)
set_target_properties(Samv71Rstc PROPERTIES C_CPPCHECK "" CXX_CPPCHECK "" SKIP_LINTING ON)
set_target_properties(Samv71Pio PROPERTIES C_CPPCHECK "" CXX_CPPCHECK "" SKIP_LINTING ON)
set_target_properties(Samv71Pmc PROPERTIES C_CPPCHECK "" CXX_CPPCHECK "" SKIP_LINTING ON)
set_target_properties(Samv71Tic PROPERTIES C_CPPCHECK "" CXX_CPPCHECK "" SKIP_LINTING ON)
set_target_properties(Samv71Uart PROPERTIES C_CPPCHECK "" CXX_CPPCHECK "" SKIP_LINTING ON)
set_target_properties(Samv71Utils PROPERTIES C_CPPCHECK "" CXX_CPPCHECK "" SKIP_LINTING ON)
set_target_properties(SamX71Core PROPERTIES C_CPPCHECK "" CXX_CPPCHECK "" SKIP_LINTING ON)
set_target_properties(SamX71Hal PROPERTIES C_CPPCHECK "" CXX_CPPCHECK "" SKIP_LINTING ON)
set_target_properties(SamX71RuntimeXdmac PROPERTIES C_CPPCHECK "" CXX_CPPCHECK "" SKIP_LINTING ON)

set_target_properties(Packetizer PROPERTIES C_CPPCHECK "" CXX_CPPCHECK "" SKIP_LINTING ON)
set_target_properties(Broker PROPERTIES C_CPPCHECK "" CXX_CPPCHECK "" SKIP_LINTING ON)
set_target_properties(Escaper PROPERTIES C_CPPCHECK "" CXX_CPPCHECK "" SKIP_LINTING ON)
set_target_properties(RuntimeCommon PROPERTIES C_CPPCHECK "" CXX_CPPCHECK "" SKIP_LINTING ON)

add_subdirectory(src)
