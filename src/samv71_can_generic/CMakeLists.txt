add_library(SamV71RtemsCan STATIC)
target_sources(SamV71RtemsCan
  PRIVATE   samv71_can_generic.c
  PUBLIC    samv71_can_generic.h)

 target_include_directories(SamV71RtemsCan
  PUBLIC    ${CMAKE_SOURCE_DIR}/src/RuntimeMocks
            ${CMAKE_SOURCE_DIR}/TASTE-SAMV71-RTEMS-Runtime/src
            ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(SamV71RtemsCan
  PRIVATE   common_build_options
  PUBLIC    SAMV71::Runtime::Hal
            SAMV71::Runtime::Core
            TASTE::Broker
            TASTE::Escaper)

add_format_target(SamV71RtemsCan)

add_library(TASTE::SamV71RtemsCan ALIAS SamV71RtemsCan)
