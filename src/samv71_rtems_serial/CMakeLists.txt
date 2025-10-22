add_library(SamV71RtemsSerial STATIC)
target_sources(SamV71RtemsSerial
  PRIVATE   samv71_rtems_serial.c
  PUBLIC    samv71_rtems_serial.h
            samv71_rtems_serial_internal.h)

 target_include_directories(SamV71RtemsSerial
  PUBLIC    ${CMAKE_SOURCE_DIR}/src/RuntimeMocks
            ${CMAKE_SOURCE_DIR}/TASTE-SAMV71-RTEMS-Runtime/src
            ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(SamV71RtemsSerial
  PRIVATE   common_build_options
  PUBLIC    SAMV71::Runtime
            SAMV71::Runtime::Core
            TASTE::Broker
            TASTE::Escaper)

add_format_target(SamV71RtemsSerial)

add_library(TASTE::SamV71RtemsSerial ALIAS SamV71RtemsSerial)
