add_library(SamV71RuntimeMocks STATIC)
target_sources(SamV71RuntimeMocks
  PRIVATE   asn1crt.c
            drivers_config.c
            interfaces_info.c
            linux-serial-ccsds-driver.c
            samv71-rtems-serial-driver.c
            system_spec.c
  PUBLIC    asn1crt.h
            drivers_config.h
            interfaces_info.h
            linux-serial-ccsds-driver.h
            samv71-rtems-serial-driver.h
            system_spec.h)

target_include_directories(SamV71RuntimeMocks
  PUBLIC    ${CMAKE_SOURCE_DIR}/TASTE-SAMV71-RTEMS-Runtime/src
            ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(SamV71RuntimeMocks
  PRIVATE   common_build_options)

add_format_target(SamV71RuntimeMocks)

add_library(SAMV71::RuntimeMocks ALIAS SamV71RuntimeMocks)
