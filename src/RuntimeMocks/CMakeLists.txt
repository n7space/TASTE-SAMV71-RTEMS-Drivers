add_library(SamV71RuntimeMocks STATIC)
target_sources(SamV71RuntimeMocks
  PRIVATE   asn1crt.c
            asn1crt_encoding.c
            asn1crt_encoding_acn.c
            asn1crt_encoding_uper.c
            dataview-uniq.c
            drivers_config.c
            interfaces_info.c
            samv71-rtems-serial-driver.c
            samv71-rtems-can-driver.c
            system_spec.c
  PUBLIC    asn1crt.h
            asn1crt_encoding.h
            asn1crt_encoding_acn.h
            asn1crt_encoding_uper.h
            dataview-uniq.h
            drivers_config.h
            interfaces_info.h
            samv71-rtems-serial-driver.h
            samv71-rtems-can-driver.h
            system_spec.h)

target_include_directories(SamV71RuntimeMocks
  PUBLIC    ${CMAKE_SOURCE_DIR}/TASTE-SAMX-RTEMS-Runtime/src
            ${CMAKE_CURRENT_SOURCE_DIR})

add_format_target(SamV71RuntimeMocks)

add_library(SAMV71::RuntimeMocks ALIAS SamV71RuntimeMocks)
