add_executable(Samv71App)
target_sources(Samv71App
    PRIVATE     main.c)

target_include_directories(Samv71App
    PRIVATE     ${CMAKE_SOURCE_DIR}/src
                ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(Samv71App
    PRIVATE     common_build_options
                SAMV71::Runtime::Hal
                TASTE::SamV71RtemsSerial
                TASTE::Broker
                SAMV71::RuntimeMocks
                SAMV71::Runtime::BrokerLock)

add_format_target(Samv71App)
