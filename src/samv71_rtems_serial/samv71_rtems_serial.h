#ifndef SAMV71_RTEMS_SERIAL_H
#define SAMV71_RTEMS_SERIAL_H

#include "samv71_rtems_serial_internal.h"

#include <stddef.h>
#include <stdint.h>

#include <Broker.h>
#include <Escaper.h>
#include <rtems.h>

#include <Hal.h>

#include <drivers_config.h>
#include <system_spec.h>

#define Serial_SAMV71_RTEMS_FIFO_BUFFER_SIZE 256
#define Serial_SAMV71_RTEMS_RECV_BUFFER_SIZE 256
#define Serial_SAMV71_RTEMS_ENCODED_PACKET_MAX_SIZE 256
#define Serial_SAMV71_RTEMS_DECODED_PACKET_MAX_SIZE BROKER_BUFFER_SIZE

#define Serial_SAMV71_RTEMS_UART_TLS_SIZE 16384
#define Serial_SAMV71_RTEMS_STACK_SIZE                                         \
  (1024 > RTEMS_MINIMUM_STACK_SIZE ? 1024 : RTEMS_MINIMUM_STACK_SIZE)
#define Serial_SAMV71_RTEMS_TASK_BUFFER_SIZE                                   \
  (RTEMS_TASK_STORAGE_SIZE(Serial_SAMV71_RTEMS_STACK_SIZE +                    \
                               Serial_SAMV71_RTEMS_UART_TLS_SIZE,              \
                           RTEMS_FLOATING_POINT))

typedef struct {
  Serial_SamV71_Rtems_Device_T m_device;
  Hal_Uart m_hal_uart;
  Hal_Uart_Config m_hal_uart_config;
  uint8_t m_fifo_memory_block[Serial_SAMV71_RTEMS_FIFO_BUFFER_SIZE];
  uint8_t m_recv_buffer[Serial_SAMV71_RTEMS_RECV_BUFFER_SIZE];
  uint8_t m_encoded_packet_buffer[Serial_SAMV71_RTEMS_ENCODED_PACKET_MAX_SIZE];
  uint8_t m_decoded_packet_buffer[Serial_SAMV71_RTEMS_DECODED_PACKET_MAX_SIZE];
  Escaper m_escaper;
  enum SystemBus m_ip_device_bus_id;
  rtems_id m_task;
  /* TaskHandle_t m_task; */
  RTEMS_ALIGNED(RTEMS_TASK_STORAGE_ALIGNMENT)
  char m_task_buffer[Serial_SAMV71_RTEMS_TASK_BUFFER_SIZE];
  /* StackType_t m_task_stack_buffer[DRIVER_TASK_STACK_SIZE]; */
  Uart_RxHandler m_uart_rx_handler;
  /* SemaphoreHandle_t m_rx_semaphore; */
  /* StaticSemaphore_t m_rx_semaphore_buffer; */
  Uart_TxHandler m_uart_tx_handler;
  /* SemaphoreHandle_t m_tx_semaphore; */
  /* StaticSemaphore_t m_tx_semaphore_buffer; */
  Uart_ErrorHandler m_uart_error_handler;
  rtems_id m_rx_semaphore;
  rtems_id m_tx_semaphore;
} samv71_rtems_serial_private_data;

void Samv71RtemsSerialInit(
    void *private_data, const enum SystemBus bus_id,
    const enum SystemDevice device_id,
    const Serial_SamV71_Rtems_Conf_T *const device_configuration,
    const Serial_SamV71_Rtems_Conf_T *const remote_device_configuration);
void Samv71RtemsSerialPoll(void *private_data);
void Samv71RtemsSerialSend(void *private_data, const uint8_t *const data,
                           const size_t length);

#endif
