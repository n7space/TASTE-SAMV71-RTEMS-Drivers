#include "samv71_rtems_serial.h"

#include <assert.h>

#include <Escaper.h>
#include <EscaperInternal.h>

static inline void
SamV71RtemsSerialInit_uart_register(samv71_rtems_serial_private_data *self,
                                    Serial_SamV71_Rtems_Device_T deviceName) {
  switch (deviceName) {
  case uart0:
    self->m_hal_uart_config.id = Uart_Id_0;
    break;
  case uart1:
    self->m_hal_uart_config.id = Uart_Id_1;
    break;
  case uart2:
    self->m_hal_uart_config.id = Uart_Id_2;
    break;
  case uart3:
    self->m_hal_uart_config.id = Uart_Id_3;
    break;
  case uart4:
    self->m_hal_uart_config.id = Uart_Id_4;
    break;
  default:
    assert(false && "Not supported device name");
  }
}

static inline void
SamV71RtemsSerialInit_uart_data_bits(samv71_rtems_serial_private_data *self,
                                     Serial_SamV71_Rtems_Conf_T_bits bits) {
  (void)self;
  (void)bits;
  assert((bits == 8) && "Not supported number of data bits");
}

static inline void SamV71RtemsSerialInit_uart_parity(
     samv71_rtems_serial_private_data*self,
    bool useParity,
    Serial_SamV71_Rtems_Parity_T parity) {
  if (useParity) {
    switch (parity) {
    case Serial_SamV71_Rtems_Parity_T_odd:
      self->m_hal_uart_config.parity = Uart_Parity_Odd;
      break;
    case Serial_SamV71_Rtems_Parity_T_even:
      self->m_hal_uart_config.parity = Uart_Parity_Even;
      break;
    default:
      assert(false && "Not supported parity");
    }
  } else {
    self->m_hal_uart_config.parity = Uart_Parity_None;
  }
}

static inline void
SamV71RtemsSerialInit_uart_baudrate(samv71_rtems_serial_private_data *self,
                                    Serial_SamV71_Rtems_Baudrate_T speed) {

  switch (speed) {
  case Serial_SamV71_Rtems_Baudrate_T_b9600:
    self->m_hal_uart_config.baudrate = 9600;
    break;
  case Serial_SamV71_Rtems_Baudrate_T_b19200:
    self->m_hal_uart_config.baudrate = 19200;
    break;
  case Serial_SamV71_Rtems_Baudrate_T_b38400:
    self->m_hal_uart_config.baudrate = 38400;
    break;
  case Serial_SamV71_Rtems_Baudrate_T_b57600:
    self->m_hal_uart_config.baudrate = 57600;
    break;
  case Serial_SamV71_Rtems_Baudrate_T_b115200:
    self->m_hal_uart_config.baudrate = 115200;
    break;
  case Serial_SamV71_Rtems_Baudrate_T_b230400:
    self->m_hal_uart_config.baudrate = 230400;
    break;
  default:
    assert(false && "Not supported baudrate");
    break;
  }
}






static inline void SamV71RtemsSerialInit_uart_init(
    samv71_rtems_serial_private_data *const self,
    const Serial_SamV71_Rtems_Conf_T *const device_configuration) {
  SamV71RtemsSerialInit_uart_register(self, device_configuration->devname);
  SamV71RtemsSerialInit_uart_data_bits(self, device_configuration->bits);
  SamV71RtemsSerialInit_uart_parity(self, device_configuration->use_paritybit,
                                    device_configuration->parity);
  SamV71RtemsSerialInit_uart_baudrate(self, device_configuration->speed);
  Hal_uart_init(&self->m_hal_uart, self->m_hal_uart_config);
}

static void UartRxCallback(void *private_data) {
   samv71_rtems_serial_private_data*self =
    (samv71_rtems_serial_private_data *)private_data;
   rtems_semaphore_release(self->m_rx_semaphore); // TODO ISR
  //xSemaphoreGiveFromISR(self->m_rx_semaphore, NULL);
}


static inline void
SamV71RtemsSerialInit_rx_handler(samv71_rtems_serial_private_data *const self) {
  self->m_uart_rx_handler.characterCallback = UartRxCallback;
  self->m_uart_rx_handler.lengthCallback = UartRxCallback;
  self->m_uart_rx_handler.lengthArg = self;
  self->m_uart_rx_handler.characterArg = self;
  self->m_uart_rx_handler.targetCharacter = STOP_BYTE;
  self->m_uart_rx_handler.targetLength =
    Serial_SAMV71_RTEMS_RECV_BUFFER_SIZE / 2;
  rtems_semaphore_create(rtems_build_name('d', 'r', 'v', 'r'),
						 1,
						 RTEMS_BINARY_SEMAPHORE,
						 1,
						 &self->m_rx_semaphore);
  /* self->m_rx_semaphore = */
  /*     xSemaphoreCreateBinaryStatic(&self->m_rx_semaphore_buffer); */
  /* xSemaphoreGive(self->m_rx_semaphore); */
}

static ByteFifo *UartTxCallback(void *private_data) {
   samv71_rtems_serial_private_data*self =
      (samv71_rtems_serial_private_data *)private_data;

   rtems_semaphore_release(self->m_tx_semaphore); // TODO isr
  /* xSemaphoreGiveFromISR(self->m_tx_semaphore, NULL); */
  return NULL;
}


static inline void
SamV71RtemsSerialInit_tx_handler(samv71_rtems_serial_private_data *const self) {
  self->m_uart_tx_handler.callback = UartTxCallback;
  self->m_uart_tx_handler.arg = self;
  rtems_semaphore_create(rtems_build_name('d', 'r', 'v', 't'),
						 1,
						 RTEMS_BINARY_SEMAPHORE,
						 1,
						 &self->m_tx_semaphore);
  /* self->m_tx_semaphore = */
  /*     xSemaphoreCreateBinaryStatic(&self->m_tx_semaphore_buffer); */
  /* xSemaphoreGive(self->m_tx_semaphore); */
}




void Samv71RtemsSerialInit(
    void *private_data, const enum SystemBus bus_id,
    const enum SystemDevice device_id,
    const Serial_SamV71_Rtems_Conf_T *const device_configuration,
    const Serial_SamV71_Rtems_Conf_T *const remote_device_configuration) {
  (void)device_id;
  (void)remote_device_configuration;

  samv71_rtems_serial_private_data *self =
    (samv71_rtems_serial_private_data *)private_data;

  self->m_ip_device_bus_id = bus_id;

  SamV71RtemsSerialInit_uart_init(self, device_configuration);
  SamV71RtemsSerialInit_rx_handler(self);
  SamV71RtemsSerialInit_tx_handler(self);

  Escaper_init(&self->m_escaper, self->m_encoded_packet_buffer,
               Serial_SAMV71_RTEMS_ENCODED_PACKET_MAX_SIZE,
               self->m_decoded_packet_buffer,
               Serial_SAMV71_RTEMS_DECODED_PACKET_MAX_SIZE);

  // TODO, create rtems task

  /* rtems_task_config */

  /* rtems_task_construct */
}

static inline void SamV71RtemsSerialInterrupt_rx_enable(
     samv71_rtems_serial_private_data*const self) {
  self->m_hal_uart.uart.reg->ier =
      UART_IER_RXRDY_MASK | UART_IER_FRAME_MASK | UART_IER_OVRE_MASK;
}

static inline void SamV71RtemsSerialInterrupt_rx_disable(
     samv71_rtems_serial_private_data*const self) {
  self->m_hal_uart.uart.reg->idr =
      UART_IDR_RXRDY_MASK | UART_IDR_FRAME_MASK | UART_IDR_OVRE_MASK;
}


void Samv71RtemsSerialPoll(void *private_data) {
   samv71_rtems_serial_private_data*self =
     (samv71_rtems_serial_private_data *)private_data;
    size_t length = 0;

  Escaper_start_decoder(&self->m_escaper);
  rtems_semaphore_obtain(self->m_rx_semaphore, RTEMS_WAIT, RTEMS_NO_WAIT);
  Hal_uart_read(&self->m_hal_uart, self->m_fifo_memory_block,
                Serial_SAMV71_RTEMS_RECV_BUFFER_SIZE, self->m_uart_rx_handler);
  while (true) {
    /// Wait for data to arrive. Semaphore will be given
    rtems_semaphore_obtain(self->m_rx_semaphore, RTEMS_WAIT, RTEMS_NO_WAIT);

    length = ByteFifo_getCount(&self->m_hal_uart.rxFifo);

    for (size_t i = 0; i < length; i++) {
      SamV71RtemsSerialInterrupt_rx_disable(self);
      ByteFifo_pull(&self->m_hal_uart.rxFifo, &self->m_recv_buffer[i]);
      SamV71RtemsSerialInterrupt_rx_enable(self);
    }

    Escaper_decode_packet(&self->m_escaper, self->m_ip_device_bus_id,
                          self->m_recv_buffer, length, Broker_receive_packet);
  }

}

void Samv71RtemsSerialSend(void *private_data, const uint8_t *const data,
                           const size_t length) {
  samv71_rtems_serial_private_data *self =
    (samv71_rtems_serial_private_data *)private_data;
    size_t index = 0;
  size_t packetLength = 0;

  Escaper_start_encoder(&self->m_escaper);
  while (index < length) {
    packetLength =
        Escaper_encode_packet(&self->m_escaper, data, length, &index);
    rtems_semaphore_obtain(self->m_tx_semaphore, RTEMS_WAIT, RTEMS_NO_WAIT);
    Hal_uart_write(&self->m_hal_uart,
                   (uint8_t *const)&self->m_encoded_packet_buffer, packetLength,
                   &self->m_uart_tx_handler);

  }
}
