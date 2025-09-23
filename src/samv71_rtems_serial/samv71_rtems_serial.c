#include "samv71_serial_ccsds.h"

static inline void
SamV71SerialCcsdsInit_uart_register(samv71_serial_ccsds_private_data *self,
                                    Serial_CCSDS_SamV71_Device_T deviceName) {
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


static inline void SamV71SerialCcsdsInit_uart_init(
    samv71_serial_ccsds_private_data *const self,
    const Serial_CCSDS_SamV71_Conf_T *const device_configuration) {
  SamV71SerialCcsdsInit_uart_register(self, device_configuration->devname);
  SamV71SerialCcsdsInit_uart_data_bits(self, device_configuration->bits);
  SamV71SerialCcsdsInit_uart_parity(self, device_configuration->use_paritybit,
                                    device_configuration->parity);
  SamV71SerialCcsdsInit_uart_baudrate(self, device_configuration->speed);
  Hal_uart_init(&self->m_hal_uart, self->m_hal_uart_config);
}


void SamV71SerialCcsdsInit(
    void *private_data, const enum SystemBus bus_id,
    const enum SystemDevice device_id,
    const Serial_CCSDS_SamV71_Conf_T *const device_configuration,
    const Serial_CCSDS_SamV71_Conf_T *const remote_device_configuration) {
  (void)device_id;
  (void)remote_device_configuration;

  samv71_serial_ccsds_private_data *self =
    (samv71_serial_ccsds_private_data *)private_data;

  self->m_ip_device_bus_id = bus_id;

  SamV71SerialCcsdsInit_uart_init(self, device_configuration);
}

void SamV71SerialCcsdsPoll(void *private_data) {
  samv71_serial_ccsds_private_data *self =
      (samv71_serial_ccsds_private_data *)private_data;
}

void SamV71SerialCcsdsSend(void *private_data, const uint8_t *const data,
                           const size_t length) {
  samv71_serial_ccsds_private_data *self =
      (samv71_serial_ccsds_private_data *)private_data;
}
