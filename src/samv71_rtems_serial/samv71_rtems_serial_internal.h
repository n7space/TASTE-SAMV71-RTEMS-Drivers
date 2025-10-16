#ifndef SAMV71_RTEMS_SERIAL_INTERNAL_H
#define SAMV71_RTEMS_SERIAL_INTERNAL_H

#include <Uart/Uart.h>

#define DRIVER_TASK_STACK_SIZE 512
#define DRIVER_TASK_PRIORITY 1

/// \brief Uart configuration structure
typedef struct {
	Uart_Id id; /// Uart device identifier (0-4)
	Uart_Parity parity; //< used parity bits
	uint32_t baudrate; //< chosen baud rate
} Samv71RtemsSerial_Uart_Config;

/// \brief uart structure
typedef struct {
	Uart uart;
	ByteFifo rxFifo;
	ByteFifo txFifo;
} Samv71RtemsSerial_Uart;

#endif
