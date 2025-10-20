/**@file
 * This file is part of the TASTE SAMV71 RTEMS Drivers.
 *
 * @copyright 2025 N7 Space Sp. z o.o.
 *
 * Licensed under the ESA Public License (ESA-PL) Permissive (Type 3),
 * Version 2.4 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://essr.esa.int/license/list
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SAMV71_RTEMS_SERIAL_H
#define SAMV71_RTEMS_SERIAL_H

/**
 * @file     samv71_rtems_serial.h
 * @brief    Driver for TASTE for SAMV71 UART
 */

#include "samv71_rtems_serial_internal.h"

#include <rtems.h>

#include <stddef.h>
#include <stdint.h>

#include <Broker.h>
#include <Escaper.h>

#include <Hal.h>
#include <Uart/Uart.h>

#include <drivers_config.h>
#include <system_spec.h>

#define Serial_SAMV71_RTEMS_FIFO_BUFFER_SIZE 256
#define Serial_SAMV71_RTEMS_RECV_BUFFER_SIZE 256
#define Serial_SAMV71_RTEMS_ENCODED_PACKET_MAX_SIZE 256
#define Serial_SAMV71_RTEMS_DECODED_PACKET_MAX_SIZE BROKER_BUFFER_SIZE

#define Serial_SAMV71_RTEMS_UART_TLS_SIZE 16384
#define Serial_SAMV71_RTEMS_STACK_SIZE \
	(1024 > RTEMS_MINIMUM_STACK_SIZE ? 1024 : RTEMS_MINIMUM_STACK_SIZE)
#define Serial_SAMV71_RTEMS_TASK_BUFFER_SIZE                                \
	(RTEMS_TASK_STORAGE_SIZE(Serial_SAMV71_RTEMS_STACK_SIZE +           \
					 Serial_SAMV71_RTEMS_UART_TLS_SIZE, \
				 RTEMS_FLOATING_POINT))

/**
 * @brief Structure for samv71_rtems_serial driver internal data
 *
 * This structure is allocated by runtime and the pointer is passed to all
 * driver functions. The name of this structure shall match driver definition
 * from ocarina_components.aadl and has suffix '_private_data'.
 */
typedef struct {
	Serial_SamV71_Rtems_Device_T m_device;
	Samv71RtemsSerial_Uart m_hal_uart;
	Samv71RtemsSerial_Uart_Config m_hal_uart_config;
	uint8_t m_fifo_memory_block[Serial_SAMV71_RTEMS_FIFO_BUFFER_SIZE];
	uint8_t m_recv_buffer[Serial_SAMV71_RTEMS_RECV_BUFFER_SIZE];
	uint8_t m_encoded_packet_buffer
		[Serial_SAMV71_RTEMS_ENCODED_PACKET_MAX_SIZE];
	uint8_t m_decoded_packet_buffer
		[Serial_SAMV71_RTEMS_DECODED_PACKET_MAX_SIZE];
	Escaper m_escaper;
	enum SystemBus m_ip_device_bus_id;
	rtems_id m_task;
	RTEMS_ALIGNED(RTEMS_TASK_STORAGE_ALIGNMENT)
	char m_task_buffer[Serial_SAMV71_RTEMS_TASK_BUFFER_SIZE];
	Uart_RxHandler m_uart_rx_handler;
	Uart_TxHandler m_uart_tx_handler;
	Uart_ErrorHandler m_uart_error_handler;
	rtems_id m_rx_semaphore;
	rtems_id m_tx_semaphore;
} samv71_rtems_serial_private_data;

/**
 * @brief Function pointer definition for registering uart error callback.
 */
typedef void (*Samv71RtemsSerial_UserUartErrorCallback)(Uart_ErrorFlags,
							void *);

/**
 * @brief Function pointer definition for registering xdmad error callback.
 */
typedef void (*Samv71RtemsSerial_UserXdmadErrorCallback)(void *);

/**
 * @brief Initialize samv71_rtems_serial driver.
 *
 * Function is used by runtime to initialize the driver.
 *
 * @param private_data                  Driver private data, allocated by
 * runtime
 * @param bus_id                        Identifier of the bus, which is driver
 * @param device_id                     Identifier of the device
 * @param device_configuration          Configuration of device
 * @param remote_device_configuration   Configuration of remote device
 */
void Samv71RtemsSerialInit(
	void *private_data, const enum SystemBus bus_id,
	const enum SystemDevice device_id,
	const Serial_SamV71_Rtems_Conf_T *const device_configuration,
	const Serial_SamV71_Rtems_Conf_T *const remote_device_configuration);

/**
 * @brief Function which implements receiving data from remote partition.
 *
 * Functions works in separate thread, which is initialized by
 * SamV71SerialCcsdsInit
 *
 * @param private_data   Driver private data, allocated by runtime
 */
void Samv71RtemsSerialPoll(void *private_data);

/**
 * @brief Send data to remote partition.
 *
 * Function is used by runtime.
 *
 * @param private_data   Driver private data, allocated by runtime
 * @param data           The Buffer which data to send to connected remote
 * partition
 * @param length         The size of the buffer
 */
void Samv71RtemsSerialSend(void *private_data, const uint8_t *const data,
			   const size_t length);

/**
 * @brief Register callback for uart errors.
 *
 * @param callback       Pointer to the callback function.
 * @param arg            Argument which shall be passed when calling callback function.
 */
void Samv71RtemsSerialRegisterUserUartErrorCallback(
	Samv71RtemsSerial_UserUartErrorCallback callback, void *arg);

/**
 * @brief Register callback for xdmad errors.
 *
 * @param callback       Pointer to the callback function.
 * @param arg            Argument which shall be passed when calling callback function.
 */
void Samv71RtemsSerialRegisterUserXdmadErrorCallback(
	Samv71RtemsSerial_UserXdmadErrorCallback callback, void *arg);

#endif
