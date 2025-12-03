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

#ifndef SAMV71_RTEMS_CAN_H
#define SAMV71_RTEMS_CAN_H

/**
 * @file     samv71_can_generic.h
 * @brief    Driver for TASTE for SAMV71 CAN
 */

#include <rtems.h>

#include <drivers_config.h>
#include <system_spec.h>
#include <Mcan/Mcan.h>
#include <Pio/Pio.h>

#include <Escaper.h>
#include <Broker.h>

#define MSGRAM_SIZE 512
#define MSGRAM_BYTE_SIZE (MSGRAM_SIZE * sizeof(uint32_t))
#define MSGRAM_BYTE_SIZE_EXPONENT 11
#define MSGRAM_STDID_FILTER_OFFSET 0
#define MSGRAM_STDID_FILTER_SIZE 0
#define MSGRAM_EXTID_FILTER_OFFSET 0
#define MSGRAM_EXTID_FILTER_SIZE 0
#define MSGRAM_RXFIFO0_OFFSET 0
#define MSGRAM_RXFIFO0_SIZE 384
#define MSGRAM_RXFIFO1_OFFSET 384
#define MSGRAM_RXFIFO1_SIZE 0
#define MSGRAM_RXBUFFER_OFFSET 384
#define MSGRAM_RXBUFFER_SIZE 0
#define MSGRAM_TXEVENTINFO_OFFSET 384
#define MSGRAM_TXEVENTINFO_SIZE 0
#define MSGRAM_TXBUFFER_OFFSET 384
#define MSGRAM_TXBUFFER_SIZE 128

#define Can_SAMV71_RTEMS_TLS_SIZE 16384
#define Can_SAMV71_RTEMS_STACK_SIZE \
	(1024 > RTEMS_MINIMUM_STACK_SIZE ? 1024 : RTEMS_MINIMUM_STACK_SIZE)
#define Can_SAMV71_RTEMS_TASK_BUFFER_SIZE                           \
	(RTEMS_TASK_STORAGE_SIZE(Can_SAMV71_RTEMS_STACK_SIZE +      \
					 Can_SAMV71_RTEMS_TLS_SIZE, \
				 RTEMS_FLOATING_POINT))

/**
 * @brief Structure for samv71_can_generic driver internal data
 *
 * This structure is allocated by runtime and the pointer is passed to all
 * driver functions. The name of this structure shall match driver definition
 * from ocarina_components.aadl and has suffix '_private_data'.
 */
typedef struct __attribute__((aligned(4096))) {
	uint32_t msgRam[MSGRAM_SIZE];
	enum SystemBus m_bus_id;
	const CAN_Samv71_Rtems_Conf_T *m_config;
	Mcan mcan;
	Pio pioCanTx;
	rtems_id m_task;
	RTEMS_ALIGNED(RTEMS_TASK_STORAGE_ALIGNMENT)
	char m_task_buffer[Can_SAMV71_RTEMS_TASK_BUFFER_SIZE];
	rtems_id m_rx_semaphore;
	Escaper m_escaper;
	uint8_t m_tx_buffer[8];
	uint8_t m_rx_buffer[8];
	uint8_t m_value_buffer[BROKER_BUFFER_SIZE];
} samv71_can_generic_private_data;

/**
 * @brief Initialize samv71_can_generic driver.
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
void SamV71RtemsCanInit(
	void *private_data, const enum SystemBus bus_id,
	const enum SystemDevice device_id,
	const CAN_Samv71_Rtems_Conf_T *const device_configuration,
	const CAN_Samv71_Rtems_Conf_T *const remote_device_configuration);

/**
 * @brief Function which implements receiving data from remote partition.
 *
 * Functions works in separate thread, which is initialized by SamV71RtemsCanInit
 *
 * @param private_data   Driver private data, allocated by runtime
 */
void SamV71RtemsCanPoll(void *private_data);

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
void SamV71RtemsCanSend(void *private_data, const uint8_t *const data,
			const size_t length);

#endif
