/**@file
 * This file is part of the TASTE SAMV71 RTEMS Drivers.
 *
 * @copyright 2025-2026 N7 Space Sp. z o.o.
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

#ifndef SAMV71_RTEMS_SERIAL_INTERNAL_H
#define SAMV71_RTEMS_SERIAL_INTERNAL_H

/**
 * @file     samv71_rtems_serial_internal.h
 * @brief    Internal structures for driver SAMV71 UART
 */

#include <Pio/Pio.h>
#include <Pmc/Pmc.h>
#include <Uart/Uart.h>

/// \brief UART configuration structure
typedef struct {
	Uart_Id id; //< Uart device identifier (0-4)
	Uart_Parity parity; //< used parity bits
	uint32_t baudrate; //< chosen baud rate
} Samv71RtemsSerial_UartConfig;

/// \brief UART structure
typedef struct {
	Uart uart;
	ByteFifo rxFifo;
	ByteFifo txFifo;
} Samv71RtemsSerial_Uart;

// \brief UART pin configuration
typedef struct {
	Pio_Port port;
	Pmc_PeripheralId peripheralId;
	uint32_t pinMask;
	Pio_Control control;
} Samv71RtemsSerial_UartPinConfig;

#endif
