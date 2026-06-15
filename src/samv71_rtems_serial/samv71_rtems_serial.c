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

#include "samv71_rtems_serial.h"

#include <stdlib.h>

#include <Broker.h>
#include <assert.h>
#include <rtems.h>

#include <Hal.h>

#include <Escaper.h>
#include <EscaperInternal.h>
#include <Nvic/Nvic.h>
#include <Pio/Pio.h>
#include <Pmc/Pmc.h>
#include <SamV71Core/SamV71Core.h>
#include <Scb/Scb.h>
#include <Uart/Uart.h>
#include <Xdmac/xdmad.h>

static Samv71RtemsSerial_UserUartErrorCallback
	Samv71RtemsSerial_user_uart_error_callback = NULL;
static void *Samv71RtemsSerial_user_uart_error_callback_arg = NULL;

static Samv71RtemsSerial_UserXdmadErrorCallback
	Samv71RtemsSerial_user_xdmad_error_callback = NULL;
static void *Samv71RtemsSerial_user_xdmad_error_callback_arg = NULL;

// global variable required by xdmad.c
rtems_id xdmad_lock;

static sXdmad xdmad;
static Uart *uart0handle;
static Uart *uart1handle;
static Uart *uart2handle;
static Uart *uart3handle;
static Uart *uart4handle;

// To make sure UART is handled with highest priority, set the IRQ priority to 0
#define UART_INTERRUPT_PRIORITY 0
#define UART_XDMAC_INTERRUPT_PRIORITY UART_INTERRUPT_PRIORITY

#define UART_RX_EVENT RTEMS_EVENT_0

#define XDMAD_NO_POLLING 0

void UART0_Handler(void)
{
	if (uart0handle != NULL)
		Uart_handleInterrupt(uart0handle, NULL);
}

void UART1_Handler(void)
{
	if (uart1handle != NULL)
		Uart_handleInterrupt(uart1handle, NULL);
}

void UART2_Handler(void)
{
	if (uart2handle != NULL)
		Uart_handleInterrupt(uart2handle, NULL);
}

void UART3_Handler(void)
{
	if (uart3handle != NULL)
		Uart_handleInterrupt(uart3handle, NULL);
}

void UART4_Handler(void)
{
	if (uart4handle != NULL)
		Uart_handleInterrupt(uart4handle, NULL);
}

void XDMAC_Handler(void)
{
	XDMAD_Handler(&xdmad);
}

inline static void initDMALock()
{
	const rtems_status_code status_code =
		rtems_semaphore_create(SamV71Core_GenerateNewSemaphoreName(),
				       1, // Initial value, unlocked
				       RTEMS_BINARY_SEMAPHORE,
				       0, // Priority ceiling
				       &xdmad_lock);

	assert(status_code == RTEMS_SUCCESSFUL);
}

static inline void initDMA(void)
{
	SamV71Core_EnablePeripheralClock(Pmc_PeripheralId_Xdmac);

	Nvic_clearInterruptPending(Nvic_Irq_Xdmac);
	Nvic_setInterruptPriority(Nvic_Irq_Xdmac,
				  UART_XDMAC_INTERRUPT_PRIORITY);

	XDMAD_Initialize(&xdmad, XDMAD_NO_POLLING);

	SamV71Core_InterruptSubscribe(Nvic_Irq_Xdmac, "xdmac",
				      (rtems_interrupt_handler)&XDMAC_Handler,
				      NULL);
}

static inline void initUartIrq(Nvic_Irq irqId, char const *const info,
			       rtems_interrupt_handler handler)
{
	Nvic_clearInterruptPending(irqId);
	Nvic_setInterruptPriority(irqId, UART_INTERRUPT_PRIORITY);
	SamV71Core_InterruptSubscribe(irqId, info, handler, NULL);
}

static void uartLowLevelInit()
{
	static bool uartInitialized = false;
	if (!uartInitialized) {
		initDMALock();
		initUartIrq(Nvic_Irq_Uart0, "uart0",
			    (rtems_interrupt_handler)UART0_Handler);
		initUartIrq(Nvic_Irq_Uart1, "uart1",
			    (rtems_interrupt_handler)UART1_Handler);
		initUartIrq(Nvic_Irq_Uart2, "uart2",
			    (rtems_interrupt_handler)UART2_Handler);
		initUartIrq(Nvic_Irq_Uart3, "uart3",
			    (rtems_interrupt_handler)UART3_Handler);
		initUartIrq(Nvic_Irq_Uart4, "uart4",
			    (rtems_interrupt_handler)UART4_Handler);
		initDMA();
		uartInitialized = true;
	}
}

static void uartDMAHandler(uint32_t xdmacChannel, void *args)
{
	XDMAD_FreeChannel(&xdmad, xdmacChannel);
	const Uart_TxHandler *const uartTxHandler = (Uart_TxHandler *)args;
	uartTxHandler->callback(uartTxHandler->arg);
}

static void uartErrorHandler(const Uart_ErrorFlags *errorFlags, void *arg)
{
	(void)arg;
	if (Samv71RtemsSerial_user_uart_error_callback != NULL) {
		Samv71RtemsSerial_user_uart_error_callback(
			*errorFlags,
			Samv71RtemsSerial_user_uart_error_callback_arg);
	}
}

/*
 * Available UART pins configurations for 144-pin package SAMV71:
 * UART0:
 *   - URXD0 - PA9 (PIO peripheral A)
 *   - UTXD0 - PA10 (PIO peripheral A)
 * UART1:
 *   - URXD1 - PA5 (PIO peripheral C)
 *   - UTXD1 - PA4, PA6 (PIO peripheral C), PD26 (PIO peripheral D)
 * UART2:
 *   - URXD2 - PD25 (PIO peripheral C)
 *   - UTXD2 - PD26 (PIO peripheral C)
 * UART3:
 *   - URXD3 - PD28 (PIO peripheral A)
 *   - UTXD3 - PD30 (PIO peripheral A), PD31 (PIO peripheral A)
 * UART4:
 *   - URXD4 - PD18 (PIO peripheral C)
 *   - UTXD4 - PD3, PD19 (PIO peripheral C)
 */

static inline Samv71RtemsSerial_UartPinConfig
makeUartPinConfig(Pio_Port port, Pmc_PeripheralId peripheralId,
		  uint32_t pinMask, Pio_Control control)
{
	const Samv71RtemsSerial_UartPinConfig pinConfig = {
		.port = port,
		.peripheralId = peripheralId,
		.pinMask = pinMask,
		.control = control,
	};

	return pinConfig;
}

static inline Uart_Id
getUartId(const Serial_SamV71_Rtems_Device_T *const device)
{
	switch (device->kind) {
	case uart0_PRESENT:
		return Uart_Id_0;
	case uart1_PRESENT:
		return Uart_Id_1;
	case uart2_PRESENT:
		return Uart_Id_2;
	case uart3_PRESENT:
		return Uart_Id_3;
	case uart4_PRESENT:
		return Uart_Id_4;
	default:
		// If this branch is hit, then the user provided configuration is invalid,
		// or something went very wrong and the program will enter invalid state, so
		// the safest course of action is to assert and abort (if the asserts are
		// disabled).
		assert(false && "Unsupported UART");
		abort();
	}
}

static inline Nvic_Irq
getUartIrqId(const Serial_SamV71_Rtems_Device_T *const device)
{
	switch (device->kind) {
	case uart0_PRESENT:
		return Nvic_Irq_Uart0;
	case uart1_PRESENT:
		return Nvic_Irq_Uart1;
	case uart2_PRESENT:
		return Nvic_Irq_Uart2;
	case uart3_PRESENT:
		return Nvic_Irq_Uart3;
	case uart4_PRESENT:
		return Nvic_Irq_Uart4;
	default:
		// If this branch is hit, then the user provided configuration is invalid,
		// or something went very wrong and the program will enter invalid state, so
		// the safest course of action is to assert and abort (if the asserts are
		// disabled).
		assert(false && "Unsupported UART");
		abort();
	}
}

static Samv71RtemsSerial_UartPinConfig
getUartTxPinConfig(const Serial_SamV71_Rtems_Device_T *const device)
{
	switch (device->kind) {
	case uart0_PRESENT:
		return makeUartPinConfig(Pio_Port_A, Pmc_PeripheralId_PioA,
					 PIO_PIN_10, Pio_Control_PeripheralA);
	case uart1_PRESENT:
		switch (device->u.uart1.tx) {
		case Serial_SamV71_Rtems_Device_T_uart1_tx_pa4:
			return makeUartPinConfig(Pio_Port_A,
						 Pmc_PeripheralId_PioA,
						 PIO_PIN_4,
						 Pio_Control_PeripheralC);
		case Serial_SamV71_Rtems_Device_T_uart1_tx_pa6:
			return makeUartPinConfig(Pio_Port_A,
						 Pmc_PeripheralId_PioA,
						 PIO_PIN_6,
						 Pio_Control_PeripheralC);
		case Serial_SamV71_Rtems_Device_T_uart1_tx_pd26:
			return makeUartPinConfig(Pio_Port_D,
						 Pmc_PeripheralId_PioD,
						 PIO_PIN_26,
						 Pio_Control_PeripheralD);
		default:
			// If this branch is hit, then the user provided configuration is invalid,
			// or something went very wrong and the program will enter invalid state,
			// so the safest course of action is to assert and abort (if the asserts
			// are disabled).
			assert(false && "Unsupported UART1 TX pin");
			abort();
		}
	case uart2_PRESENT:
		return makeUartPinConfig(Pio_Port_D, Pmc_PeripheralId_PioD,
					 PIO_PIN_26, Pio_Control_PeripheralC);
	case uart3_PRESENT:
		switch (device->u.uart3.tx) {
		case Serial_SamV71_Rtems_Device_T_uart3_tx_pd30:
			return makeUartPinConfig(Pio_Port_D,
						 Pmc_PeripheralId_PioD,
						 PIO_PIN_30,
						 Pio_Control_PeripheralA);
		case Serial_SamV71_Rtems_Device_T_uart3_tx_pd31:
			return makeUartPinConfig(Pio_Port_D,
						 Pmc_PeripheralId_PioD,
						 PIO_PIN_31,
						 Pio_Control_PeripheralA);
		default:
			// If this branch is hit, then the user provided configuration is invalid,
			// or something went very wrong and the program will enter invalid state,
			// so the safest course of action is to assert and abort (if the asserts
			// are disabled).
			assert(false && "Unsupported UART3 TX pin");
			abort();
		}
	case uart4_PRESENT:
		switch (device->u.uart4.tx) {
		case Serial_SamV71_Rtems_Device_T_uart4_tx_pd3:
			return makeUartPinConfig(Pio_Port_D,
						 Pmc_PeripheralId_PioD,
						 PIO_PIN_3,
						 Pio_Control_PeripheralC);
		case Serial_SamV71_Rtems_Device_T_uart4_tx_pd19:
			return makeUartPinConfig(Pio_Port_D,
						 Pmc_PeripheralId_PioD,
						 PIO_PIN_19,
						 Pio_Control_PeripheralC);
		default:
			// If this branch is hit, then the user provided configuration is invalid,
			// or something went very wrong and the program will enter invalid state,
			// so the safest course of action is to assert and abort (if the asserts
			// are disabled).
			assert(false && "Unsupported UART4 TX pin");
			abort();
		}
	default:
		// If this branch is hit, then the user provided configuration is invalid,
		// or something went very wrong and the program will enter invalid state,
		// so the safest course of action is to assert and abort (if the asserts
		// are disabled).
		assert(false && "Unsupported UART");
		abort();
	}
}

static Samv71RtemsSerial_UartPinConfig
getUartRxPinConfig(const Serial_SamV71_Rtems_Device_T *const device)
{
	switch (device->kind) {
	case uart0_PRESENT:
		return makeUartPinConfig(Pio_Port_A, Pmc_PeripheralId_PioA,
					 PIO_PIN_9, Pio_Control_PeripheralA);
	case uart1_PRESENT:
		return makeUartPinConfig(Pio_Port_A, Pmc_PeripheralId_PioA,
					 PIO_PIN_5, Pio_Control_PeripheralC);
	case uart2_PRESENT:
		return makeUartPinConfig(Pio_Port_D, Pmc_PeripheralId_PioD,
					 PIO_PIN_25, Pio_Control_PeripheralC);
	case uart3_PRESENT:
		return makeUartPinConfig(Pio_Port_D, Pmc_PeripheralId_PioD,
					 PIO_PIN_28, Pio_Control_PeripheralA);
	case uart4_PRESENT:
		return makeUartPinConfig(Pio_Port_D, Pmc_PeripheralId_PioD,
					 PIO_PIN_18, Pio_Control_PeripheralC);
	default:
		// If this branch is hit, then the user provided configuration is invalid,
		// or something went very wrong and the program will enter invalid state,
		// so the safest course of action is to assert and abort (if the asserts
		// are disabled).
		assert(false && "Unsupported UART");
		abort();
	}
}

static inline void
initUartPin(const Samv71RtemsSerial_UartPinConfig *const pinConfig,
	    Pio_Direction direction)
{
	Pio_Port_Config pioConfig = {.pinsConfig =
                                   {
                                       .pull = Pio_Pull_Up,
                                       .filter = Pio_Filter_None,
                                       .isMultiDriveEnabled = false,
                                       .isSchmittTriggerDisabled = false,
                                       .irq = Pio_Irq_None,
                                       .direction = direction,
                                       .control = pinConfig->control,
                                   },
                               .debounceFilterDiv = 0,
                               .pins = pinConfig->pinMask};
	Pio pio;
	ErrorCode errorCode = 0;

	Pio_init(pinConfig->port, &pio, &errorCode);
	Pio_setPortConfig(&pio, &pioConfig, &errorCode);
}

static inline Pmc_PeripheralId getUartPeripheralId(Uart_Id id)
{
	switch (id) {
	case Uart_Id_0:
		return Pmc_PeripheralId_Uart0;
	case Uart_Id_1:
		return Pmc_PeripheralId_Uart1;
	case Uart_Id_2:
		return Pmc_PeripheralId_Uart2;
	case Uart_Id_3:
		return Pmc_PeripheralId_Uart3;
	case Uart_Id_4:
		return Pmc_PeripheralId_Uart4;
	default:
		// If this branch is hit, then the user provided configuration is invalid,
		// or something went very wrong and the program will enter invalid state,
		// so the safest course of action is to assert and abort (if the asserts
		// are disabled).
		assert(false && "Unsupported UART");
		abort();
	}
}

static inline void initUartPio(const Serial_SamV71_Rtems_Device_T *const device)
{
	const Samv71RtemsSerial_UartPinConfig txPinConfig =
		getUartTxPinConfig(device);
	const Samv71RtemsSerial_UartPinConfig rxPinConfig =
		getUartRxPinConfig(device);

	initUartPin(&txPinConfig, Pio_Direction_Output);
	initUartPin(&rxPinConfig, Pio_Direction_Input);
}

inline static void initUartPmc(const Serial_SamV71_Rtems_Device_T *const device)
{
	const Samv71RtemsSerial_UartPinConfig txPinConfig =
		getUartTxPinConfig(device);
	const Samv71RtemsSerial_UartPinConfig rxPinConfig =
		getUartRxPinConfig(device);

	SamV71Core_EnablePeripheralClock(txPinConfig.peripheralId);
	if (rxPinConfig.peripheralId != txPinConfig.peripheralId) {
		SamV71Core_EnablePeripheralClock(rxPinConfig.peripheralId);
	}
	SamV71Core_EnablePeripheralClock(
		getUartPeripheralId(getUartId(device)));
}

inline static void initUartHandle(Uart *uart, Uart_Id id)
{
	switch (id) {
	case Uart_Id_0:
		uart0handle = uart;
		break;
	case Uart_Id_1:
		uart1handle = uart;
		break;
	case Uart_Id_2:
		uart2handle = uart;
		break;
	case Uart_Id_3:
		uart3handle = uart;
		break;
	case Uart_Id_4:
		uart4handle = uart;
		break;
	default:
		// If this branch is hit, then the user provided configuration is invalid,
		// or something went very wrong and the program will enter invalid state,
		// so the safest course of action is to assert and abort (if the asserts
		// are disabled).
		assert(false && "Unsupported UART");
		abort();
	}
}

static void initUartHardware(Samv71RtemsSerial_Uart *const halUart,
			     Samv71RtemsSerial_UartConfig halUartConfig,
			     const Serial_SamV71_Rtems_Device_T *const device)
{
	uartLowLevelInit();

	assert(halUartConfig.id <= Uart_Id_4);
	assert((halUartConfig.parity <= Uart_Parity_Odd) ||
	       (halUartConfig.parity == Uart_Parity_None));

	initUartPmc(device);
	initUartPio(device);
	initUartHandle(&halUart->uart, halUartConfig.id);

	Uart_init(halUartConfig.id, &halUart->uart);
	Uart_reset(&halUart->uart);

	const Uart_Config config = {
		.isTxEnabled = true,
		.isRxEnabled = true,
		.isTestModeEnabled = false,
		.parity = halUartConfig.parity,
		.baudRate = halUartConfig.baudrate,
		.baudRateClkSrc = Uart_BaudRateClk_PeripheralCk,
		.baudRateClkFreq = SamV71Core_GetMainClockFrequency()
	};
	Uart_setConfig(&halUart->uart, &config);
}

static void initUartTxDMACHannel(Samv71RtemsSerial_Uart *const halUart,
				 const uint8_t *const buffer,
				 const uint16_t length,
				 const Uart_TxHandler *const txHandler,
				 uint32_t channelNumber)
{
	const eXdmadRC prepareResult =
		XDMAD_PrepareChannel(&xdmad, channelNumber);
	if (prepareResult != XDMAD_OK) {
		if (Samv71RtemsSerial_user_xdmad_error_callback != NULL) {
			Samv71RtemsSerial_user_xdmad_error_callback(
				Samv71RtemsSerial_user_xdmad_error_callback_arg);
		}
		return;
	}

	//< Get Uart Tx peripheral xdmac id
	const uint32_t periphID = xdmad.XdmaChannels[channelNumber].bDstTxIfID
				  << XDMAC_CC_PERID_Pos;
	sXdmadCfg config = {
		// uBlock max length is equal to uart write max data length. Thus one
		// uBlock can be used.
		.mbr_ubc = length,
		// Data buffer as source addres
		.mbr_sa = (uint32_t)buffer,
		// Uart tx holding register as a destination address
		.mbr_da = (uint32_t)&halUart->uart.registers->thr,
		// Config memory to peripheral transfer. Increment source buffer address.
		// Keep destination address buffer fixed.
		.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN | XDMAC_CC_MBSIZE_SINGLE |
			   XDMAC_CC_DSYNC_MEM2PER |
			   XDMAC_CC_SWREQ_HWR_CONNECTED |
			   XDMAC_CC_MEMSET_NORMAL_MODE | XDMAC_CC_DWIDTH_BYTE |
			   XDMAC_CC_SIF_AHB_IF1 | XDMAC_CC_DIF_AHB_IF1 |
			   XDMAC_CC_SAM_INCREMENTED_AM | XDMAC_CC_DAM_FIXED_AM |
			   periphID,
		// Disable data striding.
		.mbr_bc = 0,
		.mbr_ds = 0,
		.mbr_sus = 0,
		.mbr_dus = 0,
	};

	const eXdmadRC configureResult = XDMAD_ConfigureTransfer(
		&xdmad, channelNumber, &config, 0, 0,
		XDMAC_CIE_BIE | XDMAC_CIE_RBIE | XDMAC_CIE_WBIE |
			XDMAC_CIE_ROIE);
	if (configureResult != XDMAD_OK) {
		if (Samv71RtemsSerial_user_xdmad_error_callback != NULL) {
			Samv71RtemsSerial_user_xdmad_error_callback(
				Samv71RtemsSerial_user_xdmad_error_callback_arg);
		}
		return;
	}
	const eXdmadRC callbackResult = XDMAD_SetCallback(
		&xdmad, channelNumber, uartDMAHandler, (void *)txHandler);

	if (callbackResult != XDMAD_OK) {
		if (Samv71RtemsSerial_user_xdmad_error_callback != NULL) {
			Samv71RtemsSerial_user_xdmad_error_callback(
				Samv71RtemsSerial_user_xdmad_error_callback_arg);
		}
	}
}

/** \brief Asynchronously sends bytes over uart.
 *
 * \param [in] halUart Hal_Uart structure contains uart device descriptor and
 *                     relevant fifos.
 * \param [in] buffer array containing bytes to send
 * \param [in] length length of array of bytes
 * \param [in] txHandler pointer to the handler called after successful array
 *                       transmission
 */
static void uartWrite(Samv71RtemsSerial_Uart *const halUart,
		      const uint8_t *const buffer, const uint16_t length,
		      const Uart_TxHandler *const txHandler)
{
	const uint32_t channelNumber =
		XDMAD_AllocateChannel(&xdmad, XDMAD_TRANSFER_MEMORY,
				      getUartPeripheralId(halUart->uart.id));

	if (channelNumber <
	    (xdmad.pXdmacs->XDMAC_GTYPE & XDMAC_GTYPE_NB_CH_Msk)) {
		initUartTxDMACHannel(halUart, buffer, length, txHandler,
				     channelNumber);
		const eXdmadRC startResult =
			XDMAD_StartTransfer(&xdmad, channelNumber);

		if ((startResult != XDMAD_OK) &&
		    (Samv71RtemsSerial_user_xdmad_error_callback != NULL)) {
			Samv71RtemsSerial_user_xdmad_error_callback(
				Samv71RtemsSerial_user_xdmad_error_callback_arg);
		}
	} else if (Samv71RtemsSerial_user_xdmad_error_callback != NULL) {
		Samv71RtemsSerial_user_xdmad_error_callback(
			Samv71RtemsSerial_user_xdmad_error_callback_arg);
	}
}

/** \brief Asynchronously receives bytes over uart.
 *
 * \param [in] halUart Hal_Uart structure contains uart device descriptor and
 *                     relevant fifos.
 * \param [in] buffer array where received bytes will be stored
 * \param [in] length length of array of bytes
 * \param [in] rxHandler handler called after successful array reception or
 *                       after matching character was found
 */
static void uartRead(Samv71RtemsSerial_Uart *const halUart,
		     uint8_t *const buffer, const uint16_t length,
		     const Uart_RxHandler rxHandler)
{
	const Uart_ErrorHandler errorHandler = { .callback = uartErrorHandler,
						 .arg = halUart };
	ByteFifo_init(&halUart->rxFifo, buffer, length);
	Uart_registerErrorHandler(&halUart->uart, errorHandler);
	Uart_readAsync(&halUart->uart, &halUart->rxFifo, rxHandler);
}

static inline void registerUartId(samv71_rtems_serial_private_data *self,
				  Serial_SamV71_Rtems_Device_T deviceName)
{
	self->m_hal_uart_config.id = getUartId(&deviceName);
}

static inline void initUartParity(samv71_rtems_serial_private_data *self,
				  Serial_SamV71_Rtems_Parity_T parity)
{
	switch (parity) {
	case Serial_SamV71_Rtems_Parity_T_odd:
		self->m_hal_uart_config.parity = Uart_Parity_Odd;
		break;
	case Serial_SamV71_Rtems_Parity_T_even:
		self->m_hal_uart_config.parity = Uart_Parity_Even;
		break;
	case Serial_SamV71_Rtems_Parity_T_none:
		self->m_hal_uart_config.parity = Uart_Parity_None;
		break;
	default:
		// If this branch is hit, then the user provided configuration is invalid,
		// or something went very wrong and the program will enter invalid state,
		// so the safest course of action is to assert and abort (if the asserts
		// are disabled).
		assert(false && "Not supported parity");
		abort();
	}
}

static inline void initUartBaudrate(samv71_rtems_serial_private_data *self,
				    Serial_SamV71_Rtems_Baudrate_T speed)
{
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
		// If this branch is hit, then the user provided configuration is invalid,
		// or something went very wrong and the program will enter invalid state,
		// so the safest course of action is to assert and abort (if the asserts
		// are disabled).
		assert(false && "Not supported baudrate");
		abort();
	}
}

static inline void
initUart(samv71_rtems_serial_private_data *const self,
	 const Serial_SamV71_Rtems_Conf_T *const device_configuration)
{
	self->m_device = device_configuration->devname;
	registerUartId(self, self->m_device);
	initUartParity(self, device_configuration->parity);
	initUartBaudrate(self, device_configuration->speed);
	initUartHardware(&self->m_hal_uart, self->m_hal_uart_config,
			 &self->m_device);
}

static void uartRxCallback(void *private_data)
{
	const samv71_rtems_serial_private_data *const self =
		(samv71_rtems_serial_private_data *)private_data;
	rtems_event_send(self->m_task, UART_RX_EVENT);
}

static void initUartRxHandler(samv71_rtems_serial_private_data *const self)
{
	self->m_uart_rx_handler.lengthCallback = uartRxCallback;
	self->m_uart_rx_handler.lengthArg = self;
	self->m_uart_rx_handler.characterCallback = uartRxCallback;
	self->m_uart_rx_handler.characterArg = self;
	if (self->m_raw_mode) {
		self->m_uart_rx_handler.targetCharacter = 0xC0;
		self->m_uart_rx_handler.targetLength =
			Serial_SAMV71_RTEMS_RECV_BUFFER_SIZE / 2;
	} else {
		self->m_uart_rx_handler.characterCallback = uartRxCallback;
		self->m_uart_rx_handler.characterArg = self;
		self->m_uart_rx_handler.targetCharacter = STOP_BYTE;
		self->m_uart_rx_handler.targetLength =
			Serial_SAMV71_RTEMS_RECV_BUFFER_SIZE / 2;
	}
}

static ByteFifo *uartTxCallback(void *private_data)
{
	const samv71_rtems_serial_private_data *const self =
		(samv71_rtems_serial_private_data *)private_data;

	const rtems_status_code releaseResult =
		rtems_semaphore_release(self->m_tx_semaphore);
	assert(releaseResult == RTEMS_SUCCESSFUL);
	return NULL;
}

static void initUartTxHandler(samv71_rtems_serial_private_data *const self)
{
	self->m_uart_tx_handler.callback = uartTxCallback;
	self->m_uart_tx_handler.arg = self;

	const rtems_status_code status_code =
		rtems_semaphore_create(SamV71Core_GenerateNewSemaphoreName(),
				       1, // Initial value, unlocked
				       RTEMS_SIMPLE_BINARY_SEMAPHORE,
				       0, // Priority ceiling
				       &self->m_tx_semaphore);

	assert(status_code == RTEMS_SUCCESSFUL);
}

static inline uint32_t
enterCriticalSection(samv71_rtems_serial_private_data *const self)
{
	const uint32_t mask = self->m_hal_uart.uart.registers->imr;
	self->m_hal_uart.uart.registers->idr = mask;
	MEMORY_SYNC_BARRIER();
	return mask;
}

static inline void
exitCriticalSection(samv71_rtems_serial_private_data *const self,
		    const uint32_t state)
{
	MEMORY_SYNC_BARRIER();
	self->m_hal_uart.uart.registers->ier = state;
}

void Samv71RtemsSerialInit(
	void *private_data, const enum SystemBus bus_id,
	const enum SystemDevice device_id,
	const Serial_SamV71_Rtems_Conf_T *const device_configuration,
	const Serial_SamV71_Rtems_Conf_T *const remote_device_configuration)
{
	(void)device_id;
	(void)remote_device_configuration;

	samv71_rtems_serial_private_data *const self =
		(samv71_rtems_serial_private_data *)private_data;

	self->m_ip_device_bus_id = bus_id;
	self->m_raw_mode = device_configuration->transmit_mode ==
			   Serial_SamV71_Rtems_Transmit_Mode_T_raw_single_byte;

	initUart(self, device_configuration);
	initUartRxHandler(self);
	initUartTxHandler(self);

	Escaper_init(&self->m_escaper, self->m_encoded_packet_buffer,
		     Serial_SAMV71_RTEMS_ENCODED_PACKET_MAX_SIZE,
		     self->m_decoded_packet_buffer,
		     Serial_SAMV71_RTEMS_DECODED_PACKET_MAX_SIZE);

	const rtems_task_config taskConfig = {
		.name = SamV71Core_GenerateNewTaskName(),
		.initial_priority = 1,
		.storage_area = self->m_task_buffer,
		.storage_size = Serial_SAMV71_RTEMS_TASK_BUFFER_SIZE,
		.maximum_thread_local_storage_size =
			Serial_SAMV71_RTEMS_UART_TLS_SIZE,
		.storage_free = NULL,
		.initial_modes = RTEMS_PREEMPT,
		.attributes = RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT
	};

	const rtems_status_code taskConstructionResult =
		rtems_task_construct(&taskConfig, &self->m_task);
	assert(taskConstructionResult == RTEMS_SUCCESSFUL);

	const rtems_status_code taskStartStatus = rtems_task_start(
		self->m_task, (rtems_task_entry)&Samv71RtemsSerialPoll,
		(rtems_task_argument)self);
	assert(taskStartStatus == RTEMS_SUCCESSFUL);
}

void Samv71RtemsSerialPoll(rtems_task_argument private_data)
{
	samv71_rtems_serial_private_data *self =
		(samv71_rtems_serial_private_data *)private_data;

	if (!self->m_raw_mode) {
		// if raw mode is disabled, start the Escaper's decoder
		Escaper_start_decoder(&self->m_escaper);
	}

	uartRead(&self->m_hal_uart, self->m_fifo_memory_block,
		 Serial_SAMV71_RTEMS_RECV_BUFFER_SIZE, self->m_uart_rx_handler);
	while (true) {
		rtems_event_set received_events = 0;
		/// Wait for data to arrive - event shall be triggered by ISR
		const rtems_status_code eventStatus = rtems_event_receive(
			UART_RX_EVENT, RTEMS_WAIT | RTEMS_EVENT_ANY,
			RTEMS_NO_TIMEOUT, &received_events);

		if (eventStatus == RTEMS_SUCCESSFUL &&
		    (received_events & UART_RX_EVENT)) {
			// Extract all the data to a local buffer to make sure the time in
			// critical section is minimal
			const uint32_t irqMask = enterCriticalSection(self);
			size_t length = 0;
			while (ByteFifo_pull(&self->m_hal_uart.rxFifo,
					     &self->m_recv_buffer[length])) {
				length++;
			}
			exitCriticalSection(self, irqMask);

			if (self->m_raw_mode) {
				// if raw mode is enabled, call the Broker directly
				for (size_t i = 0; i < length; i++) {
					Broker_receive_packet(
						self->m_ip_device_bus_id,
						&self->m_recv_buffer[i], 1);
				}
			} else {
				// if raw mode is disabled, use Escaper
				Escaper_decode_packet(&self->m_escaper,
						      self->m_ip_device_bus_id,
						      self->m_recv_buffer,
						      length,
						      Broker_receive_packet);
			}
		}
	}
}

void Samv71RtemsSerialSend(void *private_data, const uint8_t *const data,
			   const size_t length)
{
	samv71_rtems_serial_private_data *const self =
		(samv71_rtems_serial_private_data *)private_data;
	size_t index = 0;

	if (!self->m_raw_mode) {
		// if raw mode is disabled, start the Escaper's encoder
		// and use it to process all the data before sending
		Escaper_start_encoder(&self->m_escaper);

		while (index < length) {
			const size_t packetLength = Escaper_encode_packet(
				&self->m_escaper, data, length, &index);

			// wait for completion of previous transfer
			const rtems_status_code obtainResult =
				rtems_semaphore_obtain(self->m_tx_semaphore,
						       RTEMS_WAIT,
						       RTEMS_NO_TIMEOUT);
			assert(obtainResult == RTEMS_SUCCESSFUL);

			uartWrite(
				&self->m_hal_uart,
				(uint8_t *const)&self->m_encoded_packet_buffer,
				packetLength, &self->m_uart_tx_handler);
		}
	} else {
		// otherwise skip the encoding and send the data directly
		// wait for completion of previous transfer
		const rtems_status_code obtainResult = rtems_semaphore_obtain(
			self->m_tx_semaphore, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
		assert(obtainResult == RTEMS_SUCCESSFUL);
		uartWrite(&self->m_hal_uart, data, length,
			  &self->m_uart_tx_handler);
	}
}

void Samv71RtemsSerialRegisterUserUartErrorCallback(
	Samv71RtemsSerial_UserUartErrorCallback callback, void *arg)
{
	Samv71RtemsSerial_user_uart_error_callback = callback;
	Samv71RtemsSerial_user_uart_error_callback_arg = arg;
}

void Samv71RtemsSerialRegisterUserXdmadErrorCallback(
	Samv71RtemsSerial_UserXdmadErrorCallback callback, void *arg)
{
	Samv71RtemsSerial_user_xdmad_error_callback = callback;
	Samv71RtemsSerial_user_xdmad_error_callback_arg = arg;
}
