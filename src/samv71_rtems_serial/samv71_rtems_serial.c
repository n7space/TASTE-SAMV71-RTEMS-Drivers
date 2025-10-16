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

#include "samv71_rtems_serial.h"

#include <rtems.h>
#include <assert.h>

#include <Hal.h>

#include <Escaper.h>
#include <EscaperInternal.h>
#include <Nvic/Nvic.h>
#include <Uart/Uart.h>
#include <Xdmac/xdmad.h>
#include <Pio/Pio.h>
#include <Nvic/Nvic.h>
#include <Pmc/Pmc.h>
#include <SamV71Core/SamV71Core.h>

// global variable required by xdmad.c
rtems_id xdmad_lock;

static Uart *uart0handle;
static Uart *uart1handle;
static Uart *uart2handle;
static Uart *uart3handle;
static Uart *uart4handle;

/**
 * @brief UART priotity definition
 * System interrupts priorities levels must be smaller than
 * kernel interrupts levels. The lower the priority value the
 * higher the priority is. Thus, the UART interrupt priority value
 * must be equal or greater then configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY.
 */
#define UART_INTERRUPT_PRIORITY RTEMS_MAXIMUM_PRIORITY
#define UART_XDMAC_INTERRUPT_PRIORITY UART_INTERRUPT_PRIORITY

#define XDMAD_NO_POLLING 0

#define UART_ID_UART0 "UART0: "
#define UART_ID_UART1 "UART1: "
#define UART_ID_UART2 "UART2: "
#define UART_ID_UART3 "UART3: "
#define UART_ID_UART4 "UART4: "

#define UART_XDMAD_ERROR_NO_AVALIABLE_CHANNELS \
	"Hal:Hal_uartWrite: The xdmac channels are not avaliable.\n\r"

#define UART_READ_ERROR_OVERRUN_ERROR "Hal:Hal_uartRead: Overrun error.\n\r"
#define UART_READ_ERROR_FRAME_ERROR "Hal:Hal_uartRead: Frame error.\n\r"
#define UART_READ_ERROR_PARITY_ERROR "Hal:Hal_uartRead: Parity error.\n\r"

#define UART_RX_INTERRUPT_ERROR_FIFO_FULL \
	"Hal:Hal_interruptHandler: FIFO is full.\n\r"

void UART0_Handler(void)
{
	if (uart0handle != NULL)
		Uart_handleInterrupt(uart0handle);
}

void UART1_Handler(void)
{
	if (uart1handle != NULL)
		Uart_handleInterrupt(uart1handle);
}

void UART2_Handler(void)
{
	if (uart2handle != NULL)
		Uart_handleInterrupt(uart2handle);
}

void UART3_Handler(void)
{
	if (uart3handle != NULL)
		Uart_handleInterrupt(uart3handle);
}

void UART4_Handler(void)
{
	if (uart4handle != NULL)
		Uart_handleInterrupt(uart4handle);
}

inline static void Init_setup_xdmad_lock()
{
	const rtems_status_code status_code =
		rtems_semaphore_create(SamV71Core_GenerateNewSemaphoreName(),
				       1, // Initial value, unlocked
				       RTEMS_BINARY_SEMAPHORE,
				       0, // Priority ceiling
				       &xdmad_lock);

	assert(status_code == RTEMS_SUCCESSFUL);
}

static sXdmad xdmad;

void XDMAC_Handler(void)
{
	XDMAD_Handler(&xdmad);
}

static inline void Samv71RtemsSerial_Hal_uart_init_dma(void)
{
	SamV71Core_EnablePeripheralClock(Pmc_PeripheralId_Xdmac);

	Nvic_clearInterruptPending(Nvic_Irq_Xdmac);
	Nvic_setInterruptPriority(Nvic_Irq_Xdmac,
				  UART_XDMAC_INTERRUPT_PRIORITY);
	Nvic_enableInterrupt(Nvic_Irq_Xdmac);

	XDMAD_Initialize(&xdmad, XDMAD_NO_POLLING);
}

static void SamV71RtemsSerial_Init_global()
{
	static bool SamV71RtemsSerial_inited = false;
	if (!SamV71RtemsSerial_inited) {
		SamV71RtemsSerial_inited = true;
		Init_setup_xdmad_lock();
		SamV71Core_InterruptSubscribe(
			58, "xdmac", (rtems_interrupt_handler)&XDMAC_Handler,
			NULL);
		SamV71Core_InterruptSubscribe(
			7, "uart0", (rtems_interrupt_handler)&UART0_Handler,
			NULL);
		SamV71Core_InterruptSubscribe(
			8, "uart1", (rtems_interrupt_handler)&UART1_Handler,
			NULL);
		SamV71Core_InterruptSubscribe(
			44, "uart2", (rtems_interrupt_handler)&UART2_Handler,
			NULL);
		SamV71Core_InterruptSubscribe(
			45, "uart3", (rtems_interrupt_handler)&UART3_Handler,
			NULL);
		SamV71Core_InterruptSubscribe(
			46, "uart4", (rtems_interrupt_handler)&UART4_Handler,
			NULL);
		Samv71RtemsSerial_Hal_uart_init_dma();
	}
}

void Samv71RtemsSerial_Hal_uart_xdmad_handler(uint32_t xdmacChannel, void *args)
{
	XDMAD_FreeChannel(&xdmad, xdmacChannel);
	Uart_TxHandler *uartTxHandler = (Uart_TxHandler *)args;
	uartTxHandler->callback(uartTxHandler->arg);
}

static inline void Samv71RtemsSerial_Hal_uart_print_uart_id(Uart_Id id)
{
	/* switch(id) { */
	/*     case Uart_Id_0: */
	/*         Hal_console_usart_write(UART_ID_UART0, strlen(UART_ID_UART0)); */
	/*         break; */
	/*     case Uart_Id_1: */
	/*         Hal_console_usart_write(UART_ID_UART1, strlen(UART_ID_UART1)); */
	/*         break; */
	/*     case Uart_Id_2: */
	/*         Hal_console_usart_write(UART_ID_UART2, strlen(UART_ID_UART2)); */
	/*         break; */
	/*     case Uart_Id_3: */
	/*         Hal_console_usart_write(UART_ID_UART3, strlen(UART_ID_UART3)); */
	/*         break; */
	/*     case Uart_Id_4: */
	/*         Hal_console_usart_write(UART_ID_UART4, strlen(UART_ID_UART4)); */
	/*         break; */
	/* } */
}

static inline void
Samv71RtemsSerial_Hal_uart_error_handler(Uart_ErrorFlags errorFlags, void *arg)
{
	Samv71RtemsSerial_Uart *halUart = (Samv71RtemsSerial_Uart *)arg;

	Samv71RtemsSerial_Hal_uart_print_uart_id(halUart->uart.id);
	if (errorFlags.hasOverrunOccurred == true) {
		/* Hal_console_usart_write(UART_READ_ERROR_OVERRUN_ERROR,
     * strlen(UART_READ_ERROR_OVERRUN_ERROR)); */
	}
	if (errorFlags.hasFramingErrorOccurred == true) {
		/* Hal_console_usart_write(UART_READ_ERROR_FRAME_ERROR,
     * strlen(UART_READ_ERROR_FRAME_ERROR)); */
	}
	if (errorFlags.hasParityErrorOccurred == true) {
		/* Hal_console_usart_write(UART_READ_ERROR_PARITY_ERROR,
     * strlen(UART_READ_ERROR_PARITY_ERROR)); */
	}
	if (errorFlags.hasRxFifoFullErrorOccurred == true) {
		/* Hal_console_usart_write(UART_RX_INTERRUPT_ERROR_FIFO_FULL,
     * strlen(UART_RX_INTERRUPT_ERROR_FIFO_FULL)); */
		assert(false && "Rx FIFO is full.");
	}
}

inline static void
Samv71RtemsSerial_Hal_uart_init_uart0_pio(Pio_Port *const port,
					  Pio_Port_Config *const pioConfigTx,
					  Pio_Port_Config *const pioConfigRx)
{
	*port = Pio_Port_A;

	pioConfigRx->pins = PIO_PIN_9;
	pioConfigRx->pinsConfig.control = Pio_Control_PeripheralA;

	pioConfigTx->pins = PIO_PIN_10;
	pioConfigTx->pinsConfig.control = Pio_Control_PeripheralA;
}

inline static void
Samv71RtemsSerial_Hal_uart_init_uart1_pio(Pio_Port *const port,
					  Pio_Port_Config *const pioConfigTx,
					  Pio_Port_Config *const pioConfigRx)
{
	*port = Pio_Port_A;

	pioConfigRx->pins = PIO_PIN_5;
	pioConfigRx->pinsConfig.control = Pio_Control_PeripheralC;

	pioConfigTx->pins = PIO_PIN_6;
	pioConfigTx->pinsConfig.control = Pio_Control_PeripheralC;
}

inline static void
Samv71RtemsSerial_Hal_uart_init_uart2_pio(Pio_Port *const port,
					  Pio_Port_Config *const pioConfigTx,
					  Pio_Port_Config *const pioConfigRx)
{
	*port = Pio_Port_D;

	pioConfigRx->pins = PIO_PIN_25;
	pioConfigRx->pinsConfig.control = Pio_Control_PeripheralC;

	pioConfigTx->pins = PIO_PIN_26;
	pioConfigTx->pinsConfig.control = Pio_Control_PeripheralC;
}

inline static void
Samv71RtemsSerial_Hal_uart_init_uart3_pio(Pio_Port *const port,
					  Pio_Port_Config *const pioConfigTx,
					  Pio_Port_Config *const pioConfigRx)
{
	*port = Pio_Port_D;

	pioConfigRx->pins = PIO_PIN_28;
	pioConfigRx->pinsConfig.control = Pio_Control_PeripheralA;

	pioConfigTx->pins = PIO_PIN_30;
	pioConfigTx->pinsConfig.control = Pio_Control_PeripheralA;
}

inline static void
Samv71RtemsSerial_Hal_uart_init_uart4_pio(Pio_Port *const port,
					  Pio_Port_Config *const pioConfigTx,
					  Pio_Port_Config *const pioConfigRx)
{
	*port = Pio_Port_D;

	pioConfigRx->pins = PIO_PIN_18;
	pioConfigRx->pinsConfig.control = Pio_Control_PeripheralC;

	pioConfigTx->pins = PIO_PIN_19;
	pioConfigTx->pinsConfig.control = Pio_Control_PeripheralC;
}

static inline Pmc_PeripheralId
Samv71RtemsSerial_Hal_get_periph_uart_id(Uart_Id id)
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
	}
}

static inline Pmc_PeripheralId
Samv71RtemsSerial_Hal_get_periph_uart_pio_id(Uart_Id id)
{
	switch (id) {
	case Uart_Id_0:
	case Uart_Id_1:
		return Pmc_PeripheralId_PioA;
	case Uart_Id_2:
	case Uart_Id_3:
	case Uart_Id_4:
		return Pmc_PeripheralId_PioD;
	}
}

static inline void Samv71RtemsSerial_Hal_uart_init_pio(Uart_Id id)
{
	Pio_Port port;
	Pio_Port_Config pioConfigTx = {.pinsConfig =
                                     {
                                         .pull = Pio_Pull_Up,
                                         .filter = Pio_Filter_None,
                                         .isMultiDriveEnabled = false,
                                         .isSchmittTriggerDisabled = false,
                                         .irq = Pio_Irq_None,
                                         .direction = Pio_Direction_Output,
                                     },
                                 .debounceFilterDiv = 0};
	pioConfigTx.pinsConfig.direction = Pio_Direction_Output;

	Pio_Port_Config pioConfigRx = pioConfigRx;
	pioConfigRx.pinsConfig.direction = Pio_Direction_Input;

	switch (id) {
	case Uart_Id_0:
		Samv71RtemsSerial_Hal_uart_init_uart0_pio(&port, &pioConfigTx,
							  &pioConfigRx);
		break;
	case Uart_Id_1:
		Samv71RtemsSerial_Hal_uart_init_uart1_pio(&port, &pioConfigTx,
							  &pioConfigRx);
		break;
	case Uart_Id_2:
		Samv71RtemsSerial_Hal_uart_init_uart2_pio(&port, &pioConfigTx,
							  &pioConfigRx);
		break;
	case Uart_Id_3:
		Samv71RtemsSerial_Hal_uart_init_uart3_pio(&port, &pioConfigTx,
							  &pioConfigRx);
		break;
	case Uart_Id_4:
		Samv71RtemsSerial_Hal_uart_init_uart4_pio(&port, &pioConfigTx,
							  &pioConfigRx);
		break;
	}
	Pio pio;
	ErrorCode errorCode = 0;
	Pio_init(port, &pio, &errorCode);
	Pio_setPortConfig(&pio, &pioConfigTx, &errorCode);
	Pio_setPortConfig(&pio, &pioConfigRx, &errorCode);
}

inline static void Samv71RtemsSerial_Hal_uart_init_pmc(Uart_Id id)
{
	SamV71Core_EnablePeripheralClock(
		Samv71RtemsSerial_Hal_get_periph_uart_pio_id(id));
	SamV71Core_EnablePeripheralClock(
		Samv71RtemsSerial_Hal_get_periph_uart_id(id));
}

inline static void Samv71RtemsSerial_Hal_uart_init_handle(Uart *uart,
							  Uart_Id id)
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
	}
}

/// \brief Starts up, initializes and configures Uart and coresponding
/// periferals \param [in] halUart Hal_Uart structure contains uart device
/// descriptor and relevant fifos. \param [in] halUartConfig configuration
/// structure
static void
SamV71RtemsSerialInit_Hal_uart_init(Samv71RtemsSerial_Uart *const halUart,
				    Samv71RtemsSerial_Uart_Config halUartConfig)
{
	SamV71RtemsSerial_Init_global();

	assert(halUartConfig.id <= Uart_Id_4);
	assert((halUartConfig.parity <= Uart_Parity_Odd) ||
	       (halUartConfig.parity == Uart_Parity_None));

	// init uart
	Samv71RtemsSerial_Hal_uart_init_pmc(halUartConfig.id);
	Samv71RtemsSerial_Hal_uart_init_pio(halUartConfig.id);
	Samv71RtemsSerial_Hal_uart_init_handle(&halUart->uart,
					       halUartConfig.id);

	Uart_init(halUartConfig.id, &halUart->uart);
	Uart_startup(&halUart->uart);

	Uart_Config config = { .isTxEnabled = true,
			       .isRxEnabled = true,
			       .isTestModeEnabled = false,
			       .parity = halUartConfig.parity,
			       .baudRate = halUartConfig.baudrate,
			       .baudRateClkSrc = Uart_BaudRateClk_PeripheralCk,
			       .baudRateClkFreq =
				       SamV71Core_GetMainClockFrequency() };
	Uart_setConfig(&halUart->uart, &config);
}

static void Samv71RtemsSerial_Hal_uart_write_init_xdmac_channel(
	Samv71RtemsSerial_Uart *const halUart, uint8_t *const buffer,
	const uint16_t length, const Uart_TxHandler *const txHandler,
	uint32_t channelNumber)
{
	eXdmadRC prepareResult = XDMAD_PrepareChannel(&xdmad, channelNumber);
	assert(prepareResult == XDMAD_OK);

	//< Get Uart Tx peripheral xdmac id
	uint32_t periphID = xdmad.XdmaChannels[channelNumber].bDstTxIfID
			    << XDMAC_CC_PERID_Pos;
	sXdmadCfg config = {
		.mbr_ubc =
			length, //< uBlock max length is equal to uart write max data
		// length. Thus one uBlock can be used.
		.mbr_sa = (uint32_t)buffer, //< Data buffer as source addres
		.mbr_da =
			(uint32_t)&halUart->uart.reg
				->thr, //< Uart tx holding register as a destination address
		.mbr_cfg =
			XDMAC_CC_TYPE_PER_TRAN | XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DSYNC_MEM2PER | XDMAC_CC_SWREQ_HWR_CONNECTED |
			XDMAC_CC_MEMSET_NORMAL_MODE | XDMAC_CC_DWIDTH_BYTE |
			XDMAC_CC_SIF_AHB_IF1 | XDMAC_CC_DIF_AHB_IF1 |
			XDMAC_CC_SAM_INCREMENTED_AM | XDMAC_CC_DAM_FIXED_AM |
			periphID, //< Config memory to peripheral transfer. Increment
		// source buffer address. Keep
		// desitnation address buffer fixed
		.mbr_bc = 0, //< do not add any data stride
		.mbr_ds = 0,
		.mbr_sus = 0,
		.mbr_dus = 0,
	};

	eXdmadRC configureResult = XDMAD_ConfigureTransfer(
		&xdmad, channelNumber, &config, 0, 0,
		XDMAC_CIE_BIE | XDMAC_CIE_RBIE | XDMAC_CIE_WBIE |
			XDMAC_CIE_ROIE);
	assert(configureResult == XDMAD_OK);
	eXdmadRC callbackResult = XDMAD_SetCallback(
		&xdmad, channelNumber, Samv71RtemsSerial_Hal_uart_xdmad_handler,
		(void *)txHandler);
	assert(callbackResult == XDMAD_OK);
}

/// \brief Asynchronously sends bytes over uart.
/// \param [in] halUart Hal_Uart structure contains uart device descriptor and
/// relevant fifos. \param [in] buffer array containing bytes to send \param
/// [in] length length of array of bytes \param [in] txHandler pointer to the
/// handler called after successful array transmission
static void SamV71RtemsSerialInit_Hal_uart_write(
	Samv71RtemsSerial_Uart *const halUart, uint8_t *const buffer,
	const uint16_t length, const Uart_TxHandler *const txHandler)
{
	uint32_t channelNumber = XDMAD_AllocateChannel(
		&xdmad, XDMAD_TRANSFER_MEMORY,
		Samv71RtemsSerial_Hal_get_periph_uart_id(halUart->uart.id));
	if (channelNumber <
	    (xdmad.pXdmacs->XDMAC_GTYPE & XDMAC_GTYPE_NB_CH_Msk)) {
		Samv71RtemsSerial_Hal_uart_write_init_xdmac_channel(
			halUart, buffer, length, txHandler, channelNumber);
		eXdmadRC startResult =
			XDMAD_StartTransfer(&xdmad, channelNumber);
		assert(startResult == XDMAD_OK);
	} else {
		/* Hal_console_usart_write((uint8_t*)UART_XDMAD_ERROR_NO_AVALIABLE_CHANNELS,
     */
		/*                         strlen(UART_XDMAD_ERROR_NO_AVALIABLE_CHANNELS));
     */
	}
}

/// \brief Asynchronously receives bytes over uart.
/// \param [in] halUart Hal_Uart structure contains uart device descriptor and
/// relevant fifos. \param [in] buffer array where received bytes will be
/// storedx \param [in] length length of array of bytes \param [in] rxHandler
/// handler called after successful array reception or after maching character
/// was found
static void
SamV71RtemsSerial_Hal_uart_read(Samv71RtemsSerial_Uart *const halUart,
				uint8_t *const buffer, const uint16_t length,
				const Uart_RxHandler rxHandler)
{
	Uart_ErrorHandler errorHandler = {
		.callback = Samv71RtemsSerial_Hal_uart_error_handler,
		.arg = halUart
	};
	ByteFifo_init(&halUart->rxFifo, buffer, length);
	Uart_registerErrorHandler(&halUart->uart, errorHandler);
	Uart_readAsync(&halUart->uart, &halUart->rxFifo, rxHandler);
}

static inline void
SamV71RtemsSerialInit_uart_register(samv71_rtems_serial_private_data *self,
				    Serial_SamV71_Rtems_Device_T deviceName)
{
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
SamV71RtemsSerialInit_uart_parity(samv71_rtems_serial_private_data *self,
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
		assert(false && "Not supported parity");
	}
}

static inline void
SamV71RtemsSerialInit_uart_baudrate(samv71_rtems_serial_private_data *self,
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
		assert(false && "Not supported baudrate");
		break;
	}
}

static inline void SamV71RtemsSerialInit_uart_init(
	samv71_rtems_serial_private_data *const self,
	const Serial_SamV71_Rtems_Conf_T *const device_configuration)
{
	SamV71RtemsSerialInit_uart_register(self,
					    device_configuration->devname);
	SamV71RtemsSerialInit_uart_parity(self, device_configuration->parity);
	SamV71RtemsSerialInit_uart_baudrate(self, device_configuration->speed);
	SamV71RtemsSerialInit_Hal_uart_init(&self->m_hal_uart,
					    self->m_hal_uart_config);
}

static void UartRxCallback(void *private_data)
{
	samv71_rtems_serial_private_data *self =
		(samv71_rtems_serial_private_data *)private_data;
	rtems_status_code releaseResult =
		rtems_semaphore_release(self->m_rx_semaphore);
	assert(releaseResult == RTEMS_SUCCESSFUL);
}

static void
SamV71RtemsSerialInit_rx_handler(samv71_rtems_serial_private_data *const self)
{
	self->m_uart_rx_handler.characterCallback = UartRxCallback;
	self->m_uart_rx_handler.lengthCallback = UartRxCallback;
	self->m_uart_rx_handler.lengthArg = self;
	self->m_uart_rx_handler.characterArg = self;
	self->m_uart_rx_handler.targetCharacter = STOP_BYTE;
	self->m_uart_rx_handler.targetLength =
		Serial_SAMV71_RTEMS_RECV_BUFFER_SIZE / 2;

	const rtems_status_code status_code =
		rtems_semaphore_create(SamV71Core_GenerateNewSemaphoreName(),
				       1, // Initial value, unlocked
				       RTEMS_SIMPLE_BINARY_SEMAPHORE,
				       0, // Priority ceiling
				       &self->m_rx_semaphore);

	assert(status_code == RTEMS_SUCCESSFUL);
}

static ByteFifo *UartTxCallback(void *private_data)
{
	samv71_rtems_serial_private_data *self =
		(samv71_rtems_serial_private_data *)private_data;

	rtems_status_code releaseResult =
		rtems_semaphore_release(self->m_tx_semaphore);
	assert(releaseResult == RTEMS_SUCCESSFUL);
	return NULL;
}

static void
SamV71RtemsSerialInit_tx_handler(samv71_rtems_serial_private_data *const self)
{
	self->m_uart_tx_handler.callback = UartTxCallback;
	self->m_uart_tx_handler.arg = self;

	const rtems_status_code status_code =
		rtems_semaphore_create(SamV71Core_GenerateNewSemaphoreName(),
				       1, // Initial value, unlocked
				       RTEMS_SIMPLE_BINARY_SEMAPHORE,
				       0, // Priority ceiling
				       &self->m_tx_semaphore);

	assert(status_code == RTEMS_SUCCESSFUL);
}

void Samv71RtemsSerialInit(
	void *private_data, const enum SystemBus bus_id,
	const enum SystemDevice device_id,
	const Serial_SamV71_Rtems_Conf_T *const device_configuration,
	const Serial_SamV71_Rtems_Conf_T *const remote_device_configuration)
{
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

	rtems_task_config taskConfig = {
		.name = rtems_build_name('p', 'o', 'l', 'l'),
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

static inline void SamV71RtemsSerialInterrupt_rx_enable(
	samv71_rtems_serial_private_data *const self)
{
	self->m_hal_uart.uart.reg->ier =
		UART_IER_RXRDY_MASK | UART_IER_FRAME_MASK | UART_IER_OVRE_MASK;
}

static inline void SamV71RtemsSerialInterrupt_rx_disable(
	samv71_rtems_serial_private_data *const self)
{
	self->m_hal_uart.uart.reg->idr =
		UART_IDR_RXRDY_MASK | UART_IDR_FRAME_MASK | UART_IDR_OVRE_MASK;
}

void Samv71RtemsSerialPoll(void *private_data)
{
	samv71_rtems_serial_private_data *self =
		(samv71_rtems_serial_private_data *)private_data;
	size_t length = 0;

	Escaper_start_decoder(&self->m_escaper);
	rtems_status_code obtainResult = rtems_semaphore_obtain(
		self->m_rx_semaphore, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
	assert(obtainResult == RTEMS_SUCCESSFUL);
	SamV71RtemsSerial_Hal_uart_read(&self->m_hal_uart,
					self->m_fifo_memory_block,
					Serial_SAMV71_RTEMS_RECV_BUFFER_SIZE,
					self->m_uart_rx_handler);
	while (true) {
		/// Wait for data to arrive. Semaphore will be given
		obtainResult = rtems_semaphore_obtain(
			self->m_rx_semaphore, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
		assert(obtainResult == RTEMS_SUCCESSFUL);

		length = ByteFifo_getCount(&self->m_hal_uart.rxFifo);

		for (size_t i = 0; i < length; i++) {
			SamV71RtemsSerialInterrupt_rx_disable(self);
			ByteFifo_pull(&self->m_hal_uart.rxFifo,
				      &self->m_recv_buffer[i]);
			SamV71RtemsSerialInterrupt_rx_enable(self);
		}

		Escaper_decode_packet(&self->m_escaper,
				      self->m_ip_device_bus_id,
				      self->m_recv_buffer, length,
				      Broker_receive_packet);
	}
}

void Samv71RtemsSerialSend(void *private_data, const uint8_t *const data,
			   const size_t length)
{
	samv71_rtems_serial_private_data *self =
		(samv71_rtems_serial_private_data *)private_data;
	size_t index = 0;
	size_t packetLength = 0;

	Escaper_start_encoder(&self->m_escaper);
	while (index < length) {
		packetLength = Escaper_encode_packet(&self->m_escaper, data,
						     length, &index);
		rtems_semaphore_obtain(self->m_tx_semaphore, RTEMS_WAIT,
				       RTEMS_NO_TIMEOUT);
		SamV71RtemsSerialInit_Hal_uart_write(
			&self->m_hal_uart,
			(uint8_t *const)&self->m_encoded_packet_buffer,
			packetLength, &self->m_uart_tx_handler);
	}
}
