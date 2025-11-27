#include "samv71_can_generic.h"

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <assert>

#include <Mcan/Mcan.h>
#include <Nvic/Nvic.h>
#include <Pmc/Pmc.h>
#include <Pio/Pio.h>
#include <Pmc/PmcPeripheralId.h>
#include <Utils/ErrorCode.h>

#include <Broker/Broker.h>
#include <SamV71Core/SamV71Core.h>

#define MCAN_WAIT_TIMEOUT 100000u
#define CONFIG_TIMEOUT 1000u

static bool is_can_pck_configured = FALSE;

static const Mcan_Config defaultConfig = {
    .msgRamBaseAddress = NULL,
    .mode = Mcan_Mode_Normal,
    .isFdEnabled = FALSE,
    .nominalBitTiming = {
      .bitRatePrescaler = 0u,
      .synchronizationJump = 2u,
      .timeSegmentAfterSamplePoint = 2u,
      .timeSegmentBeforeSamplePoint = 15u,
    },
    .dataBitTiming = {
      .bitRatePrescaler = 0u,
      .synchronizationJump = 2u,
      .timeSegmentAfterSamplePoint = 2u,
      .timeSegmentBeforeSamplePoint = 15u,
    },
    .transmitterDelayCompensation = {
      .isEnabled = FALSE,
      .filter = 0u,
      .offset = 0u,
    },
    .timestampClk = Mcan_TimestampClk_Internal,
    .timestampTimeoutPrescaler = 14u,
    .timeout = {
      .isEnabled = FALSE,
      .type = Mcan_TimeoutType_Continuous,
      .period = 0u,
    },
    .standardIdFilter = {
      .isIdRejected = FALSE,
      .nonMatchingPolicy = Mcan_NonMatchingPolicy_RxFifo0,
      .filterListAddress = NULL,
      .filterListSize = 0u,
    },
    .extendedIdFilter = {
      .isIdRejected = FALSE,
      .nonMatchingPolicy = Mcan_NonMatchingPolicy_RxFifo0,
      .filterListAddress = NULL,
      .filterListSize = 0u,
    },
    .rxFifo0 = {
      .isEnabled = TRUE,
      .startAddress = NULL,
      .size = MSGRAM_RXFIFO0_SIZE / sizeof(uint32_t),
      .watermark = 0u,
      .mode = Mcan_RxFifoOperationMode_Blocking,
      .elementSize = Mcan_ElementSize_8,
    },
    .rxFifo1 = {
      .isEnabled = FALSE,
      .startAddress = NULL,
      .size = MSGRAM_RXFIFO1_SIZE / sizeof(uint32_t),
      .watermark = 0u,
      .mode = Mcan_RxFifoOperationMode_Blocking,
      .elementSize = Mcan_ElementSize_8,
    },
    .rxBuffer = {
      .startAddress = NULL,
      .elementSize = Mcan_ElementSize_8,
    },
    .txBuffer = {
      .isEnabled = TRUE,
      .startAddress = NULL,
      .bufferSize = 0u,
      .queueSize = MSGRAM_TXBUFFER_SIZE / sizeof(uint32_t),
      .queueType = Mcan_TxQueueType_Fifo,
      .elementSize = Mcan_ElementSize_8,
    },
    .txEventFifo = {.isEnabled = FALSE,
					.startAddress = NULL,
					.size = 0,
					.watermark = 0,
	},
    .interrupts = {
	  {
		.isEnabled = TRUE,
		.line = Mcan_InterruptLine_0,
	  },
	  {
		.isEnabled = FALSE,
		.line = Mcan_InterruptLine_1,
	  }
	},
    .isLine0InterruptEnabled = TRUE,
    .isLine1InterruptEnabled = FALSE,
    .wdtCounter = 0u,
  };

static uint8_t getElementBytesCount(const Mcan_ElementSize elementSize)
{
	switch (elementSize) {
	case Mcan_ElementSize_8:
		return 8;
	case Mcan_ElementSize_12:
		return 12;
	case Mcan_ElementSize_16:
		return 16;
	case Mcan_ElementSize_20:
		return 20;
	case Mcan_ElementSize_24:
		return 24;
	case Mcan_ElementSize_32:
		return 32;
	case Mcan_ElementSize_48:
		return 48;
	case Mcan_ElementSize_64:
		return 64;
	default:
		return 0;
	}
}

static bool txCompleteIrqCalled = false;
static bool rxWatermarkIrqCalled = false;
static bool timeoutOccurredIrqCalled = false;

static void MCAN0_INT0_Handler(void *private_data)
{
	/* Mcan_InterruptStatus status; */
	/* Mcan_getInterruptStatus(&mcan, &status); */
	/* if (status.hasTcOccurred) { */
	/* 	txCompleteIrqCalled = true; */
	/* } */
	/* if (status.hasRf0wOccurred) { */
	/* 	rxWatermarkIrqCalled = true; */
	/* } */
	/* if (status.hasTooOccurred) { */
	/* 	timeoutOccurredIrqCalled = true; */
	/* } */
	samv71_can_generic_private_data *self =
		(samv71_can_generic_private_data *)private_data;

	Mcan_InterruptStatus status;
	Mcan_getInterruptStatus(&self->mcan, &status);
	if (status.hasRf0nOccurred) {
		rtems_status_code releaseResult =
			rtems_semaphore_release(self->m_rx_semaphore);
		assert(releaseResult == RTEMS_SUCCESSFUL);
	}
}

static void MCAN1_INT0_Handler(void *private_data)
{
	samv71_can_generic_private_data *self =
		(samv71_can_generic_private_data *)private_data;

	Mcan_InterruptStatus status;
	Mcan_getInterruptStatus(&self->mcan, &status);
	if (status.hasRf0nOccurred) {
		rtems_status_code releaseResult =
			rtems_semaphore_release(self->m_rx_semaphore);
		assert(releaseResult == RTEMS_SUCCESSFUL);
	}
}

static bool waitForTransmissionFinished(Mcan *mcan, uint32_t timeout,
					const uint8_t index)
{
	for (uint32_t counter = 0; counter < timeout; ++counter) {
		if (Mcan_txBufferIsTransmissionFinished(mcan, index)) {
			return true;
		}
	}
	return false;
}

static void fillMsgData(uint8_t *txData, uint8_t bytesCount)
{
	for (uint8_t i = 0; i < bytesCount; ++i) {
		txData[i] = i + 1u;
	}
}

static void configurePioCan0(Pio *pio)
{
	Pio_Pin_Config pioCanTxConfig = {
		.control = Pio_Control_PeripheralA,
		.direction = Pio_Direction_Output,
		.pull = Pio_Pull_Up,
		.filter = Pio_Filter_None,
		.isMultiDriveEnabled = FALSE,
		.irq = Pio_Irq_None,
		.driveStrength = Pio_Drive_Low,
		.isSchmittTriggerDisabled = FALSE,
	};
	SamV71Core_EnablePeripheralClock(Pmc_PeripheralId_PioB);
	ErrorCode errorCode = 0;
	bool pioStatus = Pio_init(Pio_Port_B, pio, &errorCode);
	assert(pioStatus);
	assert(errorCode == ErrorCode_NoError);
	pioStatus = Pio_setPinsConfig(pio, PIO_PIN_2 | PIO_PIN_2,
				      &pioCanTxConfig, &errorCode);
	assert(pioStatus);
	assert(errorCode == ErrorCode_NoError);
}

static void configurePioCan1(Pio *pio)
{
	const Pio_Pin_Config pioCanTxConfig = {
		.control = Pio_Control_PeripheralC,
		.direction = Pio_Direction_Output,
		.pull = Pio_Pull_Up,
		.filter = Pio_Filter_None,
		.isMultiDriveEnabled = FALSE,
		.irq = Pio_Irq_None,
		.driveStrength = Pio_Drive_Low,
		.isSchmittTriggerDisabled = FALSE,
	};
	SamV71Core_EnablePeripheralClock(Pmc_PeripheralId_PioC);
	ErrorCode errorCode = 0;
	bool pioStatus = Pio_init(Pio_Port_C, pio, &errorCode);
	assert(pioStatus);
	assert(errorCode == ErrorCode_NoError);
	pioStatus = Pio_setPinsConfig(pio, PIO_PIN_14 | PIO_PIN_12,
				      &pioCanTxConfig, &errorCode);
	assert(pioStatus);
	assert(errorCode == ErrorCode_NoError);
}

static void configureMcan0(samv71_can_generic_private_data *self,
			   const CAN_Samv71_Rtems_Conf_T *const config)
{
	configurePioCan0(&self->pioCanTx);
	const Pmc_PckConfig pckConfig = {
		.isEnabled = true,
		.src = Pmc_PckSrc_Pllack,
		.presc = 14,
	};

	bool setCfgResult = SamV71Core_SetPckConfig(Pmc_PckId_5, &pckConfig,
						    PMC_DEFAULT_TIMEOUT, NULL);
	assert(setCfgResult);

	SamV71Core_InterruptSubscribe(Nvic_Irq_Mcan0_Irq0, "mcan0_0",
				      MCAN0_INT0_Handler, self);
	SamV71Core_EnablePeripheralClock(Pmc_PeripheralId_Mcan0);
	Mcan_init(&self->mcan, Mcan_getDeviceRegisters(Mcan_Id_0));
}

static void configureMcan1(samv71_can_generic_private_data *self,
			   const CAN_Samv71_Rtems_Conf_T *const config)
{
	configurePioCan1(&self->pioCanTx);
	const Pmc_PckConfig pckConfig = {
		.isEnabled = true,
		.src = Pmc_PckSrc_Pllack,
		.presc = 14,
	};

	bool setCfgResult = SamV71Core_SetPckConfig(Pmc_PckId_5, &pckConfig,
						    PMC_DEFAULT_TIMEOUT, NULL);
	assert(setCfgResult);
	SamV71Core_InterruptSubscribe(Nvic_Irq_Mcan1_Irq0, "mcan1_0",
				      MCAN1_INT0_Handler, self);
	SamV71Core_EnablePeripheralClock(Pmc_PeripheralId_Mcan1);
	Mcan_init(&self->mcan, Mcan_getDeviceRegisters(Mcan_Id_1));
}

void SamV71RtemsCanInit(
	void *private_data, const enum SystemBus bus_id,
	const enum SystemDevice device_id,
	const CAN_Samv71_Rtems_Conf_T *const device_configuration,
	const CAN_Samv71_Rtems_Conf_T *const remote_device_configuration)
{
	samv71_can_generic_private_data *self =
		(samv71_can_generic_private_data *)private_data;

	memset(self->msgRam, 0, MSGRAM_SIZE * sizeof(uint32_t));
	self->m_bus_id = bus_id;
	self->m_config = device_configuration;

	if (self->m_config->can_interface == mcan_interface_mcan0) {
		configureMcan0(self, self->m_config);
	} else if (device_configuration->can_interface == mcan_interface_mcan1) {
		configureMcan1(self, self->m_config);
	} else {
		assert(0);
	}

	rtems_cache_disable_data();

	Mcan_Config conf = defaultConfig;
	conf.msgRamBaseAddress = self->msgRam;
	conf.standardIdFilter.filterListAddress =
		&self->msgRam[MSGRAM_STDID_FILTER_OFFSET];
	conf.extendedIdFilter.filterListAddress =
		&self->msgRam[MSGRAM_EXTID_FILTER_OFFSET];
	conf.rxFifo0.startAddress = &self->msgRam[MSGRAM_RXFIFO0_OFFSET];
	conf.rxFifo1.startAddress = &self->msgRam[MSGRAM_RXFIFO1_OFFSET];
	conf.rxBuffer.startAddress = &self->msgRam[MSGRAM_RXBUFFER_OFFSET];
	conf.txBuffer.startAddress = &self->msgRam[MSGRAM_TXBUFFER_OFFSET];
	conf.txEventFifo.startAddress = &self->msgRam[MSGRAM_TXBUFFER_OFFSET];

	ErrorCode errCode = ErrorCode_NoError;
	bool setConfResult =
		Mcan_setConfig(&self->mcan, &conf, CONFIG_TIMEOUT, &errCode);
	assert(setConfResult);
	assert(errCode == ErrorCode_NoError);

	/* this is for comparision */
	/* Mcan_Config readConfig; */
	/* Mcan_getConfig(&mcan, &readConfig); */
	/* int cmpResult = memcmp(&conf, &readConfig, sizeof(Mcan_Config)); */
	/* assert(cmpResult == 0); */

	const rtems_status_code status_code =
		rtems_semaphore_create(SamV71Core_GenerateNewSemaphoreName(),
				       1, // Initial value, unlocked
				       RTEMS_SIMPLE_BINARY_SEMAPHORE,
				       0, // Priority ceiling
				       &self->m_rx_semaphore);

	assert(status_code == RTEMS_SUCCESSFUL);

	rtems_task_config taskConfig = {
		.name = rtems_build_name('p', 'o', 'l', 'l'),
		.initial_priority = 1,
		.storage_area = self->m_task_buffer,
		.storage_size = Can_SAMV71_RTEMS_TASK_BUFFER_SIZE,
		.maximum_thread_local_storage_size =
			Can_SAMV71_RTEMS_UART_TLS_SIZE,
		.storage_free = NULL,
		.initial_modes = RTEMS_PREEMPT,
		.attributes = RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT
	};

	const rtems_status_code taskConstructionResult =
		rtems_task_construct(&taskConfig, &self->m_task);
	assert(taskConstructionResult == RTEMS_SUCCESSFUL);

	const rtems_status_code taskStartStatus = rtems_task_start(
		self->m_task, (rtems_task_entry)&SamV71RtemsCanPoll,
		(rtems_task_argument)self);
	assert(taskStartStatus == RTEMS_SUCCESSFUL);
}

void SamV71RtemsCanPoll(void *private_data)
{
	samv71_can_generic_private_data *self =
		(samv71_can_generic_private_data *)private_data;
	ErrorCode errCode = ErrorCode_NoError;

	while (true) {
		/// Wait for data to arrive. Semaphore will be given
		rtems_status_code obtainResult = obtainResult =
			rtems_semaphore_obtain(self->m_rx_semaphore, RTEMS_WAIT,
					       RTEMS_NO_TIMEOUT);
		assert(obtainResult == RTEMS_SUCCESSFUL);

		Mcan_RxFifoStatus fifoStatus;
		bool fifoStatusResult = Mcan_getRxFifoStatus(
			&self->mcan, Mcan_RxFifoId_0, &fifoStatus, NULL);
		assert(fifoStatusResult);
		if (fifoStatus.count > 0) {
			uint8_t rxData[64];
			Mcan_RxElement rxElement;
			rxElement.data = rxData;
			bool fifoPullResult =
				Mcan_rxFifoPull(&self->mcan, Mcan_RxFifoId_0,
						&rxElement, &errCode);

			assert(fifoPullResult);
			assert(errCode == ErrorCode_NoError);

			Broker_receive_packet(self->m_bus_id, rxData,
					      rxElement.dataSize);
		}
	}
}

void SamV71RtemsCanSend(void *private_data, const uint8_t *const data,
			const size_t length)
{
	samv71_can_generic_private_data *self =
		(samv71_can_generic_private_data *)private_data;

	ErrorCode errCode = ErrorCode_NoError;

	Mcan_TxElement txElement;

	if (true) { // TODO change
		txElement.esiFlag = Mcan_ElementEsi_Dominant;
		txElement.idType = Mcan_IdType_Standard;
		txElement.frameType = Mcan_FrameType_Data;
		txElement.id = 0x7ff;
		txElement.marker = 0;
		txElement.isTxEventStored = FALSE;
		txElement.isCanFdFormatEnabled = FALSE;
		txElement.isBitRateSwitchingEnabled = FALSE;
		txElement.dataSize = length;
		txElement.data = data;
		txElement.isInterruptEnabled = FALSE;
	} else if (true) { // TODO change
		const uint32_t id = *(uint32_t *)(data);

		txElement.esiFlag = Mcan_ElementEsi_Dominant;
		txElement.idType = id & 0xe0000000 ? Mcan_IdType_Extended :
						     Mcan_IdType_Standard;
		txElement.frameType = Mcan_FrameType_Data;
		txElement.id = id & 0x1fffffff >> 29;
		txElement.marker = 0;
		txElement.isTxEventStored = FALSE;
		txElement.isCanFdFormatEnabled = FALSE;
		txElement.isBitRateSwitchingEnabled = FALSE;
		txElement.dataSize = length - sizeof(uint32_t);
		txElement.data = data + sizeof(uint32_t);
		txElement.isInterruptEnabled = FALSE;
	} else {
		assert(0);
	}

	uint8_t pushIndex;
	bool pushResult =
		Mcan_txQueuePush(&self->mcan, txElement, &pushIndex, &errCode);
	assert(pushResult);
	assert(errCode == ErrorCode_NoError);

	bool result = waitForTransmissionFinished(&self->mcan,
						  MCAN_WAIT_TIMEOUT, pushIndex);
	assert(result);
}

// config shall be here
// so, the europrund
// escaper
