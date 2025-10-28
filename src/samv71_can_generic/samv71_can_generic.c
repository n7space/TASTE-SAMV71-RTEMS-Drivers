#include "samv71_can_generic.h"

#include "Utils/ErrorCode.h"
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <Mcan/Mcan.h>
#include <Nvic/Nvic.h>
#include <Pmc/Pmc.h>
#include <Pio/Pio.h>

#include <SamV71Core/SamV71Core.h>

#define MSGRAM_SIZE 1024
#define MSGRAM_STDID_FILTER_OFFSET 0
#define MSGRAM_EXTID_FILTER_OFFSET 4
#define MSGRAM_RXFIFO0_OFFSET 16
#define MSGRAM_RXFIFO1_OFFSET 144
#define MSGRAM_RXBUFFER_OFFSET 272
#define MSGRAM_TXEVENTINFO_OFFSET 400
#define MSGRAM_TXBUFFER_OFFSET 432

// TODO temporary
#define MCAN_TEST_TIMEOUT 100000u
#define CONFIG_TIMEOUT 1000u

static  __attribute__((aligned(4096))) uint32_t msgRam[MSGRAM_SIZE];

static Mcan mcan;

static const Mcan_Config defaultConfig = {
    .msgRamBaseAddress = msgRam,
    .mode = Mcan_Mode_InternalLoopBackTest,
    .isFdEnabled = TRUE,
    .nominalBitTiming = {
      .bitRatePrescaler = 23u,
      .synchronizationJump = 1u,
      .timeSegmentAfterSamplePoint = 1u,
      .timeSegmentBeforeSamplePoint = 1u,
    },
    .dataBitTiming = {
      .bitRatePrescaler = 1u,
      .synchronizationJump = 1u,
      .timeSegmentAfterSamplePoint = 1u,
      .timeSegmentBeforeSamplePoint = 1u,
    },
    .transmitterDelayCompensation = {
      .isEnabled = FALSE,
      .filter = 0u,
      .offset = 0u,
    },
    .timestampClk = Mcan_TimestampClk_Internal,
    .timestampTimeoutPrescaler = 0u,
    .timeout = {
      .isEnabled = FALSE,
      .type = Mcan_TimeoutType_Continuous,
      .period = 0u,
    },
    .standardIdFilter = {
      .isIdRejected = TRUE,
      .nonMatchingPolicy = Mcan_NonMatchingPolicy_RxFifo0,
      .filterListAddress = NULL,
      .filterListSize = 0u,
    },
    .extendedIdFilter = {
      .isIdRejected = TRUE,
      .nonMatchingPolicy = Mcan_NonMatchingPolicy_RxFifo0,
      .filterListAddress = NULL,
      .filterListSize = 0u,
    },
    .rxFifo0 = {
      .isEnabled = TRUE,
      .startAddress = &msgRam[MSGRAM_RXFIFO0_OFFSET],
      .size = 1u,
      .watermark = 0u,
      .mode = Mcan_RxFifoOperationMode_Overwrite,
      .elementSize = Mcan_ElementSize_8,
    },
    .rxFifo1 = {
      .isEnabled = TRUE,
      .startAddress = &msgRam[MSGRAM_RXFIFO1_OFFSET],
      .size = 1u,
      .watermark = 0u,
      .mode = Mcan_RxFifoOperationMode_Overwrite,
      .elementSize = Mcan_ElementSize_8,
    },
    .rxBuffer = {
      .startAddress = &msgRam[MSGRAM_RXBUFFER_OFFSET],
      .elementSize = Mcan_ElementSize_8,
    },
    .txBuffer = {
      .isEnabled = TRUE,
      .startAddress = &msgRam[MSGRAM_TXBUFFER_OFFSET],
      .bufferSize = 1u,
      .queueSize = 0u,
      .queueType = Mcan_TxQueueType_Fifo,
      .elementSize = Mcan_ElementSize_8,
    },
    .txEventFifo = {.isEnabled = false,
					.startAddress = NULL,
					.size = 0,
					.watermark = 0,
	},
    .interrupts = {},
    .isLine0InterruptEnabled = FALSE,
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

static void MCAN1_INT0_Handler(void)
{
	Mcan_InterruptStatus status;
	Mcan_getInterruptStatus(&mcan, &status);
	if (status.hasTcOccurred) {
		txCompleteIrqCalled = true;
	}
	if (status.hasRf0wOccurred) {
		rxWatermarkIrqCalled = true;
	}
	if (status.hasTooOccurred) {
		timeoutOccurredIrqCalled = true;
	}
}

static bool waitForTransmissionFinished(uint32_t timeout)
{
	for (uint32_t counter = 0; counter < timeout; ++counter) {
		if (Mcan_txBufferIsTransmissionFinished(&mcan, 0)) {
			return true;
		}
	}
	return false;
}

void SamV71RtemsCanInit(
	void *private_data, const enum SystemBus bus_id,
	const enum SystemDevice device_id,
	const CAN_Samv71_Rtems_Conf_T *const device_configuration,
	const CAN_Samv71_Rtems_Conf_T *const remote_device_configuration)
{
	const Pmc_PckConfig pckConfig = {
		.isEnabled = true,
		.src = Pmc_PckSrc_Pllack,
		.presc = 4,
	};

	bool setCfgResult = SamV71Core_SetPckConfig(Pmc_PckId_5, &pckConfig,
						    PMC_DEFAULT_TIMEOUT, NULL);
	assert(setCfgResult);

  	Pio pioCanTx;
	ErrorCode errorCode = 0;
	bool pioStatus = Pio_init(Pio_Port_C, &pioCanTx, &errorCode);
  	assert(pioStatus);
	assert(errorCode == ErrorCode_NoError);
	Pio_Port_Config pioCanTxConfig = {
	  .pins = PIO_PIN_14,
	  .pinsConfig = {
		.control = Pio_Control_PeripheralC,
		.direction = Pio_Direction_Output,
		.pull = Pio_Pull_Up,
		.filter = Pio_Filter_None,
		.isMultiDriveEnabled = FALSE,
		.irq = Pio_Irq_None,
		.driveStrength = Pio_Drive_Low,
		.isSchmittTriggerDisabled = FALSE,
	  },
	  .debounceFilterDiv = 0,
	};
	pioStatus = Pio_setPortConfig(&pioCanTx, &pioCanTxConfig, &errorCode);
  	assert(pioStatus);
	assert(errorCode == ErrorCode_NoError);

	Pio pioCanRx;
	pioStatus = Pio_init(Pio_Port_C, &pioCanRx, &errorCode);
	assert(pioStatus);
	assert(errorCode == ErrorCode_NoError);
	Pio_Port_Config pioCanRxConfig = {
	  .pins = PIO_PIN_12,
	  .pinsConfig = {
		.control = Pio_Control_PeripheralC,
		.direction = Pio_Direction_Input,
		.pull = Pio_Pull_Up,
		.filter = Pio_Filter_None,
		.isMultiDriveEnabled = FALSE,
		.irq = Pio_Irq_None,
		.driveStrength = Pio_Drive_Low,
		.isSchmittTriggerDisabled = FALSE,
	  },
	  .debounceFilterDiv = 0,
	};
	pioStatus = Pio_setPortConfig(&pioCanRx, &pioCanRxConfig, &errorCode);
	assert(pioStatus);
	assert(errorCode == ErrorCode_NoError);

	SamV71Core_InterruptSubscribe(Nvic_Irq_Mcan1_Irq0, "mcan1_0",
				      MCAN1_INT0_Handler, NULL);
	SamV71Core_EnablePeripheralClock(Pmc_PeripheralId_Mcan0);
	SamV71Core_EnablePeripheralClock(Pmc_PeripheralId_Mcan1);

	memset(msgRam, sizeof(msgRam), '\0');
	Mcan_init(&mcan, Mcan_getDeviceRegisters(Mcan_Id_1));

	const Mcan_ElementSize testElementSize = Mcan_ElementSize_8;

	// possibly here is something more

	Mcan_Config conf = defaultConfig;
	conf.standardIdFilter.isIdRejected = TRUE;
	conf.extendedIdFilter.isIdRejected = TRUE;
	conf.extendedIdFilter.nonMatchingPolicy =
		Mcan_NonMatchingPolicy_RxFifo0;
	conf.extendedIdFilter.filterListSize = 0;
	conf.rxFifo0.elementSize = testElementSize;
	conf.txBuffer.elementSize = testElementSize;

	ErrorCode errCode = ErrorCode_NoError;
	bool setConfResult =
		Mcan_setConfig(&mcan, &conf, CONFIG_TIMEOUT, &errCode);
	assert(setConfResult);
	assert(errCode == ErrorCode_NoError);

	// this is for comparision
	Mcan_Config readConfig;
	Mcan_getConfig(&mcan, &readConfig);

	int cmpResult = memcmp(&conf, &readConfig, sizeof(Mcan_Config));
	assert(cmpResult == 0);

	const uint8_t bytesCount = getElementBytesCount(testElementSize);

	uint8_t txData[64];
	for (uint8_t i = 0; i < bytesCount; ++i) {
		txData[i] = i + 1u;
	}

	const Mcan_TxElement txElement = {
		.esiFlag = Mcan_ElementEsi_Dominant,
		.idType = Mcan_IdType_Extended,
		.frameType = Mcan_FrameType_Data,
		.id = 3,
		.marker = 1,
		.isTxEventStored = FALSE,
		.isCanFdFormatEnabled = TRUE,
		.isBitRateSwitchingEnabled = FALSE,
		.dataSize = bytesCount,
		.data = txData,
		.isInterruptEnabled = FALSE,
	};

	bool bufferAddResult = Mcan_txBufferAdd(&mcan, txElement, 0, &errCode);
	assert(bufferAddResult);
	assert(errCode == ErrorCode_NoError);

	bool result = waitForTransmissionFinished(MCAN_TEST_TIMEOUT);
	assert(result);
	Mcan_RxFifoStatus fifoStatus;
	bool fifoStatusResult =
		Mcan_getRxFifoStatus(&mcan, Mcan_RxFifoId_0, &fifoStatus, NULL);
	assert(fifoStatusResult);
	assert(fifoStatus.count == 1);
	assert(fifoStatus.isFull);
	assert(fifoStatus.isMessageLost == FALSE);

	uint8_t rxData[64];
	Mcan_RxElement rxElement;
	rxElement.data = rxData;
	bool fifoPullResult =
		Mcan_rxFifoPull(&mcan, Mcan_RxFifoId_0, &rxElement, &errCode);

	assert(fifoPullResult);
	assert(errCode == ErrorCode_NoError);
}

void SamV71RtemsCanPoll(void *private_data)
{
}

void SamV71RtemsCanSend(void *private_data, const uint8_t *const data,
			const size_t length)
{
}
