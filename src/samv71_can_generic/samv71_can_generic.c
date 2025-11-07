#include "samv71_can_generic.h"

#include "Pmc/PmcPeripheralId.h"
#include "Utils/ErrorCode.h"
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <Mcan/Mcan.h>
#include <Nvic/Nvic.h>
#include <Pmc/Pmc.h>
#include <Pio/Pio.h>

#include <SamV71Core/SamV71Core.h>

#define MSGRAM_SIZE 512
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

// TODO temporary
#define MCAN_TEST_TIMEOUT 100000u
#define CONFIG_TIMEOUT 1000u

static __attribute__((aligned(4096))) uint32_t msgRam[MSGRAM_SIZE];

static Mcan mcan;

static const Mcan_Config defaultConfig = {
    .msgRamBaseAddress = msgRam,
    .mode = Mcan_Mode_Normal, // Mcan_Mode_InternalLoopBackTest,
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
      .isIdRejected = TRUE,
      .nonMatchingPolicy = Mcan_NonMatchingPolicy_RxFifo0,
      .filterListAddress = NULL,
      .filterListSize = 0u,
    },
    .rxFifo0 = {
      .isEnabled = TRUE,
      .startAddress = &msgRam[MSGRAM_RXFIFO0_OFFSET],
      .size = MSGRAM_RXFIFO0_SIZE / sizeof(uint32_t),
      .watermark = 0u,
      .mode = Mcan_RxFifoOperationMode_Blocking,
      .elementSize = Mcan_ElementSize_8,
    },
    .rxFifo1 = {
      .isEnabled = FALSE,
      .startAddress = &msgRam[MSGRAM_RXFIFO1_OFFSET],
      .size = MSGRAM_RXFIFO1_SIZE / sizeof(uint32_t),
      .watermark = 0u,
      .mode = Mcan_RxFifoOperationMode_Blocking,
      .elementSize = Mcan_ElementSize_8,
    },
    .rxBuffer = {
      .startAddress = &msgRam[MSGRAM_RXBUFFER_OFFSET],
      .elementSize = Mcan_ElementSize_8,
    },
    .txBuffer = {
      .isEnabled = TRUE,
      .startAddress = &msgRam[MSGRAM_TXBUFFER_OFFSET],
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
		.isEnabled = FALSE,
		.line = 0, //Mcan_InterruptLine_0,
	  },
	  {
		.isEnabled = TRUE,
		.line = Mcan_InterruptLine_1,
	  }
	},
    .isLine0InterruptEnabled = FALSE,
    .isLine1InterruptEnabled = TRUE,
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

static bool waitForTransmissionFinished(uint32_t timeout, const uint8_t index)
{
	for (uint32_t counter = 0; counter < timeout; ++counter) {
		if (Mcan_txBufferIsTransmissionFinished(&mcan, index)) {
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

static void configurePioCan0()
{
	static Pio pioCanTx;
	ErrorCode errorCode = 0;
	bool pioStatus = Pio_init(Pio_Port_B, &pioCanTx, &errorCode);
	assert(pioStatus);
	assert(errorCode == ErrorCode_NoError);
	//Pio_Port_Config pioCanTxConfig = {
	//.pins = PIO_PIN_2,
	Pio_Pin_Config pioCanTxConfig = {
		.control = Pio_Control_PeripheralB,
		.direction = Pio_Direction_Output,
		.pull = Pio_Pull_Up,
		.filter = Pio_Filter_None,
		.isMultiDriveEnabled = FALSE,
		.irq = Pio_Irq_None,
		.driveStrength = Pio_Drive_Low,
		.isSchmittTriggerDisabled = FALSE,
	};
	//.debounceFilterDiv = 0,
	//};
	//pioStatus = Pio_setPortConfig(&pioCanTx, &pioCanTxConfig, &errorCode);
	pioStatus = Pio_setPinsConfig(&pioCanTx, PIO_PIN_2 | PIO_PIN_2, &pioCanTxConfig,
				      &errorCode);
	assert(pioStatus);
	assert(errorCode == ErrorCode_NoError);

	/* Pio pioCanRx; */
	/* pioStatus = Pio_init(Pio_Port_B, &pioCanTx, &errorCode); */
	/* assert(pioStatus); */
	/* assert(errorCode == ErrorCode_NoError); */
	/* Pio_Port_Config pioCanRxConfig = { */
	/* .pins = PIO_PIN_3, */
	const Pio_Pin_Config pioCanRxConfig = {
		.control = Pio_Control_PeripheralA,
		.direction = Pio_Direction_Output,
		.pull = Pio_Pull_Up,
		.filter = Pio_Filter_None,
		.isMultiDriveEnabled = FALSE,
		.irq = Pio_Irq_None,
		.driveStrength = Pio_Drive_Low,
		.isSchmittTriggerDisabled = FALSE,
	};
	/* 	.debounceFilterDiv = 0, */
	/* }; */
	//pioStatus = Pio_setPortConfig(&pioCanRx, &pioCanRxConfig, &errorCode);
	/* pioStatus = Pio_setPinsConfig(&pioCanTx, PIO_PIN_3, &pioCanRxConfig, */
	/* 			      &errorCode); */
	/* assert(pioStatus); */
	/* assert(errorCode == ErrorCode_NoError); */
    SamV71Core_EnablePeripheralClock(Pmc_PeripheralId_PioB);
}

static void configurePioCan1()
{
	static Pio pioCanTx;
	ErrorCode errorCode = 0;
	bool pioStatus = Pio_init(Pio_Port_C, &pioCanTx, &errorCode);
	assert(pioStatus);
	assert(errorCode == ErrorCode_NoError);
	/* Pio_Port_Config pioCanTxConfig = { */
	/*   .pins = PIO_PIN_14, */
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
	/*   .debounceFilterDiv = 0, */
	/* }; */
	//pioStatus = Pio_setPortConfig(&pioCanTx, &pioCanTxConfig, &errorCode);
	pioStatus = Pio_setPinsConfig(&pioCanTx, PIO_PIN_14 | PIO_PIN_12, &pioCanTxConfig,
				      &errorCode);
	assert(pioStatus);
	assert(errorCode == ErrorCode_NoError);

	/* Pio pioCanRx; */
	/* pioStatus = Pio_init(Pio_Port_C, &pioCanTx, &errorCode); */
	/* assert(pioStatus); */
	/* assert(errorCode == ErrorCode_NoError); */
	/* Pio_Port_Config pioCanRxConfig = { */
	/*   .pins = PIO_PIN_12, */
	/* const Pio_Pin_Config pioCanRxConfig = { */
	/* 	.control = Pio_Control_PeripheralC, */
	/* 	.direction = Pio_Direction_Output, */
	/* 	.pull = Pio_Pull_Up, */
	/* 	.filter = Pio_Filter_None, */
	/* 	.isMultiDriveEnabled = FALSE, */
	/* 	.irq = Pio_Irq_None, */
	/* 	.driveStrength = Pio_Drive_Low, */
	/* 	.isSchmittTriggerDisabled = FALSE, */
	/* }; */
	/*   .debounceFilterDiv = 0, */
	/* }; */
	//pioStatus = Pio_setPortConfig(&pioCanRx, &pioCanRxConfig, &errorCode);
	/* pioStatus = Pio_setPinsConfig(&pioCanTx, PIO_PIN_12, &pioCanRxConfig, */
	/* 			      &errorCode); */
	/* assert(pioStatus); */
	/* assert(errorCode == ErrorCode_NoError); */


	SamV71Core_EnablePeripheralClock(Pmc_PeripheralId_PioC);

}

static void gotMsg(void)
{}

void SamV71RtemsCanInit(
	void *private_data, const enum SystemBus bus_id,
	const enum SystemDevice device_id,
	const CAN_Samv71_Rtems_Conf_T *const device_configuration,
	const CAN_Samv71_Rtems_Conf_T *const remote_device_configuration)
{
	/* const Pmc_PckConfig pckConfig = { */
	/* 	.isEnabled = true, */
	/* 	.src = Pmc_PckSrc_Pllack, */
	/* 	.presc = 14, */
	/* }; */

	/* bool setCfgResult = SamV71Core_SetPckConfig(Pmc_PckId_5, &pckConfig, */
	/* 					    PMC_DEFAULT_TIMEOUT, NULL); */
	/* assert(setCfgResult); */

    	//configurePioCan0();
	configurePioCan1();

	/* SamV71Core_InterruptSubscribe(Nvic_Irq_Mcan1_Irq0, "mcan1_0", */
	/* 			      MCAN1_INT0_Handler, NULL); */
	/* SamV71Core_InterruptSubscribe(Nvic_Irq_Mcan0_Irq0, "mcan0_0", */
	/* 			      MCAN1_INT0_Handler, NULL); */

	SamV71Core_EnablePeripheralClock(Pmc_PeripheralId_Mcan0);
	SamV71Core_EnablePeripheralClock(Pmc_PeripheralId_Mcan1);

	memset(msgRam, 0, sizeof(msgRam));
	Mcan_init(&mcan, Mcan_getDeviceRegisters(Mcan_Id_1));

	const Mcan_ElementSize testElementSize = Mcan_ElementSize_8;

	// possibly here is something more

	Mcan_Config conf = defaultConfig;
	/* conf.standardIdFilter.isIdRejected = TRUE; */
	/* conf.extendedIdFilter.isIdRejected = TRUE; */
	/* conf.extendedIdFilter.nonMatchingPolicy = */
	/* 	Mcan_NonMatchingPolicy_RxFifo0; */
	/* conf.extendedIdFilter.filterListSize = 0; */
	/* conf.rxFifo0.elementSize = testElementSize; */
	/* conf.txBuffer.elementSize = testElementSize; */

	ErrorCode errCode = ErrorCode_NoError;
	bool setConfResult =
		Mcan_setConfig(&mcan, &conf, CONFIG_TIMEOUT, &errCode);
	assert(setConfResult);
	assert(errCode == ErrorCode_NoError);


	// this is for comparision
	/* Mcan_Config readConfig; */
	/* Mcan_getConfig(&mcan, &readConfig); */
	/* int cmpResult = memcmp(&conf, &readConfig, sizeof(Mcan_Config)); */
	/* assert(cmpResult == 0); */

	const uint8_t bytesCount = getElementBytesCount(testElementSize);

	uint8_t txData[64];
	fillMsgData(txData, bytesCount);

	const Mcan_TxElement txElement = {
		.esiFlag = Mcan_ElementEsi_Dominant,
		.idType = Mcan_IdType_Standard,
		.frameType = Mcan_FrameType_Data,
		.id = 3,
		.marker = 0,
		.isTxEventStored = FALSE,
		.isCanFdFormatEnabled = FALSE,
		.isBitRateSwitchingEnabled = FALSE,
		.dataSize = bytesCount,
		.data = txData,
		.isInterruptEnabled = FALSE,
	};

	/* bool bufferAddResult = Mcan_txBufferAdd(&mcan, txElement, 0, &errCode); */
	/* assert(bufferAddResult); */
	/* assert(errCode == ErrorCode_NoError); */

	uint8_t pushIndex;
	bool pushResult =
		Mcan_txQueuePush(&mcan, txElement, &pushIndex, &errCode);
	assert(pushResult);
	assert(errCode == ErrorCode_NoError);

	bool result = waitForTransmissionFinished(MCAN_TEST_TIMEOUT, pushIndex);
	assert(result);

	while (true) {
		Mcan_RxFifoStatus fifoStatus;
		bool fifoStatusResult = Mcan_getRxFifoStatus(
			&mcan, Mcan_RxFifoId_0, &fifoStatus, NULL);
		assert(fifoStatusResult);
		if (fifoStatus.count > 0) {
			uint8_t rxData[64];
			Mcan_RxElement rxElement;
			rxElement.data = rxData;
			bool fifoPullResult = Mcan_rxFifoPull(
				&mcan, Mcan_RxFifoId_0, &rxElement, &errCode);

			assert(fifoPullResult);
			assert(errCode == ErrorCode_NoError);

			gotMsg();
		}
	}
}

void SamV71RtemsCanPoll(void *private_data)
{
}

void SamV71RtemsCanSend(void *private_data, const uint8_t *const data,
			const size_t length)
{
}
