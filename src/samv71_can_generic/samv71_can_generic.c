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

#include "samv71_can_generic.h"

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>

#include <Mcan/Mcan.h>
#include <Nvic/Nvic.h>
#include <Pmc/Pmc.h>
#include <Pio/Pio.h>
#include <Pmc/PmcPeripheralId.h>
#include <Utils/ErrorCode.h>

#include <Broker.h>
#include <SamV71Core.h>

#define MCAN_WAIT_TIMEOUT 100000u
#define CONFIG_TIMEOUT 1000u

static bool isMcanPckConfigured = FALSE;
static const CAN_Samv71_Rtems_Conf_T *firstConfig = NULL;

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

static void mcan_int0_Handler(void *private_data)
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
	// Use busy loop to wait for a given bit in MCAN_TXBTO, what indicates that
	// transmission is finished. Another implementation to consided is to use a
	// semaphore and interrupt.
	for (uint32_t counter = 0; counter < timeout; ++counter) {
		if (Mcan_txBufferIsTransmissionFinished(mcan, index)) {
			return true;
		}
	}
	return false;
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
	pioStatus = Pio_setPinsConfig(pio, PIO_PIN_2 | PIO_PIN_3,
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

static Pmc_PckSrc getPckSource(const CAN_Samv71_Rtems_Conf_T *const config)
{
	switch (config->pck_source) {
	case pck_clock_source_main_clock:
		return Pmc_PckSrc_Mainck;
	case pck_clock_source_plla_clock:
		return Pmc_PckSrc_Pllack;
	default:
		assert(0 &&
		       "Cannot determine PCK source, unknown configuration value");
	}
}

static void configureMcanPck(const CAN_Samv71_Rtems_Conf_T *const config)
{
	if (isMcanPckConfigured) {
		assert(firstConfig != NULL);
		assert((firstConfig->pck_source == config->pck_source) &&
		       "Cannot configure PCK5, the driver has different configuration than other.");
		assert((firstConfig->pck_prescaler == config->pck_prescaler) &&
		       "Cannot configure PCK5, the driver has different configuration than other.");
	} else {
		firstConfig = config;
		isMcanPckConfigured = TRUE;
	}

	const Pmc_PckConfig pckConfig = {
		.isEnabled = true,
		.src = getPckSource(config),
		.presc = config->pck_prescaler,
	};

	bool setCfgResult = SamV71Core_SetPckConfig(Pmc_PckId_5, &pckConfig,
						    PMC_DEFAULT_TIMEOUT, NULL);
	assert(setCfgResult);
}

static void configureMcan0(samv71_can_generic_private_data *self)
{
	configurePioCan0(&self->pioCanTx);
	configureMcanPck(self->m_config);

	SamV71Core_InterruptSubscribe(Nvic_Irq_Mcan0_Irq0, "mcan0_0",
				      mcan_int0_Handler, self);
	SamV71Core_EnablePeripheralClock(Pmc_PeripheralId_Mcan0);
	Mcan_init(&self->mcan, Mcan_getDeviceRegisters(Mcan_Id_0));
}

static void configureMcan1(samv71_can_generic_private_data *self)
{
	configurePioCan1(&self->pioCanTx);
	configureMcanPck(self->m_config);
	SamV71Core_InterruptSubscribe(Nvic_Irq_Mcan1_Irq0, "mcan1_0",
				      mcan_int0_Handler, self);
	SamV71Core_EnablePeripheralClock(Pmc_PeripheralId_Mcan1);
	Mcan_init(&self->mcan, Mcan_getDeviceRegisters(Mcan_Id_1));
}

static Mcan_Config prepareMcanConfig(samv71_can_generic_private_data *self)
{
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

	conf.nominalBitTiming.bitRatePrescaler =
		self->m_config->bit_rate_prescaler;
	conf.nominalBitTiming.synchronizationJump =
		self->m_config->synchronization_jump;
	conf.nominalBitTiming.timeSegmentAfterSamplePoint =
		self->m_config->time_segments_after_sample_point;
	conf.nominalBitTiming.timeSegmentBeforeSamplePoint =
		self->m_config->time_segments_before_sample_point;
	conf.dataBitTiming.bitRatePrescaler =
		self->m_config->bit_rate_prescaler;
	conf.dataBitTiming.synchronizationJump =
		self->m_config->synchronization_jump;
	conf.dataBitTiming.timeSegmentAfterSamplePoint =
		self->m_config->time_segments_after_sample_point;
	conf.dataBitTiming.timeSegmentBeforeSamplePoint =
		self->m_config->time_segments_before_sample_point;

	return conf;
}

static void getCanIdAndTypeFromMessageData(const uint8_t *const data,
					   const size_t length,
					   Mcan_IdType *idType, uint32_t *id)
{
	// first 29 bits are CAN-ID
	// the bit 30 determines if CAN-ID is 11-bit (standard) or 29-bit (extended)
	assert(length >= 4);
	uint32_t address = 0;
	memcpy(&address, data, sizeof(uint32_t));
	if (idType != NULL) {
		*idType = (address & 0x20000000) ? Mcan_IdType_Extended :
						   Mcan_IdType_Standard;
	}
	if (id != NULL) {
		*id = address & 0x20000000 ? address & 0x1fffffff :
					     address & 0x000007ff;
	}
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
	SamV71Core_DisableDataCacheInRegion(self->msgRam,
					    MSGRAM_BYTE_SIZE_EXPONENT - 1);
	self->m_bus_id = bus_id;
	self->m_config = device_configuration;

	if (self->m_config->can_interface == mcan_interface_mcan0) {
		configureMcan0(self);
	} else if (device_configuration->can_interface ==
		   mcan_interface_mcan1) {
		configureMcan1(self);
	} else {
		assert(0 &&
		       "unknown mcan value of can-interface in configuration");
	}

	Mcan_Config conf = prepareMcanConfig(self);

	ErrorCode errCode = ErrorCode_NoError;
	bool setConfResult =
		Mcan_setConfig(&self->mcan, &conf, CONFIG_TIMEOUT, &errCode);
	assert(setConfResult);
	assert(errCode == ErrorCode_NoError);

	if (BROKER_BUFFER_SIZE > 8) {
		Escaper_init(&self->m_escaper, self->m_tx_buffer, 8,
			     self->m_value_buffer, BROKER_BUFFER_SIZE);
	}

	const rtems_status_code status_code =
		rtems_semaphore_create(SamV71Core_GenerateNewSemaphoreName(),
				       1, // Initial value, unlocked
				       RTEMS_SIMPLE_BINARY_SEMAPHORE,
				       0, // Priority ceiling
				       &self->m_rx_semaphore);

	assert(status_code == RTEMS_SUCCESSFUL);

	rtems_task_config taskConfig = {
		.name = SamV71Core_GenerateNewTaskName(),
		.initial_priority = 1,
		.storage_area = self->m_task_buffer,
		.storage_size = Can_SAMV71_RTEMS_TASK_BUFFER_SIZE,
		.maximum_thread_local_storage_size = Can_SAMV71_RTEMS_TLS_SIZE,
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

	if (BROKER_BUFFER_SIZE > 8) {
		Escaper_start_decoder(&self->m_escaper);
	}

	while (true) {
		/// Wait for data to arrive. Semaphore will be given
		rtems_status_code obtainResult = rtems_semaphore_obtain(
			self->m_rx_semaphore, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
		assert(obtainResult == RTEMS_SUCCESSFUL);

		Mcan_RxFifoStatus fifoStatus;
		bool fifoStatusResult = Mcan_getRxFifoStatus(
			&self->mcan, Mcan_RxFifoId_0, &fifoStatus, NULL);
		assert(fifoStatusResult);
		if (fifoStatus.count > 0) {
			Mcan_RxElement rxElement;
			rxElement.data = self->m_rx_buffer;
			bool fifoPullResult =
				Mcan_rxFifoPull(&self->mcan, Mcan_RxFifoId_0,
						&rxElement, &errCode);

			assert(fifoPullResult);
			assert(errCode == ErrorCode_NoError);

			if (BROKER_BUFFER_SIZE > 8) {
				Escaper_decode_packet(&self->m_escaper,
						      self->m_bus_id,
						      self->m_rx_buffer,
						      rxElement.dataSize,
						      &Broker_receive_packet);
			} else {
				Broker_receive_packet(self->m_bus_id,
						      self->m_rx_buffer,
						      rxElement.dataSize);
			}
		}
	}
}

static void SamV71RtemsCanSendFrame(samv71_can_generic_private_data *self,
				    const Mcan_IdType idType, const uint32_t id,
				    const uint8_t *data, const uint8_t length)
{
	Mcan_TxElement txElement;
	txElement.esiFlag = Mcan_ElementEsi_Dominant;
	txElement.idType = idType;
	txElement.id = id;
	txElement.frameType = Mcan_FrameType_Data;
	txElement.marker = 0;
	txElement.isTxEventStored = FALSE;
	txElement.isCanFdFormatEnabled = FALSE;
	txElement.isBitRateSwitchingEnabled = FALSE;
	txElement.dataSize = length;
	txElement.data = data;
	txElement.isInterruptEnabled = FALSE;
	uint8_t pushIndex = 0;
	ErrorCode errCode = ErrorCode_NoError;
	bool pushResult =
		Mcan_txQueuePush(&self->mcan, txElement, &pushIndex, &errCode);
	assert(pushResult);
	assert(errCode == ErrorCode_NoError);

	bool result = waitForTransmissionFinished(&self->mcan,
						  MCAN_WAIT_TIMEOUT, pushIndex);
	assert(result);
}

void SamV71RtemsCanSend(void *private_data, const uint8_t *const data,
			const size_t length)
{
	samv71_can_generic_private_data *self =
		(samv71_can_generic_private_data *)private_data;

	if (self->m_config->address.kind == static_can_id_PRESENT) {
		Mcan_IdType idType = Mcan_IdType_Standard;
		uint32_t id = 0;
		if (self->m_config->address.u.static_can_id.kind ==
		    can_id_standard_PRESENT) {
			idType = Mcan_IdType_Standard;
			id = self->m_config->address.u.static_can_id.u
				     .can_id_standard;
		} else if (self->m_config->address.u.static_can_id.kind ==
			   can_id_extended_PRESENT) {
			idType = Mcan_IdType_Extended;
			id = self->m_config->address.u.static_can_id.u
				     .can_id_extended;
		} else {
			assert(0 &&
			       "Unknown static can address value in configuration");
		}

		if (BROKER_BUFFER_SIZE > 8) {
			size_t index = 0;
			size_t packet_length = 0;
			Escaper_start_encoder(&self->m_escaper);
			while (index < length) {
				packet_length = Escaper_encode_packet(
					&self->m_escaper, data, length, &index);
				SamV71RtemsCanSendFrame(self, idType, id,
							self->m_tx_buffer,
							packet_length);
			}
		} else {
			SamV71RtemsCanSendFrame(self, idType, id, data, length);
		}

	} else if (self->m_config->address.kind ==
		   application_control_can_id_PRESENT) {
		Mcan_IdType idType = 0;
		uint32_t id = 0;
		getCanIdAndTypeFromMessageData(data, length, &idType, &id);

		SamV71RtemsCanSendFrame(self, idType, id,
					data + sizeof(uint32_t),
					length - sizeof(uint32_t));
	} else {
		assert(0 && "Unknown address kind in configuration");
	}
}
