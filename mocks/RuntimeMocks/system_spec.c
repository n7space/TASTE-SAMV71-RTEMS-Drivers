#include "system_spec.h"
#include <stddef.h>

#include <drivers_config.h>
#include "dataview-uniq.h"

// remote_port_names from partition partition_1
#define SYSTEM_SPEC_CUBESAT_TC_REQUEST_SIZE \
	(asn1SccMyInteger_REQUIRED_BYTES_FOR_ACN_ENCODING)
#define SYSTEM_SPEC_PONGER_PING_REQUEST_SIZE \
	(asn1SccMyInteger_REQUIRED_BYTES_FOR_ACN_ENCODING)

// remote_port_names from partition partition_2
#define SYSTEM_SPEC_GROUND_TM_REQUEST_SIZE \
	(asn1SccMyInteger_REQUIRED_BYTES_FOR_ACN_ENCODING)
#define SYSTEM_SPEC_PINGER_PONG_REQUEST_SIZE \
	(asn1SccMyInteger_REQUIRED_BYTES_FOR_ACN_ENCODING)

enum SystemBus port_to_bus_map[] = {
	BUS_INVALID_ID, BUS_BUS_1, BUS_BUS_2, BUS_BUS_1, BUS_BUS_2,
};

enum RemoteInterface bus_to_port_map[] = {
	INTERFACE_INVALID_ID,  INTERFACE_PONGER_PING, INTERFACE_CUBESAT_TC,
	INTERFACE_PINGER_PONG, INTERFACE_GROUND_TM,
};

struct PartitionBusPair port_to_partition_bus_map[] = {
	{ PARTITION_INVALID_ID, BUS_INVALID_ID },
	{ PARTITION_1, BUS_BUS_1 },
	{ PARTITION_1, BUS_BUS_2 },
	{ PARTITION_2, BUS_BUS_1 },
	{ PARTITION_2, BUS_BUS_2 },
};

enum SystemBus device_to_bus_map[SYSTEM_DEVICE_NUMBER] = {
	BUS_BUS_2, BUS_BUS_1, BUS_BUS_2, BUS_BUS_1, BUS_INVALID_ID,
};

const void *const device_configurations[SYSTEM_DEVICE_NUMBER] = {
	&pohidrv_node_1_uart4,
	&pohidrv_node_1_can0,
	&pohidrv_node_2_uart4,
	&pohidrv_node_2_can0,
	NULL,
};

const unsigned packetizer_configurations[SYSTEM_DEVICE_NUMBER] = {
	PACKETIZER_DEFAULT,	PACKETIZER_PASSTHROUGH, PACKETIZER_DEFAULT,
	PACKETIZER_PASSTHROUGH, PACKETIZER_DEFAULT,
};

int bus_message_size[SYSTEM_BUSES_NUMBER] = { 0 };

void initialize_system_spec()
{
	bus_message_size[BUS_BUS_1] =
		(SYSTEM_SPEC_PONGER_PING_REQUEST_SIZE >
		 bus_message_size[BUS_BUS_1]) ?
			SYSTEM_SPEC_PONGER_PING_REQUEST_SIZE :
			bus_message_size[BUS_BUS_1];
	bus_message_size[BUS_BUS_2] =
		(SYSTEM_SPEC_CUBESAT_TC_REQUEST_SIZE >
		 bus_message_size[BUS_BUS_2]) ?
			SYSTEM_SPEC_CUBESAT_TC_REQUEST_SIZE :
			bus_message_size[BUS_BUS_2];
	bus_message_size[BUS_BUS_1] =
		(SYSTEM_SPEC_PINGER_PONG_REQUEST_SIZE >
		 bus_message_size[BUS_BUS_1]) ?
			SYSTEM_SPEC_PINGER_PONG_REQUEST_SIZE :
			bus_message_size[BUS_BUS_1];
	bus_message_size[BUS_BUS_2] =
		(SYSTEM_SPEC_GROUND_TM_REQUEST_SIZE >
		 bus_message_size[BUS_BUS_2]) ?
			SYSTEM_SPEC_GROUND_TM_REQUEST_SIZE :
			bus_message_size[BUS_BUS_2];
}
