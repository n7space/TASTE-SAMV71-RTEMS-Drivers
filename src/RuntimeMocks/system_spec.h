#ifndef SYSTEM_SPEC_H
#define SYSTEM_SPEC_H

enum RemoteInterface {
	INTERFACE_INVALID_ID,
	INTERFACE_PINGER_PONG,
	INTERFACE_PINGER_ALIVE,
	INTERFACE_PONGER_PING,
	INTERFACE_MAX_ID,
};

enum SystemPartition {
	PARTITION_INVALID_ID,
	PARTITION_1,
	PARTITION_2,
};

enum SystemBus {
	BUS_INVALID_ID,
	BUS_BUS_1,
};

enum PacketizerCfg {
	PACKETIZER_DEFAULT,
	PACKETIZER_CCSDS,
	PACKETIZER_STRICT_CCSDS,
	PACKETIZER_THIN,
	PACKETIZER_DEVICE_PROVIDED,
	PACKETIZER_PASSTHROUGH,
	PACKETIZER_MAX_ID,
};

#define SYSTEM_BUSES_NUMBER (1 + 1)

struct PartitionBusPair {
	enum SystemPartition partition;
	enum SystemBus bus;
};

extern enum SystemBus port_to_bus_map[];
extern enum RemoteInterface bus_to_port_map[];
extern struct PartitionBusPair port_to_partition_bus_map[];

enum SystemDevice {
	DEVICE_NODE_1_UART0,
	DEVICE_NODE_2_UART0,
	DEVICE_INVALID_ID,
};

#define SYSTEM_DEVICE_NUMBER (2 + 1)

extern enum SystemBus device_to_bus_map[SYSTEM_DEVICE_NUMBER];
extern const void *const device_configurations[SYSTEM_DEVICE_NUMBER];
extern const unsigned packetizer_configurations[SYSTEM_DEVICE_NUMBER];

#endif
