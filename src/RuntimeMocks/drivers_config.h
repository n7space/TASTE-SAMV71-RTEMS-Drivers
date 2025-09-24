#ifndef DRIVERS_CONFIG_H
#define DRIVERS_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

typedef uint64_t asn1SccUint64;
typedef asn1SccUint64 asn1SccUint;
typedef bool flag;

typedef enum {
    uart0 = 0,
    uart1 = 1,
    uart2 = 2,
    uart3 = 3,
    uart4 = 4
} Serial_SamV71_Rtems_Device_T;

// please use the following macros to avoid breaking code.
#define Serial_SamV71_Rtems_Device_T_uart0 uart0
#define Serial_SamV71_Rtems_Device_T_uart1 uart1
#define Serial_SamV71_Rtems_Device_T_uart2 uart2
#define Serial_SamV71_Rtems_Device_T_uart3 uart3
#define Serial_SamV71_Rtems_Device_T_uart4 uart4

#define ERR_SERIAL_SAMV71_RTEMS_DEVICE_T                51  /*uart0 | uart1 | uart2 | uart3 | uart4*/
flag Serial_SamV71_Rtems_Device_T_IsConstraintValid(const Serial_SamV71_Rtems_Device_T* pVal, int* pErrCode);

void Serial_SamV71_Rtems_Device_T_Initialize(Serial_SamV71_Rtems_Device_T* pVal);
typedef enum {
    Serial_SamV71_Rtems_Baudrate_T_b9600 = 0,
    Serial_SamV71_Rtems_Baudrate_T_b19200 = 1,
    Serial_SamV71_Rtems_Baudrate_T_b38400 = 2,
    Serial_SamV71_Rtems_Baudrate_T_b57600 = 3,
    Serial_SamV71_Rtems_Baudrate_T_b115200 = 4,
    Serial_SamV71_Rtems_Baudrate_T_b230400 = 5
} Serial_SamV71_Rtems_Baudrate_T;

// please use the following macros to avoid breaking code.
#define Serial_SamV71_Rtems_Baudrate_T_b9600 Serial_SamV71_Rtems_Baudrate_T_b9600
#define Serial_SamV71_Rtems_Baudrate_T_b19200 Serial_SamV71_Rtems_Baudrate_T_b19200
#define Serial_SamV71_Rtems_Baudrate_T_b38400 Serial_SamV71_Rtems_Baudrate_T_b38400
#define Serial_SamV71_Rtems_Baudrate_T_b57600 Serial_SamV71_Rtems_Baudrate_T_b57600
#define Serial_SamV71_Rtems_Baudrate_T_b115200 Serial_SamV71_Rtems_Baudrate_T_b115200
#define Serial_SamV71_Rtems_Baudrate_T_b230400 Serial_SamV71_Rtems_Baudrate_T_b230400

#define ERR_SERIAL_SAMV71_RTEMS_BAUDRATE_T              56  /*b9600 | b19200 | b38400 | b57600 | b115200 | b230400*/
flag Serial_SamV71_Rtems_Baudrate_T_IsConstraintValid(const Serial_SamV71_Rtems_Baudrate_T* pVal, int* pErrCode);

void Serial_SamV71_Rtems_Baudrate_T_Initialize(Serial_SamV71_Rtems_Baudrate_T* pVal);
typedef enum {
    Serial_SamV71_Rtems_Parity_T_even = 0,
    Serial_SamV71_Rtems_Parity_T_odd = 1
} Serial_SamV71_Rtems_Parity_T;

// please use the following macros to avoid breaking code.
#define Serial_SamV71_Rtems_Parity_T_even Serial_SamV71_Rtems_Parity_T_even
#define Serial_SamV71_Rtems_Parity_T_odd Serial_SamV71_Rtems_Parity_T_odd

#define ERR_SERIAL_SAMV71_RTEMS_PARITY_T                61  /*even | odd*/
flag Serial_SamV71_Rtems_Parity_T_IsConstraintValid(const Serial_SamV71_Rtems_Parity_T* pVal, int* pErrCode);

void Serial_SamV71_Rtems_Parity_T_Initialize(Serial_SamV71_Rtems_Parity_T* pVal);
/*-- Serial_SamV71_Rtems_Conf_T --------------------------------------------*/
typedef asn1SccUint Serial_SamV71_Rtems_Conf_T_bits;

typedef struct {
    unsigned long speed:1;
    unsigned long parity:1;
    unsigned long bits:1;
    unsigned long use_paritybit:1;
} Serial_SamV71_Rtems_Conf_T_exist;
typedef struct {
    Serial_SamV71_Rtems_Device_T devname;
    Serial_SamV71_Rtems_Baudrate_T speed;
    Serial_SamV71_Rtems_Parity_T parity;
    Serial_SamV71_Rtems_Conf_T_bits bits;
    flag use_paritybit;

    Serial_SamV71_Rtems_Conf_T_exist exist;

} Serial_SamV71_Rtems_Conf_T;

#endif
