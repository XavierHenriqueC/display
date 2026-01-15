/*
 * modbusSlave.h
*/

#ifndef INC_MODBUSSLAVE_H_
#define INC_MODBUSSLAVE_H_

#include "modbus_crc.h"
#include "stm32f4xx_hal.h"

#define SLAVE_ID 3

#define ILLEGAL_FUNCTION       0x01
#define ILLEGAL_DATA_ADDRESS   0x02
#define ILLEGAL_DATA_VALUE     0x03

/* Buffers Modbus */
extern uint8_t RxData[256];
extern uint8_t TxData[256];

/* Banco de registradores Modbus */
extern uint16_t Holding_Registers_Database[4];

/* UART compartilhada */
extern UART_HandleTypeDef huart1;

uint8_t readHoldingRegs (void);
uint8_t writeHoldingRegs (void);

float modbus_u16_to_float32_be(uint16_t reg_hi, uint16_t reg_lo);
uint8_t modbusCRC_Check(uint8_t *buf, uint16_t len);

void modbusException (uint8_t exceptioncode);

#endif /* INC_MODBUSSLAVE_H_ */
