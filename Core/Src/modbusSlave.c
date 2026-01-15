/*
 * modbusSlave.c
*/

#include "modbusSlave.h"
#include "string.h"
#include "stdint.h"


/* Buffers Modbus */
uint8_t RxData[256];
uint8_t TxData[256];

/* Banco de registradores Modbus */
uint16_t Holding_Registers_Database[4] = {0, 0, 0, 0};

/* UART usada pelo Modbus (definida no main.c) */
extern UART_HandleTypeDef huart1;

void sendData (uint8_t *data, int size)
{
	// we will calculate the CRC in this function itself
	uint16_t crc = crc16(data, size);
	data[size] = crc&0xFF;   // CRC LOW
	data[size+1] = (crc>>8)&0xFF;  // CRC HIGH

	extern void transmit_RS485(uint8_t *dataTx, uint16_t Size);
	transmit_RS485(data, size+2);

}

void modbusException (uint8_t exceptioncode)
{
	//| SLAVE_ID | FUNCTION_CODE | Exception code | CRC     |
	//| 1 BYTE   |  1 BYTE       |    1 BYTE      | 2 BYTES |

	TxData[0] = RxData[0];       // slave ID
	TxData[1] = RxData[1]|0x80;  // adding 1 to the MSB of the function code
	TxData[2] = exceptioncode;   // Load the Exception code
	sendData(TxData, 3);         // send Data... CRC will be calculated in the function
}


uint8_t readHoldingRegs (void)
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]);  // start Register Address

	uint16_t numRegs = ((RxData[4]<<8)|RxData[5]);   // number to registers master has requested
	if ((numRegs<1)||(numRegs>125))  // maximum no. of Registers as per the PDF
	{
		modbusException (ILLEGAL_DATA_VALUE);  // send an exception
		return 0;
	}

	uint16_t endAddr = startAddr+numRegs-1;  // end Register
	if (endAddr>49)  // end Register can not be more than 49 as we only have record of 50 Registers in total
	{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}

	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

	TxData[0] = SLAVE_ID;  // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = numRegs*2;  // Byte count
	int indx = 3;  // we need to keep track of how many bytes has been stored in TxData Buffer

	for (int i=0; i<numRegs; i++)   // Load the actual data into TxData buffer
	{
		TxData[indx++] = (Holding_Registers_Database[startAddr]>>8)&0xFF;  // extract the higher byte
		TxData[indx++] = (Holding_Registers_Database[startAddr])&0xFF;   // extract the lower byte
		startAddr++;  // increment the register address
	}

	sendData(TxData, indx);  // send data... CRC will be calculated in the function itself
	return 1;   // success
}


uint8_t writeHoldingRegs(void)
{
    // start Register Address
    uint16_t startAddr = ((uint16_t)RxData[2] << 8) | (uint16_t)RxData[3];

    // number of registers requested
    uint16_t numRegs = ((uint16_t)RxData[4] << 8) | (uint16_t)RxData[5];

    // sanity check: quantity range (Modbus limita a 123 regs)
    if ((numRegs < 1) || (numRegs > 123)) {
        modbusException(ILLEGAL_DATA_VALUE);
        return 0;
    }

    // byte count must match numRegs * 2
    uint8_t byteCount = RxData[6];
    if (byteCount != (uint8_t)(numRegs * 2)) {
        modbusException(ILLEGAL_DATA_VALUE);
        return 0;
    }

    // address range checks (database com 50 regs: 0..49)
    if (startAddr >= 50) {
        modbusException(ILLEGAL_DATA_ADDRESS);
        return 0;
    }

    uint32_t endAddr32 = (uint32_t)startAddr + (uint32_t)numRegs - 1U;
    if (endAddr32 >= 50U) {
        modbusException(ILLEGAL_DATA_ADDRESS);
        return 0;
    }

    // escrever os dados: Data começa em RxData[7]
    int indx = 7; // índice em RxData
    for (uint16_t i = 0; i < numRegs; i++) {
        uint8_t hi = RxData[indx];
        uint8_t lo = RxData[indx + 1];
        Holding_Registers_Database[startAddr++] = ((uint16_t)hi << 8) | (uint16_t)lo;
        indx += 2;
    }

    // prepara resposta (função 0x10 ecoa start addr e quantidade)
    TxData[0] = SLAVE_ID;      // slave ID
    TxData[1] = RxData[1];     // function code (deve ser 0x10)
    TxData[2] = RxData[2];     // Start Addr HIGH Byte
    TxData[3] = RxData[3];     // Start Addr LOW Byte
    TxData[4] = RxData[4];     // num of Regs HIGH Byte
    TxData[5] = RxData[5];     // num of Regs LOW Byte

    sendData(TxData, 6);       // envia; CRC calculado dentro de sendData
    return 1;                  // sucesso
}


float modbus_u16_to_float32_be(uint16_t reg_hi, uint16_t reg_lo)
{
    // Monta o valor 32-bit em ordem big-endian de palavras:
    // [reg_hi (MSW)] [reg_lo (LSW)]
    uint32_t be32 = ((uint32_t)reg_hi << 16) | (uint32_t)reg_lo;

    // Reinterpreta os bits como float IEEE-754
    float f;
    memcpy(&f, &be32, sizeof(f));  // seguro quanto a aliasing
    return f;
}

uint8_t modbusCRC_Check(uint8_t *buf, uint16_t len)
{
    if (len < 2) return 0;

    uint16_t crc_calc = crc16(buf, len - 2);
    uint16_t crc_rx   = buf[len - 2] | (buf[len - 1] << 8);

    return (crc_calc == crc_rx);
}





