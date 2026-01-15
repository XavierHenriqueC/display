/*
 * flash_v1.0.h
 *
 *  Created on: Mar 29, 2023
 *      Author: bjlli
 */

#ifndef INC_FLASH_SECTOR_H_
#define INC_FLASH_SECTOR_H_

#include <stdint.h>
#include <string.h>
#include "stm32f4xx_hal.h"

#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base address of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base address of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base address of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base address of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base address of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base address of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base address of Sector 6, 128 Kbytes */

uint32_t GetSector(uint32_t Address);
uint32_t Flash_Write_Data (uint32_t StartSectorAddress, uint32_t *Data, uint16_t numberofwords);


#endif /* INC_FLASH_SECTOR_H_ */
