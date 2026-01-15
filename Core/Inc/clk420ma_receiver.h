/*
 * clk420ma_receiver.h
 *
 *  Created on: Mar 17, 2023
 *      Author: bjlli
 */

#ifndef SRC_CLK420MA_RECEIVER_H_
#define SRC_CLK420MA_RECEIVER_H_

#include "stm32f4xx_hal.h"


void init_clk420mA(GPIO_TypeDef* GPIOx_EN, uint16_t GPIO_Pin_EN);
float read_clk420mA(SPI_HandleTypeDef *hspi, GPIO_TypeDef* GPIOx_CS, uint16_t GPIO_Pin_CS);
float read_clk420mA_voltage(SPI_HandleTypeDef *hspi, GPIO_TypeDef* GPIOx_CS, uint16_t GPIO_Pin_CS);

#endif /* SRC_CLK420MA_RECEIVER_H_ */
