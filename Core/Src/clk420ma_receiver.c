/*
 * clk420ma_receiver.c
 *
 *  Created on: Mar 17, 2023
 *      Author: bjlli
 */


#include "clk420ma_receiver.h"

void init_clk420mA(GPIO_TypeDef* GPIOx_EN, uint16_t GPIO_Pin_EN){

	HAL_GPIO_WritePin(GPIOx_EN, GPIO_Pin_EN, GPIO_PIN_SET);

}

float read_clk420mA(SPI_HandleTypeDef *hspi, GPIO_TypeDef* GPIOx_CS, uint16_t GPIO_Pin_CS){

	uint8_t buffer[2];
	uint16_t current_int;
	float current_float;

	HAL_GPIO_WritePin(GPIOx_CS, GPIO_Pin_CS, GPIO_PIN_RESET);

	if(HAL_SPI_Receive(hspi, buffer, 2, 100)){
		return(255);
	}

	current_int = buffer[0] & 0x1F;
	current_int = current_int << 8;
	current_int |= buffer[1];
	current_int = current_int >> 1;


	current_float = (float) current_int/(200);

	HAL_GPIO_WritePin(GPIOx_CS, GPIO_Pin_CS, SET);

	return current_float;
}


float read_clk420mA_voltage(SPI_HandleTypeDef *hspi, GPIO_TypeDef* GPIOx_CS, uint16_t GPIO_Pin_CS){

	uint8_t buffer[2];
	uint16_t current_int;
	float voltage_float;

	HAL_GPIO_WritePin(GPIOx_CS, GPIO_Pin_CS, GPIO_PIN_RESET);

	if(HAL_SPI_Receive(hspi, buffer, 2, 100)){
		return(255);
	}

	current_int = buffer[0] & 0x1F;
	current_int = current_int << 8;
	current_int |= buffer[1];
	current_int = current_int >> 1;

	voltage_float = 16 - ((float) current_int/(200000))*4.99;

	HAL_GPIO_WritePin(GPIOx_CS, GPIO_Pin_CS, SET);


	return voltage_float;


}
