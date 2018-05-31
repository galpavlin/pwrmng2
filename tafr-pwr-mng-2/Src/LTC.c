#include "stm32f4xx_hal.h"
#include "LTC2943.h"


extern I2C_HandleTypeDef hi2c1;

static I2C_HandleTypeDef handle;

uint16_t prescaler = 256;
float R_sense = 0.025;

uint8_t buff[24];
uint8_t addr = 0;
HAL_StatusTypeDef test;

void LTC_init(I2C_HandleTypeDef i2chandle)
{
	handle = i2chandle;
	uint8_t txbuff[0x19] = {0};
	
	//clear charge register
	txbuff[0] = 0x02;
	txbuff[1] = 0x00;
	txbuff[2] = 0x00;
	HAL_I2C_Master_Transmit(&handle,0xc8,txbuff,3,50);
	
	//set ctrl register
	txbuff[0] = 0x01;
	txbuff[1] = 0xFC; //set to auto, prescaler to 1024, alert pin to output, shutdown to 0
	HAL_I2C_Master_Transmit(&handle,0xc8,txbuff,2,50);	

}

void LTC_fix(void)
{
	uint8_t txbuff[0x19] = {0};
	
	//clear charge register
	txbuff[0] = 0x02;
	txbuff[1] = 0x00;
	txbuff[2] = 0x00;
	HAL_I2C_Master_Transmit(&handle,0xc8,txbuff,3,50);
	
	//set ctrl register
	txbuff[0] = 0x01;
	txbuff[1] = 0xFC; //set to auto, prescaler to 1024, alert pin to output, shutdown to 0
	HAL_I2C_Master_Transmit(&handle,0xc8,txbuff,2,50);	
}


void LTC_getdata(float *temperature,float *charge,float *voltage,float *current)
{
	uint8_t reg = 0x00;
	//test = HAL_I2C_Master_Transmit(&handle,0xc8,&reg,1,50);
	test = HAL_I2C_Master_Receive(&handle,0xc8,buff,23,500); //read all registers
	
	*voltage = (buff[0x08]<<8 | buff[0x09])*0.0010803387502861;					 									//in volts
	*temperature = (float)(0.0077821011673152*((uint16_t)buff[0x14]<<8 | buff[0x15]) - 273.15);	 	//in degrees celsius
	*charge = ((buff[0x02]<<8 | buff[0x03]))*0.68;	//in milliamphours
	*current = 7.812738425855281E-5*((buff[0x0E]<<8 | buff[0x0F])- 32767);								//in amps
	
	
}


