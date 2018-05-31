#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

uint8_t set_rc_pwm(uint8_t channel, uint16_t duty)
{
	uint8_t check = 0;
	if(duty == 0x6900)
		return check;
	switch(channel)
	{
		case 0 : htim1.Instance->CCR1 = (uint16_t)duty+500; check = 1; break;
		case 1 : htim1.Instance->CCR2 = (uint16_t)duty+500; check = 1; break;
		case 2 : htim1.Instance->CCR3 = (uint16_t)duty+500; check = 1; break;
	}
	return check;
}

uint8_t set_pwm(uint8_t channel, uint16_t duty)
{
	uint8_t check = 0;
	if(duty == 0x6900)
		return check;
	switch(channel)
	{
		case 0 : htim3.Instance->CCR1 = (uint16_t)duty+500; check = 1; break;
		case 1 : htim3.Instance->CCR2 = (uint16_t)duty+500; check = 1; break;
		case 2 : htim3.Instance->CCR3 = (uint16_t)duty+500; check = 1; break;
		case 3 : htim3.Instance->CCR4 = (uint16_t)duty+500; check = 1; break;
		case 4 : htim2.Instance->CCR1 = (uint16_t)duty+500; check = 1; break;
		case 5 : htim2.Instance->CCR2 = (uint16_t)duty+500; check = 1; break;
		case 6 : htim2.Instance->CCR3 = (uint16_t)duty+500; check = 1; break;
		case 7 : htim2.Instance->CCR4 = (uint16_t)duty+500; check = 1; break;
	}
	return check;
}
