#include "stm32f4xx_hal.h"
#include "main.h"

uint16_t usb_led = 0;
uint16_t timeout = 0;


void set_output(uint8_t output, uint8_t state)
{
	switch(output)
	{
//		case 0 : HAL_GPIO_WritePin(AUX5_GPIO_Port,AUX5_Pin,state); break;
//		case 1 : HAL_GPIO_WritePin(AUX6_GPIO_Port,AUX6_Pin,state); break;
//		case 2 : HAL_GPIO_WritePin(AUX7_GPIO_Port,AUX7_Pin,state); break;
//		case 3 : HAL_GPIO_WritePin(AUX8_GPIO_Port,AUX8_Pin,state); break;
		//case 4 : HAL_GPIO_WritePin(AUX9_GPIO_Port,AUX9_Pin,state); break;
	}
}

void update_usb_led(void)
{
	if(usb_led > 0)
	{
		usb_led--;
		HAL_GPIO_WritePin(USB_LED_GPIO_Port,USB_LED_Pin,1);
	}
	else
		HAL_GPIO_WritePin(USB_LED_GPIO_Port,USB_LED_Pin,0);
	
	if(timeout>0)
	{
		timeout--;
		
	}
}

void set_usb_led(void)
{
	usb_led = 1;
	timeout = 30;
}

uint8_t is_usb_connected(void)
{
	if(timeout>0)
		return 1;
	else
		return 0;
}
