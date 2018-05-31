#include "stm32f4xx_hal.h"
#include "string.h"
#include "comm.h"
#include "bigcase.h"
#include "outputs.h"
#include "pwm.h"


extern float voltage,temperature,charge,current;
extern uint8_t armed,motors_sw_disable;

extern uint8_t outputs[5];
extern uint16_t pwm[8];
extern uint16_t rc_pwm[4];

uint8_t debug[15] = {0};

//reset
void reset_board(void)
{
	while(1);
}

void float2int16(float value, uint16_t prescaler, uint8_t * buffer)
{
	uint16_t temp;
	temp = value * prescaler;
	*buffer = (temp>>8)&0xFF;
	*(buffer+1) = temp&0xFF;
}

uint16_t sestavi(uint8_t * buffer)
{
	uint16_t temp;
	temp = (*buffer)<<8 | *(buffer+1);
	return temp;
}

void bigcase(uint8_t *buf, uint16_t len)
{

	RegisterCmd_t regcmd;
	uint8_t txbuf [5];
	regcmd.r = buf[0];
	
	for(uint8_t i = 0;i<15;i++)
	{
		debug[i] = buf[i];
	}
	
	/*--------------WRITE---------------*/
	if(regcmd.b.ReadWrite) //write bit set
	{
		//calculate and check CRC
		uint16_t crc_res = crc16(buf, 5);
		if(((crc_res>>8)&0xFF) == buf[5] && (crc_res&0xFF) == buf[6])
		{		
			switch(regcmd.b.Address)
			{
				case 0x03 :	for(uint8_t i = 1;i<5;i++) {
											if(buf[i] == 0x69) outputs[i] = 1;
											else if(buf[i] == 0x01) outputs[i] = 0; }
										regcmd.b.Ack = 1;
										tx_frame(&regcmd.r,1); break;
				case 0x04 :	pwm[0] = sestavi(&buf[1]);
										pwm[1] = sestavi(&buf[3]);
										regcmd.b.Ack = 1;
										tx_frame(&regcmd.r,1); break;
				case 0x05 :	pwm[2] = sestavi(&buf[1]);
										pwm[3] = sestavi(&buf[3]);
										regcmd.b.Ack = 1;
										tx_frame(&regcmd.r,1); break;
				case 0x06 :	rc_pwm[0] = sestavi(&buf[1]);
										rc_pwm[0] = sestavi(&buf[3]);
										regcmd.b.Ack = 1;
										tx_frame(&regcmd.r,1); break;
				case 0x07 :	rc_pwm[0] = sestavi(&buf[1]);
										regcmd.b.Ack = 1;
										tx_frame(&regcmd.r,1); break;
				case 0x08 :	if(buf[1] == 0x69) outputs[4] = 1;
										else outputs[4] = 0;
										regcmd.b.Ack = 1;
										tx_frame(&regcmd.r,1); break;
				case 0x09 :	if(buf[1] == 0x69)
											motors_sw_disable = 1;
										else
											motors_sw_disable = 0;
										regcmd.b.Ack = 1;
										tx_frame(&regcmd.r,1); break;
			}					
		}		
	}		
				
	
	/*--------------READ---------------*/
	else
	{
		uint16_t crc_res = crc16(buf, 1);
		if(((crc_res>>8)&0xFF) == buf[1] && (crc_res&0xFF) == buf[2])
		{
			switch(regcmd.b.Address)
			{
				case 0x00 : float2int16(voltage,1000,&txbuf[1]);
										float2int16(current,1000,&txbuf[3]);
										regcmd.b.Ack = 1;
										txbuf[0] = regcmd.r;
										tx_frame(txbuf,5); break;
				case 0x01 : float2int16(temperature,100,&txbuf[1]);
										float2int16(charge,1,&txbuf[3]);
										regcmd.b.Ack = 1;
										txbuf[0] = regcmd.r;
										tx_frame(txbuf,5); break;
				case 0x02 : if(armed) txbuf[1] = 0x69;
										else txbuf[1] = 0x01;
										txbuf[3] = 0x05;
										txbuf[4] = 0x05;
										regcmd.b.Ack = 1;
										txbuf[0] = regcmd.r;
										tx_frame(txbuf,5); break;
			}
		}
	}
}

