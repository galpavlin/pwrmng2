#include "stm32f4xx_hal.h"
void LTC_init(I2C_HandleTypeDef i2chandle);
void LTC_getdata(float *temperature,float *charge,float *voltage,float *current);
void LTC_fix(void);
