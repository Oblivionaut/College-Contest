#include "headfile.h"

GPIO_TypeDef * GPIOx[8] = {GPIOC, GPIOC, GPIOC, GPIOC, GPIOC, GPIOC, GPIOD, GPIOD};
uint16_t GPIO_PIN_x[8] = {GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_2, GPIO_PIN_3};
uint8_t GPIO_PIN_Status[8] = {0};

int8_t GPIO_Error[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

//白线高电平，黑线低电平
void Tracing_Read(void)
{
	for(uint8_t i = 0; i < 8; i++)
	{
		GPIO_PIN_Status[i] = !HAL_GPIO_ReadPin(GPIOx[i], GPIO_PIN_x[i]);
	}
}//储存传感器数据，有黑线就是1

int8_t Tracing_Error_Get(void)
{
	int8_t Error = 0;
	for(int i = 0; i < 8; i++)
	{
		Error += GPIO_Error[i] * GPIO_PIN_Status[i];
	}
	return Error;
}








