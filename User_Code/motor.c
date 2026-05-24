#include "headfile.h"
#include "stm32f4xx_hal.h"



PID MotorA;

int16_t Encoder_Get_CNT()
{
	int16_t Temp;
    Temp = (int16_t)__HAL_TIM_GetCounter(&htim4);
    __HAL_TIM_SetCounter(&htim4, 0);
	return Temp;
}

void Motor_SetPWM1(int16_t PWM)
{
	if(PWM > 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1,PWM);
	}
	else if(PWM < 0)
	{
		PWM = -PWM;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1,PWM);
	}
	else if(PWM == 0)
	{
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1,0);
	}
}

void Motor_SetPWM2(int16_t PWM)
{

}











