#include "headfile.h"
#include "stm32f4xx_hal.h"

PID MotorA;
PID MotorB;



int16_t EncoderA_Get_CNT()
{
	int16_t Temp;
    Temp = (int16_t)__HAL_TIM_GetCounter(&htim4);
    __HAL_TIM_SetCounter(&htim4, 0);
	return Temp;
}

int16_t EncoderB_Get_CNT()
{
	int16_t Temp;
    Temp = (int16_t)__HAL_TIM_GetCounter(&htim1);
    __HAL_TIM_SetCounter(&htim1, 0);
	return -Temp;
}

void Motor_SetPWM1(int16_t PWM)
{
	if(PWM > PWM_MAX) PWM = PWM_MAX;
	if(PWM < -PWM_MAX) PWM = -PWM_MAX;

	if(PWM > 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, PWM);
	}
	else if(PWM < 0)
	{
		PWM = -PWM;

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, PWM);
	}
	else
	{
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
	}
}

void Motor_SetPWM2(int16_t PWM)
{
	if(PWM > PWM_MAX) PWM = PWM_MAX;
	if(PWM < -PWM_MAX) PWM = -PWM_MAX;

	if(PWM > 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, PWM);
	}
	else if(PWM < 0)
	{
		PWM = -PWM;

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, PWM);
	}
	else
	{
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
	}
}

void Motor_Pid(void)
{
	/************** MotorA **************/

	MotorA.Actual = EncoderA_Get_CNT();

	MotorA.Error1 = MotorA.Error0;
	MotorA.Error0 = MotorA.Target - MotorA.Actual;

	// 积分分离
	if(MotorA.Ki != 0)
	{
		if(MotorA.Error0 < 30 && MotorA.Error0 > -30)
		{
			MotorA.ErrorInt += MotorA.Error0;
		}
	}
	else
	{
		MotorA.ErrorInt = 0;
	}

	MotorA.Out =
		MotorA.Kp * MotorA.Error0 +
		MotorA.Ki * MotorA.ErrorInt +
		MotorA.Kd * (MotorA.Error0 - MotorA.Error1);

	if(MotorA.Out > PWM_MAX) MotorA.Out = PWM_MAX;
	if(MotorA.Out < -PWM_MAX) MotorA.Out = -PWM_MAX;


	/************** MotorB **************/

	MotorB.Actual = EncoderB_Get_CNT();

	MotorB.Error1 = MotorB.Error0;
	MotorB.Error0 = MotorB.Target - MotorB.Actual;

	if(MotorB.Ki != 0)
	{
		if(MotorB.Error0 < 30 && MotorB.Error0 > -30)
		{
			MotorB.ErrorInt += MotorB.Error0;
		}
	}
	else
	{
		MotorB.ErrorInt = 0;
	}

	MotorB.Out =
		MotorB.Kp * MotorB.Error0 +
		MotorB.Ki * MotorB.ErrorInt +
		MotorB.Kd * (MotorB.Error0 - MotorB.Error1);

	if(MotorB.Out > PWM_MAX) MotorB.Out = PWM_MAX;
	if(MotorB.Out < -PWM_MAX) MotorB.Out = -PWM_MAX;


	/************** 输出PWM **************/

	Motor_SetPWM1(MotorA.Out);
	Motor_SetPWM2(MotorB.Out);
}

void Motor_SetSpeed(int8_t SpeedA, int8_t SpeedB)
{
	MotorA.Target = SpeedA;
	MotorB.Target = SpeedB;
}

void PID_SET(PID * Motor, float KP, float KI, float KD)
{
	Motor->Kp = KP;
	Motor->Ki = KI;
	Motor->Kd = KD;

	Motor->Error0 = 0;
	Motor->Error1 = 0;
	Motor->ErrorInt = 0;
	Motor->Out = 0;
}