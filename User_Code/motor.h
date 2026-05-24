#ifndef __MOTOR_H
#define __MOTOR_H

#include "headfile.h"
int16_t Encoder_Get_CNT();
void Motor_SetPWM1(int16_t PWM);
void Motor_SetPWM2(int16_t PWM);

typedef struct
{
	float Target;
	float Actual;
	float Out;
	float Kp;
	float Ki;
	float Kd;
	float Error0;
	float Error1;
	float ErrorInt;
}PID;

extern PID MotorA;













#endif
