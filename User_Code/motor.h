#ifndef __MOTOR_H
#define __MOTOR_H

#include "headfile.h"
int16_t EncoderA_Get_CNT();
int16_t EncoderB_Get_CNT();
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
extern PID MotorB;
void Motor_Pid(void);
void Motor_SetSpeed(int16_t SpeedA, int16_t SpeedB);
void PID_SET(PID * Motor, float KP, float KI, float KD);














#endif

