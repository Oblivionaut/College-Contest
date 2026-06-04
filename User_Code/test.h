#ifndef __TEST_H
#define __TEST_H

#include "stm32f4xx_hal.h"
#include "Key.h"
#include "Tracing.h"
void OLED_SHOW_SPEED(int16_t Speed);
void OLED_Tracing_Debug(void);
void GY87_DebugYaw(void);
void OLED_GY87_Data_Debug(void);
void OLED_Angle_Debug(float TargetYaw);
void OLED_MotorPid_Debug(void);

















#endif
