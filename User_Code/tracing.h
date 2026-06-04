#ifndef __TRACING_H
#define __TRACING_H

#include "headfile.h"

int8_t Tracing_Error_Get(void);
void Tracing_Read(void);

extern GPIO_TypeDef * GPIOx[8];
extern uint16_t GPIO_PIN_x[8];
extern uint8_t GPIO_PIN_Status[8];
void Normal_Tracing(void);
int8_t Tracing_Error_Get(void);
void Tracing_Mode_Select(uint8_t Mode);
void Tracing_Button_Update(void);
uint8_t Tracing_GetSelectedMode(void);
uint8_t Tracing_GetActiveMode(void);
uint8_t Tracing_GetCourseStage(void);
uint8_t Tracing_GetCourseLoopCount(void);
uint8_t Tracing_GetCourseLoopTarget(void);
uint8_t Tracing_GetLineStable(void);
uint8_t Tracing_IsRunning(void);
void OLED_Tracing_Run_Display(void);

extern uint8_t Touch_Flag;
















#endif
