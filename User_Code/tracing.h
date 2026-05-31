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

extern uint8_t Touch_Flag;
















#endif
