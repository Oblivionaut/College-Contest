#ifndef __TRACING_H
#define __TRACING_H

#include "headfile.h"

int8_t Tracing_Error_Get(void);
void Tracing_Read(void);

extern GPIO_TypeDef * GPIOx[8];
extern uint16_t GPIO_PIN_x[8];
extern uint8_t GPIO_PIN_Status[8];
void Tracing_Run(void);
int8_t Tracing_Error_Get(void);


















#endif
