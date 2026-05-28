#ifndef __TEST_H
#define __TEST_H

#include "stm32f4xx_hal.h"
#include "Key.h"
#include "Tracing.h"
void OLED_SHOW_SPEED(int16_t Speed);
Key_State Key_Scan(void);
void OLED_Tracing_Debug(void);

















#endif
