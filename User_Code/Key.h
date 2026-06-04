#ifndef __KEY_H
#define __KEY_H

#include "stm32f4xx_hal.h"

typedef enum
{
    KEY_NONE = 0,
    KEY_SHORT,
    KEY_LONG,
    KEY_RELEASE
}Key_State;

#define KEY_GPIO_PORT      GPIOE
#define KEY_GPIO_PIN       GPIO_PIN_2

#define KEY_PRESS_LEVEL    0

#define SHORT_TIME         3       // TIM2 5ms tick, about 15ms debounce
#define LONG_TIME          100     // TIM2 5ms tick, about 500ms long press
Key_State Key_Scan(void);





















#endif
