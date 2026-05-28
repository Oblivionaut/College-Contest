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
#define KEY_GPIO_PIN       GPIO_PIN_0

#define KEY_PRESS_LEVEL    0

#define SHORT_TIME         20      // 20ms
#define LONG_TIME          100     // 100ms
Key_State Key_Scan(void);





















#endif
