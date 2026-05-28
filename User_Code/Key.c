#include "headfile.h"



Key_State Key_Scan(void)
{
    static uint16_t Key_Time = 0;
    static uint8_t Last_Key = 1;

    uint8_t Key_Now;

    Key_Now = HAL_GPIO_ReadPin(KEY_GPIO_PORT, KEY_GPIO_PIN);

    // 按下
    if(Key_Now == KEY_PRESS_LEVEL)
    {
        Key_Time++;

        if(Key_Time > LONG_TIME)
        {
            Key_Time = LONG_TIME + 1;
            return KEY_LONG;
        }
    }
    // 松开
    else
    {
        if(Last_Key == KEY_PRESS_LEVEL)
        {
            if((Key_Time > SHORT_TIME) && (Key_Time < LONG_TIME))
            {
                Key_Time = 0;
                Last_Key = Key_Now;
                return KEY_SHORT;
            }

            if(Key_Time >= LONG_TIME)
            {
                Key_Time = 0;
                Last_Key = Key_Now;
                return KEY_RELEASE;
            }
        }

        Key_Time = 0;
    }

    Last_Key = Key_Now;

    return KEY_NONE;
}





