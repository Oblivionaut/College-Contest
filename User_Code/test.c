#include "headfile.h"

void OLED_SHOW_SPEED(int16_t Speed)
{
	OLED_ShowNum(0,0,Speed,3);
}

uint8_t KeyScan(void)
{
#define KEY_PIN GPIO_PIN_2
#define KEY_GPIO GPIOE
#define KEY_DOWN (HAL_GPIO_ReadPin(KEY_GPIO,KEY_PIN)==0)
	
static uint8_t sta=0,cnt=0,click=0;
static uint32_t tmr=0;
uint8_t res=0;
if(HAL_GetTick()-tmr<10)return 0;
tmr=HAL_GetTick();

switch(sta)
{
    case 0:if(KEY_DOWN){sta=1;cnt=0;}break;
    case 1:
        cnt++;
        if(!KEY_DOWN)
        {
            if(cnt>2&&cnt<80){click++;sta=2;cnt=0;}
            else {sta=0;click=0;}
        }
        else if(cnt>=80){res=3;sta=3;click=0;}
        break;
    case 2:
        cnt++;
        if(KEY_DOWN){res=2;sta=0;click=0;}
        else if(cnt>=30){res=1;sta=0;click=0;}
        break;
    case 3:if(!KEY_DOWN)sta=0;break;
}
return res;
}

void OLED_Tracing_Debug(void)
{
	char buf[17];

	/******** 第1行：灰度状态 ********/
	sprintf(buf,
			"%d%d%d%d%d%d%d%d",
			GPIO_PIN_Status[0],
			GPIO_PIN_Status[1],
			GPIO_PIN_Status[2],
			GPIO_PIN_Status[3],
			GPIO_PIN_Status[4],
			GPIO_PIN_Status[5],
			GPIO_PIN_Status[6],
			GPIO_PIN_Status[7]);

	OLED_ShowString(1, 1, buf);


	/******** 第2行：误差 ********/
	oled_printf(2, 1,
				"E:%+03d",
				Tracing_Error_Get());


	/******** 第3行：目标速度 ********/
	oled_printf(3, 1,
            "T%+03d %+03d",
            (int)MotorA.Target,
            (int)MotorB.Target);

	oled_printf(4, 1,
            "A%+03d %+03d",
            (int)MotorA.Actual,
            (int)MotorB.Actual);
}











