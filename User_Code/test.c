#include "headfile.h"



void OLED_SHOW_SPEED(int16_t Speed)
{
	OLED_ShowNum(0,0,Speed,3);
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











