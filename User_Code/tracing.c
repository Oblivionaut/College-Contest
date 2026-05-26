#include "headfile.h"

GPIO_TypeDef * GPIOx[8] =
{
	GPIOC, GPIOC, GPIOC, GPIOC,
	GPIOC, GPIOC, GPIOD, GPIOD
};

uint16_t GPIO_PIN_x[8] =
{
	GPIO_PIN_8,
	GPIO_PIN_9,
	GPIO_PIN_10,
	GPIO_PIN_11,
	GPIO_PIN_12,
	GPIO_PIN_13,
	GPIO_PIN_2,
	GPIO_PIN_3
};

uint8_t GPIO_PIN_Status[8] = {0};

int8_t GPIO_Error[8] =
{
	-7, -5, -3, -1,
	1, 3, 5, 7
};

int8_t Last_Error = 0;


/************************************************
 * 函数名：Tracing_Read
 * 功能  ：读取8路灰度状态
 * 说明  ：白线高电平，黑线低电平
 ************************************************/
void Tracing_Read(void)
{
	for(uint8_t i = 0; i < 8; i++)
	{
		GPIO_PIN_Status[i] =
			!HAL_GPIO_ReadPin(GPIOx[i], GPIO_PIN_x[i]);
	}
}


/************************************************
 * 函数名：Tracing_Error_Get
 * 功能  ：获取寻迹误差
 * 返回值：-7 ~ 7
 ************************************************/
int8_t Tracing_Error_Get(void)
{
	int16_t Sum = 0;
	int8_t Count = 0;

	for(int i = 0; i < 8; i++)
	{
		if(GPIO_PIN_Status[i])
		{
			Sum += GPIO_Error[i];
			Count++;
		}
	}

	/******** 正常检测到黑线 ********/
	if(Count)
	{
		Last_Error = Sum / Count;

		return Last_Error;
	}

	/******** 丢线处理 ********/
	else
	{
		// 根据上一次方向继续找线
		if(Last_Error >= 0)
		{
			return 7;
		}
		else
		{
			return -7;
		}
	}
}


/************************************************
 * 函数名：Tracing_Run
 * 功能  ：寻迹主函数
 ************************************************/
void Tracing_Run(void)
{
	int8_t Error;

	int16_t BaseSpeed;

	int16_t Turn;

	int16_t LeftSpeed;
	int16_t RightSpeed;


	/******** 读取灰度 ********/
	Tracing_Read();


	/******** 获取误差 ********/
	Error = Tracing_Error_Get();


	/******** 动态速度 ********/
	BaseSpeed =
		TRACK_MAX_SPEED
		- abs(Error) * TRACK_SPEED_K;


	/******** 最低速度保护 ********/
	if(BaseSpeed < TRACK_MIN_SPEED)
	{
		BaseSpeed = TRACK_MIN_SPEED;
	}


	/******** 转向计算 ********/
	Turn = Error * TRACK_K;


	/******** 转向限幅 ********/
	if(Turn > (BaseSpeed - TRACK_MIN_SPEED))
	{
		Turn = BaseSpeed - TRACK_MIN_SPEED;
	}

	if(Turn < -(BaseSpeed - TRACK_MIN_SPEED))
	{
		Turn = -(BaseSpeed - TRACK_MIN_SPEED);
	}


	/******** 差速计算 ********/
	LeftSpeed  = BaseSpeed - Turn;

	RightSpeed = BaseSpeed + Turn;


	/******** 最低速度保护 ********/
	if(LeftSpeed < TRACK_MIN_SPEED)
	{
		LeftSpeed = TRACK_MIN_SPEED;
	}

	if(RightSpeed < TRACK_MIN_SPEED)
	{
		RightSpeed = TRACK_MIN_SPEED;
	}


	/******** 最大速度保护 ********/
	if(LeftSpeed > TRACK_MAX_SPEED)
	{
		LeftSpeed = TRACK_MAX_SPEED;
	}

	if(RightSpeed > TRACK_MAX_SPEED)
	{
		RightSpeed = TRACK_MAX_SPEED;
	}


	/******** 输出目标速度 ********/
	Motor_SetSpeed(
		LeftSpeed,
		RightSpeed
	);
}