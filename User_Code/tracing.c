#include "headfile.h"
uint8_t Touch_Flag = 0;
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
	-7, -5, -2, 0, 0, 2, 5, 7
};

int8_t Last_Error = 0;

#define TRACING_MODE_COUNT              5U
#define TRACING_DOUBLE_CLICK_TICKS      60U
#define TRACING_LINE_CONFIRM_TICKS      4U
#define TRACING_NO_LINE_CONFIRM_TICKS   6U
#define TRACING_NOTIFY_TICKS            100U

#define COURSE_STRAIGHT_SPEED           14.0f
#define COURSE_START_TURN_DEG           25.0f
#define COURSE_RIGHT_TURN_DEG           30.0f
#define COURSE_LEFT_TURN_DEG            (-30.0f)
#define COURSE_MODE2_EXIT_TURN_DEG      10.0f
#define COURSE_TURN_SETTLE_TICKS        60U
#define COURSE_TRACE_MIN_LINE_TICKS     80U

typedef enum
{
    COURSE_STAGE_IDLE = 0,
    COURSE_STAGE_STRAIGHT_TO_LINE,
    COURSE_STAGE_TRACE_TO_GAP,
    COURSE_STAGE_TURN_RIGHT,
    COURSE_STAGE_TURN_LEFT,
    COURSE_STAGE_TURN_SETTLE,
    COURSE_STAGE_DONE
} CourseStage_t;

static volatile uint8_t TracingSelectedMode = 0;
static volatile uint8_t TracingActiveMode = 0;
static volatile uint8_t TracingClickPending = 0;

static uint16_t TracingClickWaitTicks = 0;
static uint8_t CourseLastMode = 0xFFU;
static CourseStage_t CourseStage = COURSE_STAGE_IDLE;
static uint8_t CourseLineStable = 0;
static uint8_t CourseLineCount = 0;
static uint8_t CourseNoLineCount = 0;
static uint8_t CourseGapCount = 0;
static uint8_t CourseHalfIndex = 0;
static uint8_t CourseLoopCount = 0;
static uint8_t CourseLoopTarget = 0;
static uint16_t CourseTraceLineTicks = 0;
static uint16_t CourseSettleTicks = 0;
static uint8_t CourseTraceExitReady = 0;
static uint8_t CourseStraightUseAngle = 1;
static uint16_t CourseNotifyTicks = 0;
static float CourseStraightYaw = 0.0f;
static float CourseTurnTargetYaw = 0.0f;

static uint8_t Tracing_HasLineFromStatus(void)
{
    uint8_t i;

    for(i = 0; i < 8U; i++)
    {
        if(GPIO_PIN_Status[i])
        {
            return 1;
        }
    }

    return 0;
}

static void Course_Notify(uint8_t Enable)
{
    HAL_GPIO_WritePin(GPIOD,
                      GPIO_PIN_14,
                      Enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD,
                      GPIO_PIN_15,
                      Enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Course_NotifyStart(void)
{
    CourseNotifyTicks = TRACING_NOTIFY_TICKS;
    Course_Notify(1);
}

static void Course_NotifyStop(void)
{
    CourseNotifyTicks = 0;
    Course_Notify(0);
}

static void Course_NotifyUpdate(void)
{
    if(CourseNotifyTicks > 0U)
    {
        CourseNotifyTicks--;

        if(CourseNotifyTicks == 0U)
        {
            Course_Notify(0);
        }
    }
}

static void Course_ResetLineFilter(void)
{
    Tracing_Read();
    CourseLineStable = 0;
    CourseLineCount = 0U;
    CourseNoLineCount = TRACING_NO_LINE_CONFIRM_TICKS;
    Touch_Flag = 0;
}

static uint8_t Course_UpdateLineState(void)
{
    uint8_t HasLine;

    Tracing_Read();
    HasLine = Tracing_HasLineFromStatus();

    if(HasLine)
    {
        if(CourseLineCount < TRACING_LINE_CONFIRM_TICKS)
        {
            CourseLineCount++;
        }

        CourseNoLineCount = 0;

        if(CourseLineCount >= TRACING_LINE_CONFIRM_TICKS)
        {
            CourseLineStable = 1;
        }
    }
    else
    {
        if(CourseNoLineCount < TRACING_NO_LINE_CONFIRM_TICKS)
        {
            CourseNoLineCount++;
        }

        CourseLineCount = 0;

        if(CourseNoLineCount >= TRACING_NO_LINE_CONFIRM_TICKS)
        {
            CourseLineStable = 0;
        }
    }

    Touch_Flag = CourseLineStable;
    return CourseLineStable;
}

static void Course_Finish(void)
{
    Angle_StopTurnTask();
    Motor_Stop();
    TracingSelectedMode = 0;
    TracingActiveMode = 0;
    CourseLastMode = 0;
    CourseStage = COURSE_STAGE_IDLE;
    Course_NotifyStart();
}

static void Course_BeginTurn(float DeltaYaw, CourseStage_t NextStage)
{
    CourseTurnTargetYaw = Angle_TargetAdd(GY87_GetYawFast(), DeltaYaw);
    CourseStage = NextStage;
    Angle_StartTurnTo(CourseTurnTargetYaw);
}

static void Course_LoadMode(uint8_t Mode)
{
    CourseGapCount = 0;
    CourseHalfIndex = 0;
    CourseLoopCount = 0;
    CourseLoopTarget = 0;
    CourseTraceLineTicks = 0;
    CourseSettleTicks = 0;
    CourseTraceExitReady = 0;
    CourseStraightUseAngle = 1;
    CourseStraightYaw = GY87_GetYawFast();
    CourseTurnTargetYaw = CourseStraightYaw;
    Course_NotifyStop();
    Course_ResetLineFilter();
    Angle_StopTurnTask();
    Motor_Stop();

    switch(Mode)
    {
        case 0:
            CourseStage = COURSE_STAGE_IDLE;
            break;

        case 1:
        case 2:
            CourseStage = COURSE_STAGE_STRAIGHT_TO_LINE;
            Angle_ResetController();
            break;

        case 3:
            CourseLoopTarget = 1;
            Course_BeginTurn(COURSE_START_TURN_DEG, COURSE_STAGE_TURN_RIGHT);
            break;

        case 4:
            CourseLoopTarget = 4;
            Course_BeginTurn(COURSE_START_TURN_DEG, COURSE_STAGE_TURN_RIGHT);
            break;

        default:
            CourseStage = COURSE_STAGE_IDLE;
            TracingActiveMode = 0;
            break;
    }
}

static void Course_EnterStraightTo(float TargetYaw)
{
    CourseStage = COURSE_STAGE_STRAIGHT_TO_LINE;
    CourseStraightYaw = Angle_Normalize(TargetYaw);
    CourseStraightUseAngle = 0;
    Angle_ResetController();
}

static void Course_EnterTrace(void)
{
    CourseStage = COURSE_STAGE_TRACE_TO_GAP;
    CourseTraceLineTicks = 0;
    CourseTraceExitReady = 0;
    Angle_ResetController();
}

static void Course_BeginTurnSettle(void)
{
    CourseStage = COURSE_STAGE_TURN_SETTLE;
    CourseSettleTicks = COURSE_TURN_SETTLE_TICKS;
    Angle_ResetController();
    Motor_Stop();
}

static void Course_ControlStraight(uint8_t Mode)
{
    if(Course_UpdateLineState())
    {
        if(Mode == 1U)
        {
            Course_Finish();
        }
        else
        {
            Course_EnterTrace();
        }
    }
    else
    {
        if(CourseStraightUseAngle)
        {
            Angle_DriveStraight(CourseStraightYaw, COURSE_STRAIGHT_SPEED);
        }
        else
        {
            Motor_SetSpeed(COURSE_STRAIGHT_SPEED, COURSE_STRAIGHT_SPEED);
        }
    }
}

static uint8_t Course_ShouldKeepTracing(void)
{
    uint8_t HasLine;

    HasLine = Course_UpdateLineState();

    if(HasLine)
    {
        if(CourseTraceLineTicks < COURSE_TRACE_MIN_LINE_TICKS)
        {
            CourseTraceLineTicks++;
        }

        if(CourseTraceLineTicks >= COURSE_TRACE_MIN_LINE_TICKS)
        {
            CourseTraceExitReady = 1;
        }

        return 1;
    }

    if(!CourseTraceExitReady)
    {
        return 1;
    }

    return 0;
}

static void Course_ControlMode2Trace(void)
{
    if(Course_ShouldKeepTracing())
    {
        Normal_Tracing();
        return;
    }

    CourseGapCount++;

    if(CourseGapCount >= 2U)
    {
        Course_Finish();
    }
    else
    {
        Course_BeginTurn(COURSE_MODE2_EXIT_TURN_DEG,
                         COURSE_STAGE_TURN_RIGHT);
    }
}

static void Course_ControlEightTrace(void)
{
    if(Course_ShouldKeepTracing())
    {
        Normal_Tracing();
        return;
    }

    if(CourseHalfIndex == 0U)
    {
        CourseHalfIndex = 1;
        Course_BeginTurn(COURSE_LEFT_TURN_DEG, COURSE_STAGE_TURN_LEFT);
    }
    else
    {
        CourseLoopCount++;

        if(CourseLoopCount >= CourseLoopTarget)
        {
            Course_Finish();
        }
        else
        {
            CourseHalfIndex = 0;
            Course_BeginTurn(COURSE_RIGHT_TURN_DEG, COURSE_STAGE_TURN_RIGHT);
        }
    }
}

static void Course_ControlTurn(void)
{
    if(Angle_TurnTask())
    {
        Course_BeginTurnSettle();
    }
}

static void Course_ControlTurnSettle(void)
{
    Motor_Stop();

    if(CourseSettleTicks > 0U)
    {
        CourseSettleTicks--;
        return;
    }

    Course_EnterStraightTo(CourseTurnTargetYaw);
}

static void Tracing_StartSelectedMode(void)
{
    TracingActiveMode = TracingSelectedMode;
    CourseLastMode = 0xFFU;
}

static void Tracing_SelectNextMode(void)
{
    TracingSelectedMode++;

    if(TracingSelectedMode >= TRACING_MODE_COUNT)
    {
        TracingSelectedMode = 0;
    }
}


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
 * 函数名：Normal_Tracing
 * 功能  ：寻迹主函数
 ************************************************/
void Normal_Tracing(void)
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

void Tracing_Mode_Select(uint8_t Mode)
{
    if(Mode != CourseLastMode)
    {
        Course_LoadMode(Mode);
        CourseLastMode = Mode;
    }

	switch(Mode)
	{
		case 0: //初始状态，停车模式
			Motor_Stop();
		break;
		
		case 1://无赛道直线行驶，碰线停车
            if(CourseStage == COURSE_STAGE_STRAIGHT_TO_LINE)
            {
                Course_ControlStraight(Mode);
            }
            else if(CourseStage == COURSE_STAGE_DONE)
            {
                Motor_Stop();
            }
		break;
		
		case 2://无赛道发车，一圈后停车
            if(CourseStage == COURSE_STAGE_TURN_RIGHT ||
               CourseStage == COURSE_STAGE_TURN_LEFT)
            {
                Course_ControlTurn();
            }
            else if(CourseStage == COURSE_STAGE_TURN_SETTLE)
            {
                Course_ControlTurnSettle();
            }
            else if(CourseStage == COURSE_STAGE_STRAIGHT_TO_LINE)
            {
                Course_ControlStraight(Mode);
            }
            else if(CourseStage == COURSE_STAGE_TRACE_TO_GAP)
            {
                Course_ControlMode2Trace();
            }
            else if(CourseStage == COURSE_STAGE_DONE)
            {
                Motor_Stop();
            }
		break;
		
		case 3://无赛道发车，绕8字一圈后停车
            if(CourseStage == COURSE_STAGE_TURN_RIGHT ||
               CourseStage == COURSE_STAGE_TURN_LEFT)
            {
                Course_ControlTurn();
            }
            else if(CourseStage == COURSE_STAGE_TURN_SETTLE)
            {
                Course_ControlTurnSettle();
            }
            else if(CourseStage == COURSE_STAGE_STRAIGHT_TO_LINE)
            {
                Course_ControlStraight(Mode);
            }
            else if(CourseStage == COURSE_STAGE_TRACE_TO_GAP)
            {
                Course_ControlEightTrace();
            }
            else if(CourseStage == COURSE_STAGE_DONE)
            {
                Motor_Stop();
            }
		break;
		
		case 4://无赛道发车，绕8字四圈后停车
            if(CourseStage == COURSE_STAGE_TURN_RIGHT ||
               CourseStage == COURSE_STAGE_TURN_LEFT)
            {
                Course_ControlTurn();
            }
            else if(CourseStage == COURSE_STAGE_TURN_SETTLE)
            {
                Course_ControlTurnSettle();
            }
            else if(CourseStage == COURSE_STAGE_STRAIGHT_TO_LINE)
            {
                Course_ControlStraight(Mode);
            }
            else if(CourseStage == COURSE_STAGE_TRACE_TO_GAP)
            {
                Course_ControlEightTrace();
            }
            else if(CourseStage == COURSE_STAGE_DONE)
            {
                Motor_Stop();
            }
		break;

        default:
            TracingActiveMode = 0;
            CourseLastMode = 0xFFU;
            Motor_Stop();
        break;
	}
}

void Tracing_Button_Update(void)
{
    Key_State Key;

    Course_NotifyUpdate();

    Key = Key_Scan();

    if(Key == KEY_SHORT)
    {
        if(TracingClickPending)
        {
            TracingClickPending = 0;
            TracingClickWaitTicks = 0;
            Tracing_StartSelectedMode();
            return;
        }

        TracingClickPending = 1;
        TracingClickWaitTicks = TRACING_DOUBLE_CLICK_TICKS;
    }

    if(TracingClickPending)
    {
        if(TracingClickWaitTicks > 0U)
        {
            TracingClickWaitTicks--;
        }
        else
        {
            TracingClickPending = 0;
            Tracing_SelectNextMode();
        }
    }
}

uint8_t Tracing_GetSelectedMode(void)
{
    return TracingSelectedMode;
}

uint8_t Tracing_GetActiveMode(void)
{
    return TracingActiveMode;
}

uint8_t Tracing_GetCourseStage(void)
{
    return (uint8_t)CourseStage;
}

uint8_t Tracing_GetCourseLoopCount(void)
{
    return CourseLoopCount;
}

uint8_t Tracing_GetCourseLoopTarget(void)
{
    return CourseLoopTarget;
}

uint8_t Tracing_GetLineStable(void)
{
    return CourseLineStable;
}

uint8_t Tracing_IsRunning(void)
{
    if(TracingActiveMode == 0U || CourseStage == COURSE_STAGE_DONE)
    {
        return 0;
    }

    return 1;
}

static void Tracing_ShowPaddedLine(uint8_t Line, char *Text)
{
    char Buf[17];
    uint8_t i = 0;

    while(i < 16U && Text[i] != '\0')
    {
        Buf[i] = Text[i];
        i++;
    }

    while(i < 16U)
    {
        Buf[i] = ' ';
        i++;
    }

    Buf[16] = '\0';
    OLED_ShowString(Line, 1, Buf);
}

void OLED_Tracing_Run_Display(void)
{
    char Buf[17];
    int32_t YawInt;
    int8_t Error;

    YawInt = (int32_t)(GY87_GetYawFast() + 0.5f);

    if(YawInt >= 360)
    {
        YawInt -= 360;
    }

    Error = Tracing_Error_Get();

    snprintf(Buf,
             17,
             "SEL:%u ACT:%u",
             (unsigned int)TracingSelectedMode,
             (unsigned int)TracingActiveMode);
    Tracing_ShowPaddedLine(1, Buf);

    snprintf(Buf,
             17,
             "ST:%02u L:%u %u/%u",
             (unsigned int)CourseStage,
             (unsigned int)CourseLineStable,
             (unsigned int)CourseLoopCount,
             (unsigned int)CourseLoopTarget);
    Tracing_ShowPaddedLine(2, Buf);

    snprintf(Buf,
             17,
             "Y:%03ld E:%+03d",
             (long)YawInt,
             (int)Error);
    Tracing_ShowPaddedLine(3, Buf);

    snprintf(Buf,
             17,
             "T%+03d%+03d R%+03d%+03d",
             (int)MotorA.Target,
             (int)MotorB.Target,
             (int)MotorA.Actual,
             (int)MotorB.Actual);
    Tracing_ShowPaddedLine(4, Buf);
}


