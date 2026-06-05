#include "headfile.h"

/* ========================= */
/* 赛道/循迹重点调参区        */
/* ========================= */
/*
 * 模式说明：
 * 1=无赛道直行碰线停车，2=无赛道发车循迹一圈，
 * 3=无赛道发车绕8字一圈，4=无赛道发车绕8字四圈。
 *
 * 转角单位为度；正数按当前车体方向右转，负数左转。
 */

#define TRACK_MAX_SPEED                 11      /* 循迹直道最高速度 */
#define TRACK_MIN_SPEED                 5       /* 循迹弯道/基础最低速度 */
#define TRACK_INNER_MIN_SPEED           5       /* 大误差转弯时内侧轮最低速度 */
#define TRACK_K                         2       /* 循迹差速转向增益 */
#define TRACK_SPEED_K                   1       /* 误差越大时的减速系数 */
#define TRACK_TURN_LIMIT                (TRACK_MAX_SPEED - TRACK_MIN_SPEED)
#define TRACK_LOST_ERROR                10      /* 丢线时按最外侧误差找线 */

#define TRACING_MODE_COUNT              5U      /* 可选模式数量：0~4 */
#define TRACING_DOUBLE_CLICK_TICKS      60U     /* 双击确认启动的等待周期 */
#define TRACING_LINE_CONFIRM_TICKS      4U      /* 连续检测到线多少次后确认有线 */
#define TRACING_NO_LINE_CONFIRM_TICKS   10U     /* 连续丢线多少次后确认无线 */
#define TRACING_NOTIFY_TICKS            100U    /* 完成提示灯保持周期 */
#define COURSE_ANGLE_MAX_AGE_MS         250U    /* 姿态数据超过该时间未更新则暂停角度控制 */

#define COURSE_STRAIGHT_SPEED           10.0f   /* 模式1/2/3/4发车后，碰到线前的直行速度 */
#define COURSE_MODE3_START_TURN_DEG     35.0f   /* 模式3发车后，进入8字前的首次右转角度 */
#define COURSE_MODE3_RIGHT_TURN_DEG     65.0f   /* 模式3每次右半圈出线后的右转角度 */
#define COURSE_MODE3_LEFT_TURN_DEG      (-65.0f) /* 模式3每次左半圈出线后的左转角度 */
#define COURSE_MODE4_START_TURN_DEG     35.0f   /* 模式4发车后，进入8字前的首次右转角度 */
#define COURSE_MODE4_RIGHT_TURN_DEG     65.0f   /* 模式4每次右半圈出线后的右转角度 */
#define COURSE_MODE4_LEFT_TURN_DEG      (-65.0f) /* 模式4每次左半圈出线后的左转角度 */
#define COURSE_MODE2_EXIT_TARGET_OFFSET_DEG 225.0f /* 模式2第一次出线后，目标角=发车角+该角度 */
#define COURSE_MODE2_EXIT_MIN_YAW_DEG   200.0f  /* 模式2第一次出线前，车身至少要按右转方向相对发车方向转过的角度 */
#define COURSE_TURN_SETTLE_TICKS        0U      /* 转向完成后停车稳定等待周期；0=立即直行找线 */
#define COURSE_TURN_TIMEOUT_TICKS       900U    /* 角度转向最长允许周期，防止一直卡在转向阶段 */
#define COURSE_STRAIGHT_IGNORE_LINE_TICKS 40U   /* 转向后再次发车时，先忽略旧线一小段时间 */
#define COURSE_TRACE_MIN_LINE_TICKS     400U    /* 至少循迹多少周期后，才允许把丢线当作出线 */

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
	-15, -10, -7, 1, 1, 7, 10, 15
};

int8_t Last_Error = 0;

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
static uint16_t CourseTurnTicks = 0;
static uint16_t CourseStraightIgnoreLineTicks = 0;
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

static uint8_t Course_AngleFresh(void)
{
    return GY87_IsFresh(COURSE_ANGLE_MAX_AGE_MS);
}

static uint8_t Course_Mode2FirstExitAngleReady(void)
{
    float DeltaYaw;

    if(!Course_AngleFresh())
    {
        return 0;
    }

    DeltaYaw = Angle_Normalize(GY87_GetYawFast() - CourseStraightYaw);
    return (DeltaYaw >= COURSE_MODE2_EXIT_MIN_YAW_DEG) ? 1U : 0U;
}

static float Course_GetRightTurnDeg(void)
{
    return (TracingActiveMode == 4U) ?
           COURSE_MODE4_RIGHT_TURN_DEG :
           COURSE_MODE3_RIGHT_TURN_DEG;
}

static float Course_GetLeftTurnDeg(void)
{
    return (TracingActiveMode == 4U) ?
           COURSE_MODE4_LEFT_TURN_DEG :
           COURSE_MODE3_LEFT_TURN_DEG;
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

static void Course_BeginTurnTo(float TargetYaw, CourseStage_t NextStage)
{
    CourseTurnTargetYaw = Angle_Normalize(TargetYaw);
    CourseStage = NextStage;
    CourseTurnTicks = 0;
    Angle_StartTurnTo(CourseTurnTargetYaw);
}

static void Course_BeginTurn(float DeltaYaw, CourseStage_t NextStage)
{
    Course_BeginTurnTo(Angle_TargetAdd(GY87_GetYawFast(), DeltaYaw),
                       NextStage);
}

static void Course_LoadMode(uint8_t Mode)
{
    CourseGapCount = 0;
    CourseHalfIndex = 0;
    CourseLoopCount = 0;
    CourseLoopTarget = 0;
    CourseTraceLineTicks = 0;
    CourseSettleTicks = 0;
    CourseTurnTicks = 0;
    CourseStraightIgnoreLineTicks = 0;
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
            Course_BeginTurn(COURSE_MODE3_START_TURN_DEG,
                             COURSE_STAGE_TURN_RIGHT);
            break;

        case 4:
            CourseLoopTarget = 4;
            Course_BeginTurn(COURSE_MODE4_START_TURN_DEG,
                             COURSE_STAGE_TURN_RIGHT);
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
    CourseStraightIgnoreLineTicks = COURSE_STRAIGHT_IGNORE_LINE_TICKS;
    Course_ResetLineFilter();
    Angle_ResetController();
    Motor_SetSpeed(COURSE_STRAIGHT_SPEED, COURSE_STRAIGHT_SPEED);
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
    if(COURSE_TURN_SETTLE_TICKS == 0U)
    {
        Course_EnterStraightTo(CourseTurnTargetYaw);
        return;
    }

    CourseStage = COURSE_STAGE_TURN_SETTLE;
    CourseSettleTicks = COURSE_TURN_SETTLE_TICKS;
    Angle_ResetController();
    Motor_Stop();
}

static void Course_ControlStraight(uint8_t Mode)
{
    if(CourseStraightIgnoreLineTicks > 0U)
    {
        CourseStraightIgnoreLineTicks--;
        Motor_SetSpeed(COURSE_STRAIGHT_SPEED, COURSE_STRAIGHT_SPEED);
        return;
    }
    else if(Course_UpdateLineState())
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
            if(!Course_AngleFresh())
            {
                Motor_SetSpeed(COURSE_STRAIGHT_SPEED,
                               COURSE_STRAIGHT_SPEED);
                return;
            }

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

    if(CourseGapCount == 0U &&
       !Course_Mode2FirstExitAngleReady())
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
        Course_BeginTurnTo(
            Angle_TargetAdd(CourseStraightYaw,
                            COURSE_MODE2_EXIT_TARGET_OFFSET_DEG),
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
        Course_BeginTurn(Course_GetLeftTurnDeg(), COURSE_STAGE_TURN_LEFT);
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
            Course_BeginTurn(Course_GetRightTurnDeg(), COURSE_STAGE_TURN_RIGHT);
        }
    }
}

static void Course_ControlTurn(void)
{
    if(!Course_AngleFresh())
    {
        Motor_SetSpeed(0.0f, 0.0f);
        return;
    }

    if(CourseTurnTicks < COURSE_TURN_TIMEOUT_TICKS)
    {
        CourseTurnTicks++;
    }

    if(Angle_TurnTask())
    {
        Course_BeginTurnSettle();
        return;
    }

    if(CourseTurnTicks >= COURSE_TURN_TIMEOUT_TICKS)
    {
        Angle_StopTurnTask();
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
    if(TracingSelectedMode != 0U && !Course_AngleFresh())
    {
        Motor_SetSpeed(0.0f, 0.0f);
        return;
    }

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
 * 返回值：按灰度权重返回；丢线时沿上一次方向继续找线
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
			return TRACK_LOST_ERROR;
		}
		else
		{
			return -TRACK_LOST_ERROR;
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
	if(Turn > TRACK_TURN_LIMIT)
	{
		Turn = TRACK_TURN_LIMIT;
	}

	if(Turn < -TRACK_TURN_LIMIT)
	{
		Turn = -TRACK_TURN_LIMIT;
	}


	/******** 差速计算 ********/
	LeftSpeed  = BaseSpeed - Turn;

	RightSpeed = BaseSpeed + Turn;


	/******** 最低速度保护 ********/
	if(Turn > 0)
	{
		if(LeftSpeed < TRACK_INNER_MIN_SPEED)
		{
			LeftSpeed = TRACK_INNER_MIN_SPEED;
		}

		if(RightSpeed < TRACK_MIN_SPEED)
		{
			RightSpeed = TRACK_MIN_SPEED;
		}
	}
	else if(Turn < 0)
	{
		if(LeftSpeed < TRACK_MIN_SPEED)
		{
			LeftSpeed = TRACK_MIN_SPEED;
		}

		if(RightSpeed < TRACK_INNER_MIN_SPEED)
		{
			RightSpeed = TRACK_INNER_MIN_SPEED;
		}
	}
	else
	{
		if(LeftSpeed < TRACK_MIN_SPEED)
		{
			LeftSpeed = TRACK_MIN_SPEED;
		}

		if(RightSpeed < TRACK_MIN_SPEED)
		{
			RightSpeed = TRACK_MIN_SPEED;
		}
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
    int32_t TargetYawInt;
    int8_t Error;

    YawInt = (int32_t)(GY87_GetYawFast() + 0.5f);

    if(YawInt >= 360)
    {
        YawInt -= 360;
    }

    TargetYawInt =
        (int32_t)(Angle_Normalize(CourseTurnTargetYaw) + 0.5f);

    if(TargetYawInt >= 360)
    {
        TargetYawInt -= 360;
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
             "Y:%03ld TG:%03ld",
             (long)YawInt,
             (long)TargetYawInt);
    Tracing_ShowPaddedLine(3, Buf);

    snprintf(Buf,
             17,
             "E:%+03d LN:%u",
             (int)Error,
             (unsigned int)CourseLineStable);
    Tracing_ShowPaddedLine(4, Buf);
}


