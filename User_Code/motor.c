#include "headfile.h"
#include "stm32f4xx_hal.h"


/**************************************************
 *
 *              MG513X 竞赛级电机控制
 *
 * 主控:
 * STM32F407VET6
 *
 * 编码器:
 * MG513X
 * 13线霍尔
 * 四倍频 = 52脉冲/圈
 *
 * PID周期:
 * 5ms
 *
 * PWM:
 * TIM3
 * PSC = 8400 - 1
 * ARR = 1000 - 1
 *
 * PWM范围:
 * 0 ~ 999
 *
 **************************************************/


/**************************************************
 *
 *                参数区
 *
 **************************************************/

/* PWM最大值 */
#define PWM_MAX                999

/* PWM死区补偿 */
#define PWM_DEADZONE           160

/* 目标速度足够小时视为停止 */
#define TARGET_ZERO_EPS        0.05f

/* 目标速度大于该值才启用PWM死区补偿 */
#define TARGET_DEADZONE_ENABLE 0.85f

/* 目标为0且实际速度足够小时直接关输出 */
#define SPEED_STOP_EPS         0.8f

/* PWM输出变化限幅 */
#define PWM_RAMP_LIMIT         120

/* 积分限幅 */
#define I_LIMIT                2000

/* 积分分离范围 */
#define I_SEPARATION           40

/* 速度滤波系数 */
#define SPEED_FILTER_K         0.35f

/* 异常速度跳变保护 */
#define SPEED_JUMP_LIMIT       300

/* 堵转保护 */
#define BLOCK_PWM_THRESHOLD    700
#define BLOCK_SPEED_THRESHOLD  5
#define BLOCK_TARGET_THRESHOLD 15.0f
#define BLOCK_TIME_THRESHOLD   80


/**************************************************
 *
 *                PID对象
 *
 **************************************************/

PID MotorA;
PID MotorB;


/**************************************************
 *
 *              内部变量
 *
 **************************************************/

static float EncoderA_Filter = 0;
static float EncoderB_Filter = 0;

static int16_t Last_PWM_A = 0;
static int16_t Last_PWM_B = 0;

static uint16_t Block_Cnt_A = 0;
static uint16_t Block_Cnt_B = 0;

static void Motor_LimitOutputToTargetDirection(PID *Motor)
{
    if(Motor->Target > TARGET_ZERO_EPS &&
       Motor->Out < 0.0f)
    {
        Motor->Out = 0.0f;
        Motor->ErrorInt = 0.0f;
    }

    if(Motor->Target < -TARGET_ZERO_EPS &&
       Motor->Out > 0.0f)
    {
        Motor->Out = 0.0f;
        Motor->ErrorInt = 0.0f;
    }
}


/**************************************************
 *
 *              编码器测速
 *
 **************************************************/

int16_t EncoderA_Get_CNT(void)
{
    int16_t Temp;

    Temp = (int16_t)__HAL_TIM_GetCounter(&htim4);

    __HAL_TIM_SetCounter(&htim4, 0);

    return Temp;
}

int16_t EncoderB_Get_CNT(void)
{
    int16_t Temp;

    Temp = (int16_t)__HAL_TIM_GetCounter(&htim1);

    __HAL_TIM_SetCounter(&htim1, 0);

    return -Temp;
}


/**************************************************
 *
 *              PWM底层输出
 *
 **************************************************/

void Motor_SetPWM1(int16_t PWM)
{
    if(PWM > PWM_MAX)
        PWM = PWM_MAX;

    if(PWM < -PWM_MAX)
        PWM = -PWM_MAX;


    if(PWM > 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

        __HAL_TIM_SetCompare(&htim3,
                             TIM_CHANNEL_1,
                             PWM);
    }
    else if(PWM < 0)
    {
        PWM = -PWM;

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

        __HAL_TIM_SetCompare(&htim3,
                             TIM_CHANNEL_1,
                             PWM);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

        __HAL_TIM_SetCompare(&htim3,
                             TIM_CHANNEL_1,
                             0);
    }
}


void Motor_SetPWM2(int16_t PWM)
{
    if(PWM > PWM_MAX)
        PWM = PWM_MAX;

    if(PWM < -PWM_MAX)
        PWM = -PWM_MAX;


    if(PWM > 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

        __HAL_TIM_SetCompare(&htim3,
                             TIM_CHANNEL_2,
                             PWM);
    }
    else if(PWM < 0)
    {
        PWM = -PWM;

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

        __HAL_TIM_SetCompare(&htim3,
                             TIM_CHANNEL_2,
                             PWM);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

        __HAL_TIM_SetCompare(&htim3,
                             TIM_CHANNEL_2,
                             0);
    }
}


/**************************************************
 *
 *              PID主控制
 *
 **************************************************/

void Motor_Pid(void)
{
    float EncoderA;
    float EncoderB;

    float P_Out_A;
    float P_Out_B;

    float I_Out_A;
    float I_Out_B;


    /************** 编码器读取 **************/

    EncoderA = EncoderA_Get_CNT();

    EncoderB = EncoderB_Get_CNT();


    /************** 编码器异常保护 **************/

    if(EncoderA > SPEED_JUMP_LIMIT)
        EncoderA = SPEED_JUMP_LIMIT;

    if(EncoderA < -SPEED_JUMP_LIMIT)
        EncoderA = -SPEED_JUMP_LIMIT;

    if(EncoderB > SPEED_JUMP_LIMIT)
        EncoderB = SPEED_JUMP_LIMIT;

    if(EncoderB < -SPEED_JUMP_LIMIT)
        EncoderB = -SPEED_JUMP_LIMIT;


    /************** 一阶低通滤波 **************/

    EncoderA_Filter =
        EncoderA_Filter * (1.0f - SPEED_FILTER_K) +
        EncoderA * SPEED_FILTER_K;

    EncoderB_Filter =
        EncoderB_Filter * (1.0f - SPEED_FILTER_K) +
        EncoderB * SPEED_FILTER_K;


    /************** 实际速度 **************/

    MotorA.Actual = EncoderA_Filter;

    MotorB.Actual = EncoderB_Filter;


    /************** 误差更新 **************/

    MotorA.Error1 = MotorA.Error0;

    MotorB.Error1 = MotorB.Error0;


    MotorA.Error0 =
        MotorA.Target - MotorA.Actual;

    MotorB.Error0 =
        MotorB.Target - MotorB.Actual;


    /************** 比例项 **************/

    P_Out_A =
        MotorA.Kp * MotorA.Error0;

    P_Out_B =
        MotorB.Kp * MotorB.Error0;


    /**************************************************
     *
     * 抗积分饱和
     *
     * 输出没接近饱和才允许积分
     *
     **************************************************/

    if(P_Out_A < PWM_MAX &&
       P_Out_A > -PWM_MAX)
    {
        if(MotorA.Error0 < I_SEPARATION &&
           MotorA.Error0 > -I_SEPARATION)
        {
            MotorA.ErrorInt += MotorA.Error0;
        }
    }

    if(P_Out_B < PWM_MAX &&
       P_Out_B > -PWM_MAX)
    {
        if(MotorB.Error0 < I_SEPARATION &&
           MotorB.Error0 > -I_SEPARATION)
        {
            MotorB.ErrorInt += MotorB.Error0;
        }
    }


    /************** 积分限幅 **************/

    if(MotorA.ErrorInt > I_LIMIT)
        MotorA.ErrorInt = I_LIMIT;

    if(MotorA.ErrorInt < -I_LIMIT)
        MotorA.ErrorInt = -I_LIMIT;

    if(MotorB.ErrorInt > I_LIMIT)
        MotorB.ErrorInt = I_LIMIT;

    if(MotorB.ErrorInt < -I_LIMIT)
        MotorB.ErrorInt = -I_LIMIT;


    /************** 积分项 **************/

    I_Out_A =
        MotorA.Ki * MotorA.ErrorInt;

    I_Out_B =
        MotorB.Ki * MotorB.ErrorInt;


    /************** PI输出 **************/

    MotorA.Out =
        P_Out_A + I_Out_A;

    MotorB.Out =
        P_Out_B + I_Out_B;


    /************** 零速保持 **************/

    if(MotorA.Target < TARGET_ZERO_EPS &&
       MotorA.Target > -TARGET_ZERO_EPS &&
       MotorA.Actual < SPEED_STOP_EPS &&
       MotorA.Actual > -SPEED_STOP_EPS)
    {
        MotorA.Out = 0;

        MotorA.ErrorInt = 0;
    }

    if(MotorB.Target < TARGET_ZERO_EPS &&
       MotorB.Target > -TARGET_ZERO_EPS &&
       MotorB.Actual < SPEED_STOP_EPS &&
       MotorB.Actual > -SPEED_STOP_EPS)
    {
        MotorB.Out = 0;

        MotorB.ErrorInt = 0;
    }


    /************** 输出限幅 **************/

    if(MotorA.Out > PWM_MAX)
        MotorA.Out = PWM_MAX;

    if(MotorA.Out < -PWM_MAX)
        MotorA.Out = -PWM_MAX;

    if(MotorB.Out > PWM_MAX)
        MotorB.Out = PWM_MAX;

    if(MotorB.Out < -PWM_MAX)
        MotorB.Out = -PWM_MAX;


    /**************************************************
     *
     * 输出变化限幅
     *
     * 防止PWM突变导致小车抽搐
     *
     **************************************************/

    if(MotorA.Out - Last_PWM_A > PWM_RAMP_LIMIT)
        MotorA.Out =
            Last_PWM_A + PWM_RAMP_LIMIT;

    if(MotorA.Out - Last_PWM_A < -PWM_RAMP_LIMIT)
        MotorA.Out =
            Last_PWM_A - PWM_RAMP_LIMIT;

    if(MotorB.Out - Last_PWM_B > PWM_RAMP_LIMIT)
        MotorB.Out =
            Last_PWM_B + PWM_RAMP_LIMIT;

    if(MotorB.Out - Last_PWM_B < -PWM_RAMP_LIMIT)
        MotorB.Out =
            Last_PWM_B - PWM_RAMP_LIMIT;


    /************** 非零目标方向保护 **************/

    Motor_LimitOutputToTargetDirection(&MotorA);

    Motor_LimitOutputToTargetDirection(&MotorB);


    /************** PWM死区补偿 **************/

    if(MotorA.Target > TARGET_DEADZONE_ENABLE ||
       MotorA.Target < -TARGET_DEADZONE_ENABLE)
    {
        if(MotorA.Out > 0 && MotorA.Out < PWM_DEADZONE)
            MotorA.Out = PWM_DEADZONE;

        if(MotorA.Out < 0 && MotorA.Out > -PWM_DEADZONE)
            MotorA.Out = -PWM_DEADZONE;
    }


    if(MotorB.Target > TARGET_DEADZONE_ENABLE ||
       MotorB.Target < -TARGET_DEADZONE_ENABLE)
    {
        if(MotorB.Out > 0 && MotorB.Out < PWM_DEADZONE)
            MotorB.Out = PWM_DEADZONE;

        if(MotorB.Out < 0 && MotorB.Out > -PWM_DEADZONE)
            MotorB.Out = -PWM_DEADZONE;
    }


    /************** 最终限幅 **************/

    if(MotorA.Out > PWM_MAX)
        MotorA.Out = PWM_MAX;

    if(MotorA.Out < -PWM_MAX)
        MotorA.Out = -PWM_MAX;

    if(MotorB.Out > PWM_MAX)
        MotorB.Out = PWM_MAX;

    if(MotorB.Out < -PWM_MAX)
        MotorB.Out = -PWM_MAX;


    /**************************************************
     *
     * 堵转保护
     *
     **************************************************/

    if((MotorA.Target > BLOCK_TARGET_THRESHOLD ||
        MotorA.Target < -BLOCK_TARGET_THRESHOLD) &&
       (MotorA.Out > BLOCK_PWM_THRESHOLD ||
        MotorA.Out < -BLOCK_PWM_THRESHOLD) &&
       (MotorA.Actual < BLOCK_SPEED_THRESHOLD &&
        MotorA.Actual > -BLOCK_SPEED_THRESHOLD))
    {
        Block_Cnt_A++;
    }
    else
    {
        Block_Cnt_A = 0;
    }


    if((MotorB.Target > BLOCK_TARGET_THRESHOLD ||
        MotorB.Target < -BLOCK_TARGET_THRESHOLD) &&
       (MotorB.Out > BLOCK_PWM_THRESHOLD ||
        MotorB.Out < -BLOCK_PWM_THRESHOLD) &&
       (MotorB.Actual < BLOCK_SPEED_THRESHOLD &&
        MotorB.Actual > -BLOCK_SPEED_THRESHOLD))
    {
        Block_Cnt_B++;
    }
    else
    {
        Block_Cnt_B = 0;
    }


    /************** 堵转关闭输出 **************/

    if(Block_Cnt_A > BLOCK_TIME_THRESHOLD)
    {
        MotorA.Out = 0;

        MotorA.ErrorInt = 0;
    }

    if(Block_Cnt_B > BLOCK_TIME_THRESHOLD)
    {
        MotorB.Out = 0;

        MotorB.ErrorInt = 0;
    }


    /************** 保存历史PWM **************/

    Last_PWM_A = MotorA.Out;

    Last_PWM_B = MotorB.Out;


    /************** 最终输出 **************/

    Motor_SetPWM1(MotorA.Out);

    Motor_SetPWM2(MotorB.Out);
}


/**************************************************
 *
 *              设置目标速度
 *
 **************************************************/

void Motor_SetSpeed(float SpeedA,
                    float SpeedB)
{
    /************** 目标限幅 **************/

    if(SpeedA > 25.0f)
        SpeedA = 25.0f;

    if(SpeedA < -25.0f)
        SpeedA = -25.0f;

    if(SpeedB > 25.0f)
        SpeedB = 25.0f;

    if(SpeedB < -25.0f)
        SpeedB = -25.0f;


    /************** 小目标归零 **************/

    if(SpeedA < TARGET_ZERO_EPS &&
       SpeedA > -TARGET_ZERO_EPS)
    {
        SpeedA = 0.0f;
    }

    if(SpeedB < TARGET_ZERO_EPS &&
       SpeedB > -TARGET_ZERO_EPS)
    {
        SpeedB = 0.0f;
    }


    /************** 换向或停车时清积分 **************/

    if(SpeedA == 0.0f ||
       (SpeedA > 0.0f && MotorA.Target < 0.0f) ||
       (SpeedA < 0.0f && MotorA.Target > 0.0f))
    {
        MotorA.ErrorInt = 0;
    }

    if(SpeedB == 0.0f ||
       (SpeedB > 0.0f && MotorB.Target < 0.0f) ||
       (SpeedB < 0.0f && MotorB.Target > 0.0f))
    {
        MotorB.ErrorInt = 0;
    }


    MotorA.Target = SpeedA;

    MotorB.Target = SpeedB;
}


/**************************************************
 *
 *              PID初始化
 *
 **************************************************/

void PID_SET(PID *Motor,
             float KP,
             float KI,
             float KD)
{
    Motor->Kp = KP;

    Motor->Ki = KI;

    Motor->Kd = KD;


    Motor->Target = 0;

    Motor->Actual = 0;

    Motor->Error0 = 0;

    Motor->Error1 = 0;

    Motor->ErrorInt = 0;

    Motor->Out = 0;
}


/**************************************************
 *
 *              电机停止
 *
 **************************************************/

void Motor_Stop(void)
{
    MotorA.Target = 0;
    MotorB.Target = 0;

    MotorA.Out = 0;
    MotorB.Out = 0;

    MotorA.ErrorInt = 0;
    MotorB.ErrorInt = 0;

    EncoderA_Filter = 0;
    EncoderB_Filter = 0;

    Last_PWM_A = 0;
    Last_PWM_B = 0;

    Motor_SetPWM1(0);

    Motor_SetPWM2(0);
}
