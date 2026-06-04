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

static int32_t OLED_Debug_ToTenth(float Value)
{
    if(Value >= 0.0f)
    {
        return (int32_t)(Value * 10.0f + 0.5f);
    }

    return (int32_t)(Value * 10.0f - 0.5f);
}

static int32_t OLED_Debug_ToCenti(float Value)
{
    if(Value >= 0.0f)
    {
        return (int32_t)(Value * 100.0f + 0.5f);
    }

    return (int32_t)(Value * 100.0f - 0.5f);
}

static void OLED_ShowPaddedLine(uint8_t Line, char *Text)
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

static void OLED_GetSignedCenti(float Value,
                                char *Sign,
                                int32_t *Whole,
                                int32_t *Frac)
{
    int32_t Centi;

    Centi = OLED_Debug_ToCenti(Value);
    *Sign = '+';

    if(Centi < 0)
    {
        *Sign = '-';
        Centi = -Centi;
    }

    *Whole = Centi / 100;
    *Frac = Centi % 100;

    if(*Whole > 99)
    {
        *Whole = 99;
        *Frac = 99;
    }
}

static void OLED_WrapHeadingTenth(int32_t *Tenth)
{
    while(*Tenth >= 3600)
    {
        *Tenth -= 3600;
    }

    while(*Tenth < 0)
    {
        *Tenth += 3600;
    }
}

static void OLED_FormatAnglePairLine(char *Buf, float Target, float Yaw)
{
    int32_t TargetTenth;
    int32_t YawTenth;

    TargetTenth = OLED_Debug_ToTenth(Target);
    YawTenth = OLED_Debug_ToTenth(Yaw);

    OLED_WrapHeadingTenth(&TargetTenth);
    OLED_WrapHeadingTenth(&YawTenth);

    snprintf(Buf,
             17,
             "T%03ld Y%03ld.%ld S%02X",
             (long)(TargetTenth / 10),
             (long)(YawTenth / 10),
             (long)(YawTenth % 10),
             (unsigned int)(GY87.Status & 0xFFU));
}

static void OLED_FormatErrorGyroLine(char *Buf, float Error, float GyroZ)
{
    int32_t ErrorTenth;
    int32_t GyroInt;
    char ErrorSign = '+';
    char GyroSign = '+';

    ErrorTenth = OLED_Debug_ToTenth(Error);

    if(ErrorTenth < 0)
    {
        ErrorSign = '-';
        ErrorTenth = -ErrorTenth;
    }

    if(GyroZ >= 0.0f)
    {
        GyroInt = (int32_t)(GyroZ + 0.5f);
    }
    else
    {
        GyroSign = '-';
        GyroInt = (int32_t)(-GyroZ + 0.5f);
    }

    if(GyroInt > 999)
    {
        GyroInt = 999;
    }

    snprintf(Buf,
             17,
             "E%c%03ld.%ld G%c%03ld",
             ErrorSign,
             (long)(ErrorTenth / 10),
             (long)(ErrorTenth % 10),
             GyroSign,
             (long)GyroInt);
}

static void OLED_FormatSpeedLine(char *Buf, char Name, float SpeedA, float SpeedB)
{
    int32_t A;
    int32_t B;
    char SignA = '+';
    char SignB = '+';

    A = OLED_Debug_ToTenth(SpeedA);
    B = OLED_Debug_ToTenth(SpeedB);

    if(A < 0)
    {
        SignA = '-';
        A = -A;
    }

    if(B < 0)
    {
        SignB = '-';
        B = -B;
    }

    snprintf(Buf,
             17,
             "%cA%c%02ld.%ld B%c%02ld.%ld",
             Name,
             SignA,
             (long)(A / 10),
             (long)(A % 10),
             SignB,
             (long)(B / 10),
             (long)(B % 10));
}

static int32_t OLED_Debug_ToInt(float Value)
{
    if(Value >= 0.0f)
    {
        return (int32_t)(Value + 0.5f);
    }

    return (int32_t)(Value - 0.5f);
}

static void OLED_FormatPwmLine(char *Buf, float PwmA, float PwmB)
{
    int32_t A;
    int32_t B;
    char SignA = '+';
    char SignB = '+';

    A = OLED_Debug_ToInt(PwmA);
    B = OLED_Debug_ToInt(PwmB);

    if(A < 0)
    {
        SignA = '-';
        A = -A;
    }

    if(B < 0)
    {
        SignB = '-';
        B = -B;
    }

    if(A > 999)
    {
        A = 999;
    }

    if(B > 999)
    {
        B = 999;
    }

    snprintf(Buf,
             17,
             "OA%c%03ld B%c%03ld",
             SignA,
             (long)A,
             SignB,
             (long)B);
}

static void OLED_FormatMotorDebugLine(char *Buf)
{
    static uint8_t Count = 0;
    static uint8_t ShowPwm = 0;

    Count++;

    if(Count >= 5U)
    {
        Count = 0;
        ShowPwm = (ShowPwm == 0U) ? 1U : 0U;
    }

    if(ShowPwm)
    {
        OLED_FormatPwmLine(Buf, MotorA.Out, MotorB.Out);
    }
    else
    {
        OLED_FormatSpeedLine(Buf, 'R', MotorA.Actual, MotorB.Actual);
    }
}

void OLED_MotorPid_Debug(void)
{
    char Buf[17];

    snprintf(Buf,
             17,
             "T%+03d%+03d",
             (int)MotorA.Target,
             (int)MotorB.Target);
    OLED_ShowPaddedLine(1, Buf);

    snprintf(Buf,
             17,
             "R%+03d%+03d",
             (int)MotorA.Actual,
             (int)MotorB.Actual);
    OLED_ShowPaddedLine(2, Buf);

    snprintf(Buf,
             17,
             "C%+03d%+03d",
             (int)EncoderA_Raw_Debug,
             (int)EncoderB_Raw_Debug);
    OLED_ShowPaddedLine(3, Buf);

    snprintf(Buf,
             17,
             "O%+04d%+04d",
             (int)MotorA.Out,
             (int)MotorB.Out);
    OLED_ShowPaddedLine(4, Buf);
}

void OLED_GY87_Data_Debug(void)
{
    char Buf[17];
    char Sign;
    char RawSign;
    char OffsetSign;
    int32_t Whole;
    int32_t Frac;
    int32_t RawWhole;
    int32_t RawFrac;
    int32_t OffsetWhole;
    int32_t OffsetFrac;
    int32_t YawTenth;
    int32_t MagTenth;
    int32_t AccCenti;

    (void)GY87_Update();

    YawTenth = OLED_Debug_ToTenth(GY87.Yaw);
    OLED_WrapHeadingTenth(&YawTenth);
    OLED_GetSignedCenti(GY87.YawRate_DPS, &Sign, &Whole, &Frac);

    snprintf(Buf,
             17,
             "Y%03ld.%ld Z%c%ld.%02ld",
             (long)(YawTenth / 10),
             (long)(YawTenth % 10),
             Sign,
             (long)Whole,
             (long)Frac);
    OLED_ShowPaddedLine(1, Buf);

    OLED_GetSignedCenti(GY87.GyroZ_Raw_DPS,
                        &RawSign,
                        &RawWhole,
                        &RawFrac);
    OLED_GetSignedCenti(GY87.GyroZ_Offset_DPS,
                        &OffsetSign,
                        &OffsetWhole,
                        &OffsetFrac);
    snprintf(Buf,
             17,
             "R%c%ld.%02ld O%c%ld.%02ld",
             RawSign,
             (long)RawWhole,
             (long)RawFrac,
             OffsetSign,
             (long)OffsetWhole,
             (long)OffsetFrac);
    OLED_ShowPaddedLine(2, Buf);

    AccCenti = OLED_Debug_ToCenti(GY87.AccNorm_g);
    if(AccCenti < 0)
    {
        AccCenti = 0;
    }

    snprintf(Buf,
             17,
             "RG%+06d A%ld.%02ld",
             (int)GY87.GyroZ,
             (long)(AccCenti / 100),
             (long)(AccCenti % 100));
    OLED_ShowPaddedLine(3, Buf);

    MagTenth = OLED_Debug_ToTenth(GY87.MagYaw);
    OLED_WrapHeadingTenth(&MagTenth);

    snprintf(Buf,
             17,
             "M%03ld.%ld S%04X",
             (long)(MagTenth / 10),
             (long)(MagTenth % 10),
             (unsigned int)GY87.Status);
    OLED_ShowPaddedLine(4, Buf);
}

void OLED_Angle_Debug(float TargetYaw)
{
    char Buf[17];
    float Target;
    float Yaw;
    float Error;

    if((GY87.Status & GY87_STATUS_MPU_OK) == 0U)
    {
        snprintf(Buf, 17, "MPU FAIL S%02X", (unsigned int)(GY87.Status & 0xFFU));
        OLED_ShowPaddedLine(1, Buf);

        snprintf(Buf,
                 17,
                 "A%02X W%02X C%02lX L%u%u",
                 (unsigned int)GY87.MpuAddr,
                 (unsigned int)GY87.MpuWhoAmI,
                 (unsigned long)(GY87.LastI2CErrorCode & 0xFFUL),
                 (HAL_GPIO_ReadPin(GY87_I2C_SCL_PORT, GY87_I2C_SCL_PIN) == GPIO_PIN_SET) ? 1U : 0U,
                 (HAL_GPIO_ReadPin(GY87_I2C_SDA_PORT, GY87_I2C_SDA_PIN) == GPIO_PIN_SET) ? 1U : 0U);
        OLED_ShowPaddedLine(2, Buf);

        OLED_FormatSpeedLine(Buf, 'T', MotorA.Target, MotorB.Target);
        OLED_ShowPaddedLine(3, Buf);

        OLED_FormatMotorDebugLine(Buf);
        OLED_ShowPaddedLine(4, Buf);
        return;
    }

    Target = Angle_Normalize(TargetYaw);
    Yaw = GY87_GetYawFast();
    Error = Angle_Error(Target, Yaw);

    OLED_FormatAnglePairLine(Buf, Target, Yaw);
    OLED_ShowPaddedLine(1, Buf);

    OLED_FormatErrorGyroLine(Buf, Error, GY87.YawRate_DPS);
    OLED_ShowPaddedLine(2, Buf);

    OLED_FormatSpeedLine(Buf, 'T', MotorA.Target, MotorB.Target);
    OLED_ShowPaddedLine(3, Buf);

    OLED_FormatMotorDebugLine(Buf);
    OLED_ShowPaddedLine(4, Buf);
}

void GY87_DebugYaw(void)
{
    // 先获取最新的Yaw角数据（会自动更新所有传感器数据）
    float fusedYaw = GY87_GetYaw();
    float magYaw = GY87.MagYaw;
    float gyroZ = GY87.GyroZ_DPS;
    
    // 正确提取整数部分和小数部分（一位小数）
    int fusedInt = (int)fusedYaw;
    int fusedDec = (int)(fusedYaw * 10) % 10;
    
    int magInt = (int)magYaw;
    int magDec = (int)(magYaw * 10) % 10;
    
    // 第1行：融合后的最终Yaw角（最关键的数据）
    OLED_ShowString(1, 1, "Fused:");
    OLED_ShowNum(1, 7, fusedInt, 3);    // 整数部分最多3位(0-359)
    OLED_ShowString(1, 10, ".");
    OLED_ShowNum(1, 11, fusedDec, 1);   // 小数部分1位
    OLED_ShowString(1, 12, "°");
    
    // 第2行：磁力计单独计算的Yaw角（用于对比）
    OLED_ShowString(2, 1, "Mag:  ");
    OLED_ShowNum(2, 7, magInt, 3);
    OLED_ShowString(2, 10, ".");
    OLED_ShowNum(2, 11, magDec, 1);
    OLED_ShowString(2, 12, "°");
    
    // 第3行：Z轴陀螺仪角速度（单位：°/s）
    OLED_ShowString(3, 1, "GyroZ:");
    OLED_ShowSignedNum(3, 7, (int16_t)(gyroZ * 10), 5);
    OLED_ShowString(3, 12, "d/s");
    
    // 第4行：原始磁力计X/Y数据（用于校准参考）
    OLED_ShowString(4, 1, "MX:");
    OLED_ShowSignedNum(4, 4, GY87.MagX, 5);
    OLED_ShowString(4, 10, "MY:");
    OLED_ShowSignedNum(4, 13, GY87.MagY, 5);
}









