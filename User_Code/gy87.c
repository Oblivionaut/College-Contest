#include "headfile.h"

GY87_t GY87;

static uint8_t MPU_Buffer[14];
static uint16_t MPU6050_ActiveAddr = MPU6050_ADDR;

static float GyroX_Offset = 0.0f;
static float GyroY_Offset = 0.0f;
static float GyroZ_Offset = 0.0f;

static uint8_t FusionReady = 0;
static uint8_t AngleLocked = 0;
static uint8_t AngleTargetValid = 0;
static uint8_t AngleLastErrorValid = 0;
static uint8_t TurnTaskActive = 0;

static float AngleLastTarget = 0.0f;
static float AngleLastError = 0.0f;
static float AngleLastTurnSpeed = 0.0f;
static float TurnTaskTarget = 0.0f;
static volatile uint8_t Gy87_UpdateBusy = 0;

static float Gy87_Abs(float Value)
{
    return (Value < 0.0f) ? -Value : Value;
}

static float Gy87_Clamp(float Value, float Min, float Max)
{
    if(Value > Max)
    {
        return Max;
    }

    if(Value < Min)
    {
        return Min;
    }

    return Value;
}

static int16_t Gy87_MakeInt16(uint8_t High, uint8_t Low)
{
    return (int16_t)(((uint16_t)High << 8) | (uint16_t)Low);
}

float Angle_Normalize(float Angle)
{
    while(Angle >= 360.0f)
    {
        Angle -= 360.0f;
    }

    while(Angle < 0.0f)
    {
        Angle += 360.0f;
    }

    return Angle;
}

float Angle_Error(float Target, float Current)
{
    float Error;

    Error = Angle_Normalize(Target) - Angle_Normalize(Current);

    while(Error > 180.0f)
    {
        Error -= 360.0f;
    }

    while(Error < -180.0f)
    {
        Error += 360.0f;
    }

    return Error;
}

float Angle_TargetAdd(float StartYaw, float DeltaYaw)
{
    return Angle_Normalize(StartYaw + DeltaYaw);
}

static void Gy87_SetStatus(uint16_t Flag, uint8_t Enable)
{
    if(Enable)
    {
        GY87.Status |= Flag;
    }
    else
    {
        GY87.Status &= (uint16_t)(~Flag);
    }
}

static HAL_StatusTypeDef Gy87_MemWrite(uint16_t DevAddr,
                                       uint8_t Reg,
                                       uint8_t *Data,
                                       uint16_t Len)
{
    HAL_StatusTypeDef Status;

    Status = HAL_I2C_Mem_Write(&GY87_I2C_HANDLE,
                               DevAddr,
                               Reg,
                               I2C_MEMADD_SIZE_8BIT,
                               Data,
                               Len,
                               GY87_I2C_TIMEOUT_MS);

    GY87.LastI2CStatus = (uint8_t)Status;
    GY87.LastI2CErrorCode = GY87_I2C_HANDLE.ErrorCode;

    if(Status != HAL_OK)
    {
        GY87.I2CErrorCount++;
    }

    return Status;
}

static HAL_StatusTypeDef Gy87_MemRead(uint16_t DevAddr,
                                      uint8_t Reg,
                                      uint8_t *Data,
                                      uint16_t Len)
{
    HAL_StatusTypeDef Status;

    Status = HAL_I2C_Mem_Read(&GY87_I2C_HANDLE,
                              DevAddr,
                              Reg,
                              I2C_MEMADD_SIZE_8BIT,
                              Data,
                              Len,
                              GY87_I2C_TIMEOUT_MS);

    GY87.LastI2CStatus = (uint8_t)Status;
    GY87.LastI2CErrorCode = GY87_I2C_HANDLE.ErrorCode;

    if(Status != HAL_OK)
    {
        GY87.I2CErrorCount++;
    }

    return Status;
}

void GY87_WriteReg(uint16_t DevAddr, uint8_t Reg, uint8_t Data)
{
    (void)Gy87_MemWrite(DevAddr, Reg, &Data, 1U);
}

void GY87_ReadRegs(uint16_t DevAddr,
                   uint8_t Reg,
                   uint8_t *Buffer,
                   uint8_t Len)
{
    if(Gy87_MemRead(DevAddr, Reg, Buffer, Len) != HAL_OK)
    {
        memset(Buffer, 0, Len);
    }
}

static uint8_t Gy87_DeviceReady(uint16_t DevAddr)
{
    HAL_StatusTypeDef Status;

    Status = HAL_I2C_IsDeviceReady(&GY87_I2C_HANDLE,
                                   DevAddr,
                                   2U,
                                   GY87_I2C_TIMEOUT_MS);

    GY87.LastI2CStatus = (uint8_t)Status;
    GY87.LastI2CErrorCode = GY87_I2C_HANDLE.ErrorCode;

    if(Status != HAL_OK)
    {
        GY87.I2CErrorCount++;
        return 0;
    }

    return 1;
}

static uint8_t GY87_IsMpuWhoValid(uint8_t Who)
{
    if(Who == 0x68U || Who == 0x69U)
    {
        return 1;
    }

#if GY87_ACCEPT_UNKNOWN_MPU_WHO
    if(Who != 0x00U && Who != 0xFFU)
    {
        return 1;
    }
#endif

    return 0;
}

static uint8_t GY87_SelectMpuAddress(void)
{
    uint8_t i;
    uint8_t Who;
    const uint16_t AddrList[2] = {MPU6050_ADDR, MPU6050_ADDR_ALT};

    for(i = 0; i < 2U; i++)
    {
        MPU6050_ActiveAddr = AddrList[i];
        GY87.MpuAddr = (uint8_t)(AddrList[i] >> 1);
        Who = 0;

        if(Gy87_DeviceReady(AddrList[i]) &&
           Gy87_MemRead(AddrList[i], 0x75U, &Who, 1U) == HAL_OK)
        {
            GY87.MpuWhoAmI = Who;

            if(GY87_IsMpuWhoValid(Who))
            {
                Gy87_SetStatus(GY87_STATUS_MPU_OK, 1);
                return 1;
            }
        }
    }

    Gy87_SetStatus(GY87_STATUS_MPU_OK, 0);
    return 0;
}

static uint8_t MPU6050_ReadRaw(void)
{
    if(Gy87_MemRead(MPU6050_ActiveAddr, 0x3BU, MPU_Buffer, 14U) != HAL_OK)
    {
        Gy87_SetStatus(GY87_STATUS_MPU_OK, 0);
        return 0;
    }

    GY87.AccX = Gy87_MakeInt16(MPU_Buffer[0], MPU_Buffer[1]);
    GY87.AccY = Gy87_MakeInt16(MPU_Buffer[2], MPU_Buffer[3]);
    GY87.AccZ = Gy87_MakeInt16(MPU_Buffer[4], MPU_Buffer[5]);

    GY87.Temperature_C =
        ((float)Gy87_MakeInt16(MPU_Buffer[6], MPU_Buffer[7]) / 340.0f) + 36.53f;

    GY87.GyroX = Gy87_MakeInt16(MPU_Buffer[8], MPU_Buffer[9]);
    GY87.GyroY = Gy87_MakeInt16(MPU_Buffer[10], MPU_Buffer[11]);
    GY87.GyroZ = Gy87_MakeInt16(MPU_Buffer[12], MPU_Buffer[13]);

    Gy87_SetStatus(GY87_STATUS_MPU_OK, 1);
    return 1;
}

static void MPU6050_ApplyScale(void)
{
    GY87.AccX_g = (float)GY87.AccX / GY87_ACCEL_SENS;
    GY87.AccY_g = (float)GY87.AccY / GY87_ACCEL_SENS;
    GY87.AccZ_g = (float)GY87.AccZ / GY87_ACCEL_SENS;

    GY87.AccNorm_g =
        sqrtf(GY87.AccX_g * GY87.AccX_g +
              GY87.AccY_g * GY87.AccY_g +
              GY87.AccZ_g * GY87.AccZ_g);

    GY87.GyroX_DPS = ((float)GY87.GyroX / GY87_GYRO_SENS) - GyroX_Offset;
    GY87.GyroY_DPS = ((float)GY87.GyroY / GY87_GYRO_SENS) - GyroY_Offset;
    GY87.GyroZ_DPS = ((float)GY87.GyroZ / GY87_GYRO_SENS) - GyroZ_Offset;

    if(Gy87_Abs(GY87.GyroZ_DPS) < GY87_GYRO_DEADBAND_DPS)
    {
        GY87.GyroZ_DPS = 0.0f;
    }

    GY87.YawRate_DPS = GY87.GyroZ_DPS * GY87_GYRO_Z_SIGN;

    GY87.Roll =
        atan2f(GY87.AccY_g, GY87.AccZ_g) * 57.2957795f;

    GY87.Pitch =
        atan2f(-GY87.AccX_g,
               sqrtf(GY87.AccY_g * GY87.AccY_g +
                     GY87.AccZ_g * GY87.AccZ_g)) * 57.2957795f;
}

void MPU6050_Read(void)
{
    if(MPU6050_ReadRaw())
    {
        MPU6050_ApplyScale();
    }
}

static uint8_t GY87_MPU_Init(void)
{
    uint8_t Who = 0;

    GY87.MpuAddr = (uint8_t)(MPU6050_ActiveAddr >> 1);

    GY87_WriteReg(MPU6050_ActiveAddr, 0x6BU, 0x80U);
    HAL_Delay(100);

    GY87_WriteReg(MPU6050_ActiveAddr, 0x6BU, 0x01U);
    GY87_WriteReg(MPU6050_ActiveAddr, 0x6CU, 0x00U);
    GY87_WriteReg(MPU6050_ActiveAddr, 0x19U, GY87_MPU_SMPLRT_DIV);
    GY87_WriteReg(MPU6050_ActiveAddr, 0x1AU, GY87_MPU_DLPF_CFG);
    GY87_WriteReg(MPU6050_ActiveAddr, 0x1BU, GY87_MPU_GYRO_CONFIG);
    GY87_WriteReg(MPU6050_ActiveAddr, 0x1CU, GY87_MPU_ACCEL_CONFIG);

    HAL_Delay(20);

    if(Gy87_MemRead(MPU6050_ActiveAddr, 0x75U, &Who, 1U) == HAL_OK)
    {
        GY87.MpuWhoAmI = Who;

        if(GY87_IsMpuWhoValid(Who))
        {
            Gy87_SetStatus(GY87_STATUS_MPU_OK, 1);
            return 1;
        }
    }

    Gy87_SetStatus(GY87_STATUS_MPU_OK, 0);
    return 0;
}

static void GY87_GyroCalibrate(void)
{
    uint16_t i;
    uint16_t Count = 0;
    float SumX = 0.0f;
    float SumY = 0.0f;
    float SumZ = 0.0f;

    GyroX_Offset = 0.0f;
    GyroY_Offset = 0.0f;
    GyroZ_Offset = 0.0f;

    for(i = 0; i < 20U; i++)
    {
        (void)MPU6050_ReadRaw();
        HAL_Delay(GY87_GYRO_CALIB_DELAY_MS);
    }

    for(i = 0; i < GY87_GYRO_CALIB_SAMPLES; i++)
    {
        if(MPU6050_ReadRaw())
        {
            SumX += (float)GY87.GyroX / GY87_GYRO_SENS;
            SumY += (float)GY87.GyroY / GY87_GYRO_SENS;
            SumZ += (float)GY87.GyroZ / GY87_GYRO_SENS;
            Count++;
        }

        HAL_Delay(GY87_GYRO_CALIB_DELAY_MS);
    }

    if(Count > 0U)
    {
        GyroX_Offset = SumX / (float)Count;
        GyroY_Offset = SumY / (float)Count;
        GyroZ_Offset = SumZ / (float)Count;
    }

    MPU6050_ApplyScale();
}

void GY87_Init(void)
{
    memset(&GY87, 0, sizeof(GY87));

    Gy87_UpdateBusy = 0;
    FusionReady = 0;
    AngleLocked = 0;
    AngleTargetValid = 0;
    AngleLastErrorValid = 0;
    TurnTaskActive = 0;
    AngleLastTurnSpeed = 0.0f;
    MPU6050_ActiveAddr = MPU6050_ADDR;
    GY87.MpuAddr = (uint8_t)(MPU6050_ADDR >> 1);

    if(!GY87_SelectMpuAddress() || !GY87_MPU_Init())
    {
        GY87.LastUpdateMs = HAL_GetTick();
        return;
    }

    GY87_GyroCalibrate();
    GY87_SetYaw(0.0f);
    GY87.LastUpdateMs = HAL_GetTick();
}

uint8_t GY87_Update(void)
{
    uint32_t NowMs;
    uint32_t ElapsedMs;
    float Dt;

    if(Gy87_UpdateBusy)
    {
        return 0;
    }

    Gy87_UpdateBusy = 1;
    NowMs = HAL_GetTick();

    if((GY87.Status & GY87_STATUS_MPU_OK) == 0U &&
       GY87.UpdateCount == 0U)
    {
        Gy87_UpdateBusy = 0;
        return 0;
    }

    if(GY87.UpdateCount > 0U)
    {
        ElapsedMs = NowMs - GY87.LastUpdateMs;

        if(ElapsedMs < GY87_UPDATE_GUARD_MS)
        {
            Gy87_UpdateBusy = 0;
            return 0;
        }
    }

#if GY87_USE_FIXED_DT
    Dt = GY87_UPDATE_PERIOD_S;
#else
    if(GY87.UpdateCount == 0U)
    {
        Dt = GY87_UPDATE_PERIOD_S;
    }
    else
    {
        Dt = (float)(NowMs - GY87.LastUpdateMs) * 0.001f;
        Dt = Gy87_Clamp(Dt, 0.001f, GY87_UPDATE_MAX_DT_S);
    }
#endif

    if(!MPU6050_ReadRaw())
    {
        Gy87_UpdateBusy = 0;
        return 0;
    }

    MPU6050_ApplyScale();

    GY87.Yaw = Angle_Normalize(GY87.Yaw + GY87.YawRate_DPS * Dt);

#if GY87_GYRO_STATIC_LEARN_EN
    if(Gy87_Abs(GY87.YawRate_DPS) < GY87_GYRO_STATIC_DPS &&
       Gy87_Abs(GY87.AccNorm_g - 1.0f) < GY87_STATIC_ACC_TOL_G)
    {
        GyroZ_Offset += GY87.GyroZ_DPS * GY87_GYRO_BIAS_LEARN_ALPHA;
    }
#endif

    FusionReady = 1;
    Gy87_SetStatus(GY87_STATUS_FUSION_READY, 1);

    GY87.LastUpdateMs = NowMs;
    GY87.UpdateCount++;

    Gy87_UpdateBusy = 0;
    return 1;
}

float GY87_GetYaw(void)
{
    (void)GY87_Update();
    return GY87.Yaw;
}

float GY87_GetYawFast(void)
{
    return GY87.Yaw;
}

void GY87_SetYaw(float Yaw)
{
    GY87.Yaw = Angle_Normalize(Yaw);
    FusionReady = 1;
    Gy87_SetStatus(GY87_STATUS_FUSION_READY, 1);
    GY87.LastUpdateMs = HAL_GetTick();
}

uint8_t GY87_ResetYawToMag(void)
{
    GY87_SetYaw(0.0f);
    GY87.MagYaw = GY87.Yaw;
    return 1;
}

uint8_t GY87_IsReady(void)
{
    if((GY87.Status & GY87_STATUS_MPU_OK) == 0U ||
       GY87.UpdateCount == 0U)
    {
        return 0;
    }

    return (FusionReady != 0U) ? 1U : 0U;
}

void HMC5883_Read(void)
{
}

float HMC5883_GetYaw(void)
{
    GY87.MagYaw = GY87.Yaw;
    return GY87.MagYaw;
}

void Angle_ResetController(void)
{
    AngleLocked = 0;
    AngleTargetValid = 0;
    AngleLastErrorValid = 0;
    AngleLastTurnSpeed = 0.0f;
}

uint8_t Angle_IsLocked(void)
{
    return AngleLocked;
}

float Angle_CalcHoldSpeed(float TargetYaw)
{
    float Error;
    float AbsError;
    float TurnSpeed;
    float BaseSpeed;
    float DampSpeed;
    float TargetNorm;
    float MaxTurnSpeed;

    TargetNorm = Angle_Normalize(TargetYaw);

    if((GY87.Status & GY87_STATUS_MPU_OK) == 0U)
    {
        AngleLocked = 1;
        AngleLastTurnSpeed = 0.0f;
        return 0.0f;
    }

    if(!AngleTargetValid ||
       Gy87_Abs(Angle_Error(TargetNorm, AngleLastTarget)) > 0.05f)
    {
        AngleLocked = 0;
        AngleLastTurnSpeed = 0.0f;
        AngleLastTarget = TargetNorm;
        AngleTargetValid = 1;
        AngleLastErrorValid = 0;
    }

    Error = Angle_Error(TargetNorm, GY87.Yaw);
    AbsError = Gy87_Abs(Error);

    if(AngleLastErrorValid &&
       ((Error > 0.0f && AngleLastError < 0.0f) ||
        (Error < 0.0f && AngleLastError > 0.0f)) &&
       AbsError < ANGLE_CROSS_LOCK_DEG)
    {
        AngleLocked = 1;
        AngleLastError = Error;
        AngleLastTurnSpeed = 0.0f;
        return 0.0f;
    }

    AngleLastError = Error;
    AngleLastErrorValid = 1;

    if(AngleLocked)
    {
        if(AbsError > ANGLE_LOCK_OUT)
        {
            AngleLocked = 0;
        }
        else
        {
            AngleLastTurnSpeed = 0.0f;
            return 0.0f;
        }
    }

    if(AbsError < ANGLE_LOCK_IN &&
       Gy87_Abs(GY87.YawRate_DPS) < ANGLE_GYRO_LOCK)
    {
        AngleLocked = 1;
        AngleLastTurnSpeed = 0.0f;
        return 0.0f;
    }

    BaseSpeed = Error * ANGLE_KP;
    DampSpeed = GY87.YawRate_DPS * ANGLE_KD;
    DampSpeed = Gy87_Clamp(DampSpeed,
                           -ANGLE_DAMP_LIMIT,
                           ANGLE_DAMP_LIMIT);

    TurnSpeed = (BaseSpeed - DampSpeed) * ANGLE_OUTPUT_SIGN;

    if(BaseSpeed > 0.0f && TurnSpeed < 0.0f)
    {
        TurnSpeed = 0.001f;
    }

    if(BaseSpeed < 0.0f && TurnSpeed > 0.0f)
    {
        TurnSpeed = -0.001f;
    }

    if(AbsError > ANGLE_LOCK_OUT)
    {
        if(TurnSpeed > 0.0f && TurnSpeed < ANGLE_MIN_TURN_SPEED)
        {
            TurnSpeed = ANGLE_MIN_TURN_SPEED;
        }

        if(TurnSpeed < 0.0f && TurnSpeed > -ANGLE_MIN_TURN_SPEED)
        {
            TurnSpeed = -ANGLE_MIN_TURN_SPEED;
        }
    }

    MaxTurnSpeed = ANGLE_MAX_TURN_SPEED;

    if(AbsError < ANGLE_SLOW_DOWN_DEG)
    {
        MaxTurnSpeed =
            ANGLE_NEAR_MAX_TURN_SPEED +
            (ANGLE_MAX_TURN_SPEED - ANGLE_NEAR_MAX_TURN_SPEED) *
            (AbsError / ANGLE_SLOW_DOWN_DEG);

        MaxTurnSpeed = Gy87_Clamp(MaxTurnSpeed,
                                  ANGLE_NEAR_MAX_TURN_SPEED,
                                  ANGLE_MAX_TURN_SPEED);
    }

    TurnSpeed = Gy87_Clamp(TurnSpeed,
                           -MaxTurnSpeed,
                           MaxTurnSpeed);

    if(TurnSpeed - AngleLastTurnSpeed > ANGLE_SPEED_RAMP)
    {
        TurnSpeed = AngleLastTurnSpeed + ANGLE_SPEED_RAMP;
    }

    if(TurnSpeed - AngleLastTurnSpeed < -ANGLE_SPEED_RAMP)
    {
        TurnSpeed = AngleLastTurnSpeed - ANGLE_SPEED_RAMP;
    }

    AngleLastTurnSpeed = TurnSpeed;
    return TurnSpeed;
}

void Angle_Hold(float TargetYaw)
{
    float TurnSpeed;

    TurnSpeed = Angle_CalcHoldSpeed(TargetYaw);

    if(AngleLocked)
    {
        Motor_SetSpeed(0.0f, 0.0f);
    }
    else
    {
        Motor_SetSpeed(TurnSpeed, -TurnSpeed);
    }
}

void Angle_DriveStraight(float TargetYaw, float BaseSpeed)
{
    float Error;
    float AbsError;
    float Correction;

    if((GY87.Status & GY87_STATUS_MPU_OK) == 0U)
    {
        Motor_SetSpeed(0.0f, 0.0f);
        return;
    }

    Error = Angle_Error(TargetYaw, GY87.Yaw);
    AbsError = Gy87_Abs(Error);

    if(AbsError < ANGLE_STRAIGHT_DEADBAND_DEG)
    {
        Error = 0.0f;
    }

    Correction =
        (Error * ANGLE_STRAIGHT_KP -
         GY87.YawRate_DPS * ANGLE_STRAIGHT_KD) * ANGLE_OUTPUT_SIGN;

    Correction = Gy87_Clamp(Correction,
                            -ANGLE_STRAIGHT_MAX_CORR,
                            ANGLE_STRAIGHT_MAX_CORR);

    Motor_SetSpeed(BaseSpeed + Correction,
                   BaseSpeed - Correction);
}

void Angle_StartTurn(float DeltaYaw)
{
    TurnTaskTarget = Angle_TargetAdd(GY87.Yaw, DeltaYaw);
    TurnTaskActive = 1;
    Angle_ResetController();
}

void Angle_StartTurnTo(float TargetYaw)
{
    TurnTaskTarget = Angle_Normalize(TargetYaw);
    TurnTaskActive = 1;
    Angle_ResetController();
}

uint8_t Angle_TurnTask(void)
{
    if(!TurnTaskActive)
    {
        return 1;
    }

    Angle_Hold(TurnTaskTarget);

    if(Angle_IsLocked())
    {
        TurnTaskActive = 0;
        return 1;
    }

    return 0;
}

void Angle_StopTurnTask(void)
{
    TurnTaskActive = 0;
    Angle_ResetController();
    Motor_SetSpeed(0.0f, 0.0f);
}

void GY87_Test(void)
{
    Serial_Printf("Yaw:%f GZ:%f Roll:%f Pitch:%f\r\n",
                  (double)GY87.Yaw,
                  (double)GY87.GyroZ_DPS,
                  (double)GY87.Roll,
                  (double)GY87.Pitch);

    Serial_Printf("RawGZ:%d ST:%04X ADR:%02X WHO:%02X I2C:%u EC:%08lX ERR:%lu\r\n",
                  GY87.GyroZ,
                  (unsigned int)GY87.Status,
                  (unsigned int)GY87.MpuAddr,
                  (unsigned int)GY87.MpuWhoAmI,
                  (unsigned int)GY87.LastI2CStatus,
                  (unsigned long)GY87.LastI2CErrorCode,
                  (unsigned long)GY87.I2CErrorCount);
}
