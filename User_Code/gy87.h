#ifndef __GY87_H
#define __GY87_H

#include "headfile.h"

/*
 * GY87 yaw driver.
 *
 * Yaw is estimated by a Kalman filter: MPU6050 gyro predicts fast motion, and
 * HMC5883L heading corrects long-term drift when a fresh magnetic sample exists.
 */

/* ========================= */
/* I2C and update timing      */
/* ========================= */

#define GY87_I2C_HANDLE                 hi2c1
#define GY87_I2C_SCL_PORT               GPIOB
#define GY87_I2C_SCL_PIN                GPIO_PIN_6
#define GY87_I2C_SDA_PORT               GPIOB
#define GY87_I2C_SDA_PIN                GPIO_PIN_7

#ifndef MPU6050_ADDR
#define MPU6050_ADDR                    (0x68 << 1)
#endif
#define MPU6050_ADDR_ALT                (0x69 << 1)

#ifndef HMC5883_ADDR
#define HMC5883_ADDR                    (0x1E << 1)
#endif

#define GY87_I2C_TIMEOUT_MS             5U
#define GY87_UPDATE_PERIOD_S            0.005f
#define GY87_UPDATE_GUARD_MS            5U
#define GY87_UPDATE_MAX_DT_S            0.30f
#define GY87_USE_FIXED_DT               0
#define GY87_ACCEPT_UNKNOWN_MPU_WHO     1
#define GY87_STARTUP_YAW_DEG            180.0f
#define GY87_USE_KALMAN_YAW             1

/* ========================= */
/* MPU6050 configuration      */
/* ========================= */

#define GY87_MPU_SMPLRT_DIV             0x04U   /* 1kHz/(4+1)=200Hz */
#define GY87_MPU_DLPF_CFG               0x03U   /* gyro 44Hz, accel 42Hz */
#define GY87_MPU_GYRO_CONFIG            0x10U   /* +/-1000dps */
#define GY87_MPU_ACCEL_CONFIG           0x00U   /* +/-2g */

#define GY87_ACCEL_SENS                 16384.0f
#define GY87_GYRO_SENS                  32.8f

/* Change this sign if yaw increases in the wrong direction. */
#define GY87_GYRO_Z_SIGN                1.0f
#define GY87_GYRO_DEADBAND_DPS          0.20f

/* Keep the car still during calibration. */
#define GY87_GYRO_CALIB_SAMPLES         500U
#define GY87_GYRO_CALIB_DELAY_MS        2U

/* Runtime bias learning is intentionally omitted; calibrate once at startup. */
#define GY87_GYRO_DEADBAND_ENABLE       0

/* ========================= */
/* HMC5883L magnetometer      */
/* ========================= */

#define GY87_MAG_UPDATE_PERIOD_MS       15U
#define GY87_MAG_STARTUP_SAMPLES        8U
#define GY87_MAG_STARTUP_DELAY_MS       20U
#define GY87_USE_MAG_STARTUP_YAW        1
#define GY87_MAG_FUSION_ENABLE          0
#define GY87_MAG_FUSION_GAIN            0.45f
#define GY87_MAG_FUSION_MAX_RATE_DPS    1.80f
#define GY87_MAG_FUSION_MAX_ERROR_DEG   45.0f
#define GY87_MAG_FUSION_ACC_TOL_G       0.20f

#define GY87_MAG_X_SIGN                 1.0f
#define GY87_MAG_Y_SIGN                 1.0f
#define GY87_MAG_Z_SIGN                 1.0f

#define GY87_MAG_X_OFFSET               0.0f
#define GY87_MAG_Y_OFFSET               0.0f
#define GY87_MAG_Z_OFFSET               0.0f

#define GY87_MAG_X_SCALE                1.0f
#define GY87_MAG_Y_SCALE                1.0f
#define GY87_MAG_Z_SCALE                1.0f

#define GY87_MAG_MIN_NORM               50.0f
#define GY87_MAG_MAX_NORM               5000.0f
#define GY87_MAG_YAW_SIGN               1.0f
#define GY87_MAG_YAW_OFFSET_DEG         0.0f
#define GY87_MAG_DECLINATION_DEG        0.0f

/* ========================= */
/* 角度环重点调参区           */
/* ========================= */
/*
 * Angle_TurnTask() 只有在角度误差进入 ANGLE_LOCK_IN，
 * 且角速度低于 ANGLE_GYRO_LOCK，并连续满足 ANGLE_LOCK_CONFIRM_TICKS
 * 个控制周期后，才会认为转向完成。
 */

#define ANGLE_KP                        0.045f  /* 角度误差比例增益，越大转得越积极 */
#define ANGLE_KD                        0.160f  /* 陀螺角速度阻尼，越大越抑制冲过头 */
#define ANGLE_LOCK_IN                   1.0f    /* 转向完成判定的角度误差范围，越小越准 */
#define ANGLE_LOCK_OUT                  5.0f    /* 已锁定后允许保持停车的误差范围 */
#define ANGLE_GYRO_LOCK                 2.5f    /* 完成判定时允许的最大角速度 */
#define ANGLE_LOCK_CONFIRM_TICKS        6U      /* 连续满足完成条件多少次才算真正完成 */
#define ANGLE_MIN_TURN_SPEED            0.35f   /* 远离目标时允许补的最低转向速度 */
#define ANGLE_MIN_TURN_ERROR_DEG        2.2f    /* 误差大于该值才启用最低转向速度 */
#define ANGLE_MAX_TURN_SPEED            1.60f   /* 角度环最大转向速度目标 */
#define ANGLE_SLOW_DOWN_DEG             180.0f  /* 误差小于该角度后开始按距离逐步降速 */
#define ANGLE_SPEED_RAMP                0.030f  /* 加速时每周期最大增量，越小起转越柔 */
#define ANGLE_SPEED_BRAKE_RAMP          0.35f   /* 减速或反向时每周期最大变化量 */
#define ANGLE_BRAKE_START_DEG           80.0f   /* 误差小于该角度后允许预测制动 */
#define ANGLE_BRAKE_GYRO_DPS            2.5f    /* 角速度大于该值才触发预测制动 */
#define ANGLE_BRAKE_PREDICT_S           0.80f   /* 预测制动使用的停车时间估计 */
#define ANGLE_BRAKE_MARGIN_DEG          4.0f    /* 预测制动额外保留角度 */
#define ANGLE_OUTPUT_SIGN               -1.0f   /* 角度环输出方向，整车反向时只改这里 */

#define ANGLE_STRAIGHT_KP               0.16f   /* 直行航向保持比例增益 */
#define ANGLE_STRAIGHT_KD               0.06f   /* 直行航向保持角速度阻尼 */
#define ANGLE_STRAIGHT_MAX_CORR         8.0f    /* 直行时左右轮最大差速修正 */
#define ANGLE_STRAIGHT_DEADBAND_DEG     5.0f    /* 直行航向误差小于该值时不修正 */

/* ========================= */
/* Status flags               */
/* ========================= */

#define GY87_STATUS_MPU_OK              0x0001U
#define GY87_STATUS_MAG_OK              0x0002U
#define GY87_STATUS_MAG_VALID           0x0004U
#define GY87_STATUS_FUSION_READY        0x0008U
#define GY87_STATUS_MAG_REJECTED        0x0010U

typedef struct
{
    int16_t AccX;
    int16_t AccY;
    int16_t AccZ;

    int16_t GyroX;
    int16_t GyroY;
    int16_t GyroZ;

    int16_t MagX;
    int16_t MagY;
    int16_t MagZ;

    float AccX_g;
    float AccY_g;
    float AccZ_g;
    float AccNorm_g;

    float GyroX_DPS;
    float GyroY_DPS;
    float GyroZ_Raw_DPS;
    float GyroZ_Offset_DPS;
    float GyroZ_DPS;
    float YawRate_DPS;

    float MagX_f;
    float MagY_f;
    float MagZ_f;
    float MagNorm;
    float MagYaw;

    float Pitch;
    float Roll;
    float Temperature_C;

    float Yaw;

    uint16_t Status;
    uint8_t MpuWhoAmI;
    uint8_t MpuAddr;
    uint8_t LastI2CStatus;
    uint32_t LastI2CErrorCode;
    uint32_t UpdateCount;
    uint32_t MagUpdateCount;
    uint32_t I2CErrorCount;
    uint32_t LastUpdateMs;
} GY87_t;

extern GY87_t GY87;

void GY87_WriteReg(uint16_t DevAddr, uint8_t Reg, uint8_t Data);
void GY87_ReadRegs(uint16_t DevAddr, uint8_t Reg, uint8_t *Buffer, uint8_t Len);

void GY87_Init(void);
uint8_t GY87_Update(void);

void MPU6050_Read(void);
void HMC5883_Read(void);
float HMC5883_GetYaw(void);

float GY87_GetYaw(void);
float GY87_GetYawFast(void);
void GY87_SetYaw(float Yaw);
uint8_t GY87_ResetYawToMag(void);
uint8_t GY87_IsReady(void);
uint8_t GY87_IsFresh(uint32_t MaxAgeMs);

float Angle_Normalize(float Angle);
float Angle_Error(float Target, float Current);
float Angle_TargetAdd(float StartYaw, float DeltaYaw);
float Angle_CalcHoldSpeed(float TargetYaw);
void Angle_ResetController(void);
uint8_t Angle_IsLocked(void);
void Angle_Hold(float TargetYaw);
void Angle_DriveStraight(float TargetYaw, float BaseSpeed);

void Angle_StartTurn(float DeltaYaw);
void Angle_StartTurnTo(float TargetYaw);
uint8_t Angle_TurnTask(void);
void Angle_StopTurnTask(void);

void GY87_Test(void);

#endif
