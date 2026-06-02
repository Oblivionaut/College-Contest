#ifndef __GY87_H
#define __GY87_H

#include "headfile.h"

/*
 * Temporary MPU6050-only yaw driver.
 *
 * It keeps the old GY87/Angle API so the rest of the car code can stay
 * unchanged while the GY87 module is replaced. Yaw is startup-relative:
 * the heading at power-on is 0 deg. Without a magnetometer there is no
 * true earth-absolute heading, only low-drift gyro integration.
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

#define GY87_I2C_TIMEOUT_MS             5U
#define GY87_UPDATE_PERIOD_S            0.005f
#define GY87_UPDATE_GUARD_MS            5U
#define GY87_UPDATE_MAX_DT_S            0.30f
#define GY87_USE_FIXED_DT               0
#define GY87_ACCEPT_UNKNOWN_MPU_WHO     1

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

/* Slow bias learning when the car is nearly still. */
#define GY87_GYRO_STATIC_LEARN_EN       1
#define GY87_GYRO_STATIC_DPS            0.55f
#define GY87_GYRO_BIAS_LEARN_ALPHA      0.00100f
#define GY87_STATIC_ACC_TOL_G           0.10f

/* ========================= */
/* Angle controller           */
/* ========================= */

#define ANGLE_KP                        0.055f
#define ANGLE_KD                        0.18f
#define ANGLE_LOCK_IN                   3.0f
#define ANGLE_LOCK_OUT                  7.0f
#define ANGLE_GYRO_LOCK                 8.0f
#define ANGLE_CROSS_LOCK_DEG            5.0f
#define ANGLE_MIN_TURN_SPEED            0.90f
#define ANGLE_MAX_TURN_SPEED            4.0f
#define ANGLE_NEAR_MAX_TURN_SPEED       2.2f
#define ANGLE_SLOW_DOWN_DEG             18.0f
#define ANGLE_DAMP_LIMIT                0.65f
#define ANGLE_SPEED_RAMP                0.20f
#define ANGLE_OUTPUT_SIGN               1.0f

#define ANGLE_STRAIGHT_KP               0.16f
#define ANGLE_STRAIGHT_KD               0.06f
#define ANGLE_STRAIGHT_MAX_CORR         8.0f
#define ANGLE_STRAIGHT_DEADBAND_DEG     0.4f

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
