#ifndef __GY87_H
#define __GY87_H

#include "headfile.h"



/* ========================= */
/* I2C Address               */
/* ========================= */

#define MPU6050_ADDR      (0x68 << 1)

#define HMC5883_ADDR      (0x1E << 1)



/* ========================= */
/* GY87 Struct               */
/* ========================= */

typedef struct
{
    /* Accelerometer */

    int16_t AccX;
    int16_t AccY;
    int16_t AccZ;

    /* Gyroscope */

    int16_t GyroX;
    int16_t GyroY;
    int16_t GyroZ;

    /* Magnetometer */

    int16_t MagX;
    int16_t MagY;
    int16_t MagZ;

    /* Gyro Z Axis DPS */

    float GyroZ_DPS;

    /* Magnetometer Yaw */

    float MagYaw;

    /* Fusion Yaw */

    float Yaw;

}GY87_t;



extern GY87_t GY87;



/* ========================= */
/* Basic Driver              */
/* ========================= */

void GY87_WriteReg(uint16_t DevAddr,
                   uint8_t Reg,
                   uint8_t Data);

void GY87_ReadRegs(uint16_t DevAddr,
                   uint8_t Reg,
                   uint8_t *Buffer,
                   uint8_t Len);



/* ========================= */
/* Init                      */
/* ========================= */

void GY87_Init(void);



/* ========================= */
/* MPU6050                   */
/* ========================= */

void MPU6050_Read(void);



/* ========================= */
/* QMC5883                   */
/* ========================= */

void HMC5883_Read(void);

float HMC5883_GetYaw(void);



/* ========================= */
/* Fusion                    */
/* ========================= */

float GY87_GetYaw(void);



/* ========================= */
/* Debug                     */
/* ========================= */

void GY87_Test(void);



#endif

