#include "headfile.h"

GY87_t GY87;



/* ========================================= */
/* Buffer                                    */
/* ========================================= */

static uint8_t MPU_Buffer[14];

static uint8_t MAG_Buffer[6];


/* ========================================= */
/* Parameter                                 */
/* ========================================= */

/* 固定更新周期 */
/* 必须每5ms调用一次GY87_GetYaw() */

#define GY87_DT                0.01f

/* 互补滤波系数 */
/* 越大响应越快 */

#define YAW_KP                 0.10f

/* 磁力计低通滤波 */

#define MAG_LPF_ALPHA          0.3f



/* ========================================= */
/* Magnet Calibration                        */
/* ========================================= */

/* 后期自行校准修改 */

#define MAG_OFFSET_X           0
#define MAG_OFFSET_Y           0



/* ========================================= */
/* Gyro Zero Drift                           */
/* ========================================= */

static float GyroZ_Offset = 0.0f;



/* ========================================= */
/* Write Register                            */
/* ========================================= */

void GY87_WriteReg(uint16_t DevAddr,
                   uint8_t Reg,
                   uint8_t Data)
{
    HAL_I2C_Mem_Write(&hi2c1,
                      DevAddr,
                      Reg,
                      I2C_MEMADD_SIZE_8BIT,
                      &Data,
                      1,
                      100);
}



/* ========================================= */
/* Read Register                             */
/* ========================================= */

void GY87_ReadRegs(uint16_t DevAddr,
                   uint8_t Reg,
                   uint8_t *Buffer,
                   uint8_t Len)
{
    HAL_I2C_Mem_Read(&hi2c1,
                     DevAddr,
                     Reg,
                     I2C_MEMADD_SIZE_8BIT,
                     Buffer,
                     Len,
                     100);
}



/* ========================================= */
/* MPU6050 Read                              */
/* ========================================= */

void MPU6050_Read(void)
{
    GY87_ReadRegs(MPU6050_ADDR,
                  0x3B,
                  MPU_Buffer,
                  14);

    GY87.AccX =
        (int16_t)((MPU_Buffer[0] << 8)
                 | MPU_Buffer[1]);

    GY87.AccY =
        (int16_t)((MPU_Buffer[2] << 8)
                 | MPU_Buffer[3]);

    GY87.AccZ =
        (int16_t)((MPU_Buffer[4] << 8)
                 | MPU_Buffer[5]);

    GY87.GyroX =
        (int16_t)((MPU_Buffer[8] << 8)
                 | MPU_Buffer[9]);

    GY87.GyroY =
        (int16_t)((MPU_Buffer[10] << 8)
                 | MPU_Buffer[11]);

    GY87.GyroZ =
        (int16_t)((MPU_Buffer[12] << 8)
                 | MPU_Buffer[13]);

    /* ±250dps */

    GY87.GyroZ_DPS =
        ((float)GY87.GyroZ / 131.0f)
        - GyroZ_Offset;
		
		
}



/* ========================================= */
/* HMC5883 Read                              */
/* ========================================= */

void HMC5883_Read(void)
{
    static float MagX_LPF = 0.0f;

    static float MagY_LPF = 0.0f;

    GY87_ReadRegs(HMC5883_ADDR,
                  0x03,
                  MAG_Buffer,
                  6);

    GY87.MagX =
        (int16_t)((MAG_Buffer[0] << 8)
                 | MAG_Buffer[1]);

    GY87.MagZ =
        (int16_t)((MAG_Buffer[2] << 8)
                 | MAG_Buffer[3]);

    GY87.MagY =
        (int16_t)((MAG_Buffer[4] << 8)
                 | MAG_Buffer[5]);

    /* 零偏校准 */

    GY87.MagX -= MAG_OFFSET_X;
    GY87.MagY -= MAG_OFFSET_Y;

    /* 低通滤波 */

    MagX_LPF =
        MagX_LPF * (1.0f - MAG_LPF_ALPHA)
        + GY87.MagX * MAG_LPF_ALPHA;

    MagY_LPF =
        MagY_LPF * (1.0f - MAG_LPF_ALPHA)
        + GY87.MagY * MAG_LPF_ALPHA;

    GY87.MagX = (int16_t)MagX_LPF;
    GY87.MagY = (int16_t)MagY_LPF;
	
	static uint16_t LostCnt = 0;

	if(GY87.MagX == 0 &&
	   GY87.MagY == 0)
	{
		LostCnt++;

		if(LostCnt >= 50)
		{
			LostCnt = 0;

			/* 重新开启BYPASS */

			GY87_WriteReg(MPU6050_ADDR,
						  0x37,
						  0x02);

			HAL_Delay(10);

			/* 重新初始化HMC5883 */

			GY87_WriteReg(HMC5883_ADDR,
						  0x00,
						  0x70);

			GY87_WriteReg(HMC5883_ADDR,
						  0x01,
						  0x20);

			GY87_WriteReg(HMC5883_ADDR,
						  0x02,
						  0x00);
		}
	}
	else
	{
		LostCnt = 0;
	}
}



/* ========================================= */
/* Magnet Yaw                                */
/* ========================================= */

float HMC5883_GetYaw(void)
{
    float yaw;

    yaw =
        atan2f((float)GY87.MagY,
               (float)GY87.MagX)
        * 57.29578f;

    if(yaw < 0.0f)
    {
        yaw += 360.0f;
    }

    GY87.MagYaw = yaw;

    return yaw;
}



/* ========================================= */
/* Gyro Calibration                          */
/* ========================================= */

static void GY87_GyroCalibrate(void)
{
    uint16_t i;

    float Sum = 0.0f;

    for(i = 0; i < 500; i++)
    {
        MPU6050_Read();

        Sum += (float)GY87.GyroZ / 131.0f;

        HAL_Delay(2);
    }

    GyroZ_Offset = Sum / 500.0f;
}



/* ========================================= */
/* Init                                      */
/* ========================================= */

void GY87_Init(void)
{
    /* MPU6050 Wake Up */

    GY87_WriteReg(MPU6050_ADDR,
                  0x6B,
                  0x00);

    HAL_Delay(10);

    /* Gyro ±250dps */

    GY87_WriteReg(MPU6050_ADDR,
                  0x1B,
                  0x00);

    /* Acc ±2g */

    GY87_WriteReg(MPU6050_ADDR,
                  0x1C,
                  0x00);

    /* DLPF 44Hz */

    GY87_WriteReg(MPU6050_ADDR,
                  0x1A,
                  0x03);

    /* 200Hz */

    GY87_WriteReg(MPU6050_ADDR,
                  0x19,
                  0x04);

    /* BYPASS */

    GY87_WriteReg(MPU6050_ADDR,
                  0x37,
                  0x02);

    HAL_Delay(100);

    /* ========================================= */
    /* HMC5883L                                  */
    /* ========================================= */

    /* 8 Sample Average */
    /* 15Hz */
    /* Normal Measurement */

    GY87_WriteReg(HMC5883_ADDR,
                  0x00,
                  0x70);

    /* Gain */

    GY87_WriteReg(HMC5883_ADDR,
                  0x01,
                  0x20);

    /* Continuous Mode */

    GY87_WriteReg(HMC5883_ADDR,
                  0x02,
                  0x00);

    HAL_Delay(100);

    /* Gyro Calibration */

    GY87_GyroCalibrate();
}



/* ========================================= */
/* Fusion Yaw                                */
/* ========================================= */

float GY87_GetYaw(void)
{
    static uint8_t FirstFlag = 1;

    static float Yaw = 0.0f;

    float MagYaw;

    float Error;

    MPU6050_Read();

    HMC5883_Read();

    MagYaw = HMC5883_GetYaw();

    /* 初始角 */

    if(FirstFlag)
    {
        Yaw = MagYaw;

        FirstFlag = 0;
    }

    /* ========================================= */
    /* Gyro Integration                          */
    /* ========================================= */

    /* 注意方向 */

    Yaw -= GY87.GyroZ_DPS * GY87_DT;

    /* ========================================= */
    /* Normalize                                 */
    /* ========================================= */

    if(Yaw >= 360.0f)
    {
        Yaw -= 360.0f;
    }

    if(Yaw < 0.0f)
    {
        Yaw += 360.0f;
    }

    /* ========================================= */
    /* Shortest Angle Error                      */
    /* ========================================= */

    Error = MagYaw - Yaw;

    if(Error > 180.0f)
    {
        Error -= 360.0f;
    }

    if(Error < -180.0f)
    {
        Error += 360.0f;
    }

    /* ========================================= */
    /* Complementary Filter                      */
    /* ========================================= */

    Yaw += Error * YAW_KP;

    /* ========================================= */
    /* Normalize Again                           */
    /* ========================================= */

    if(Yaw >= 360.0f)
    {
        Yaw -= 360.0f;
    }

    if(Yaw < 0.0f)
    {
        Yaw += 360.0f;
    }

    GY87.Yaw = Yaw;

    return Yaw;
}



/* ========================================= */
/* Test                                       */
/* ========================================= */

void GY87_Test(void)
{
    float yaw;

    yaw = GY87_GetYaw();

    OLED_ShowString(1,
                     1,
                     "Yaw:");

    /* 显示1位小数 */

    OLED_ShowNum(1,
                 5,
                 (int)(yaw * 10),
                 4);

    OLED_ShowString(2,
                     1,
                     "GZ:");

    OLED_ShowSignedNum(2,
                       4,
                       (int16_t)(GY87.GyroZ_DPS),
                       4);

    OLED_ShowString(3,
                     1,
                     "MX:");

    OLED_ShowSignedNum(3,
                       4,
                       GY87.MagX,
                       5);

    OLED_ShowString(4,
                     1,
                     "MY:");

    OLED_ShowSignedNum(4,
                       4,
                       GY87.MagY,
                       5);
}
