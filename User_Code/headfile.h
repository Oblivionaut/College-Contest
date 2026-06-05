#ifndef __HEADFILE_H
#define __HEADFILE_H

#include "stm32f4xx_hal.h"
#include "i2c.h"

#include <math.h>
#include "oled.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "Key.h"
#include "test.h"
#include "motor.h"
#include "tim.h"
#include "Serial.h"
#include "tracing.h"
#include "gy87.h"

#ifndef MPU6050_ADDR
#define MPU6050_ADDR   (0x68 << 1)
#endif

#ifndef HMC5883_ADDR
#define HMC5883_ADDR   (0x1E << 1)
#endif



#endif
