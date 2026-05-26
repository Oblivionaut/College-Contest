#ifndef __HEADFILE_H
#define __HEADFILE_H

#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "oled.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "test.h"
#include "motor.h"
#include "tim.h"
#include "Serial.h"
#include "tracing.h"

#define PWM_MAX 100

/**************** 参数区（重点） ****************/

// 最大速度
#define TRACK_MAX_SPEED     20

// 最低速度
#define TRACK_MIN_SPEED     8

// 转向增益
#define TRACK_K             3

// 弯道减速增益
#define TRACK_SPEED_K       1


/************************************************/

#endif
