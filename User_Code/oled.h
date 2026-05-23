#ifndef __OLED_H
#define __OLED_H

#include "stm32f4xx_hal.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// OLED尺寸定义
#define OLED_WIDTH  128
#define OLED_HEIGHT 64

// I2C地址(通常为0x78或0x7A，根据硬件决定)
#define OLED_ADDR   0x78

// 函数声明
void OLED_Init(void);
void OLED_Clear(void);
void OLED_SetPos(uint8_t x, uint8_t y);
void OLED_ShowChar(uint8_t row, uint8_t col, char ch);
void OLED_ShowString(uint8_t row, uint8_t col, char *str);

/**
 * @brief  OLED格式化打印函数
 * @param  row: 打印行号，从1开始(1-8)
 * @param  col: 打印列号，从1开始(1-16)
 * @param  fmt: 格式化字符串，与printf用法相同
 * @retval 无
 */
void oled_printf(uint8_t row, uint8_t col, const char *fmt, ...);

#endif
