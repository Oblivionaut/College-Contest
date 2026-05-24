#ifndef __OLED_H
#define __OLED_H

#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "stdarg.h"

/**
 * @brief  0.96寸I2C OLED驱动（SSD1306控制器）
 * @note   硬件I2C版本，默认使用I2C2(PB10=SCL, PB11=SDA)
 * @note   显示坐标：行1-4(每行16像素)，列1-16(每列8像素)
 */

/**
 * @brief  OLED初始化
 * @note   必须在MX_I2C2_Init()之后调用
 */
void OLED_Init(void);

/**
 * @brief  全屏清屏（极速版，<10ms）
 */
void OLED_Clear(void);

/**
 * @brief  局部清屏（只清除指定区域，无闪烁）
 * @param  Line   起始行(1-4)
 * @param  Column 起始列(1-16)
 * @param  Len    要清除的字符个数
 */
void OLED_ClearArea(uint8_t Line, uint8_t Column, uint8_t Len);

/**
 * @brief  显示单个8x16字符
 * @param  Line   行号(1-4)
 * @param  Column 列号(1-16)
 * @param  Char   ASCII字符(0x20-0x7E)
 */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);

/**
 * @brief  显示字符串
 * @param  Line   起始行(1-4)
 * @param  Column 起始列(1-16)
 * @param  String 以'\0'结尾的字符串
 */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);

/**
 * @brief  显示无符号十进制数字
 * @param  Line   起始行(1-4)
 * @param  Column 起始列(1-16)
 * @param  Number 数字(0-4294967295)
 * @param  Length 显示长度(1-10)
 */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/**
 * @brief  显示有符号十进制数字
 * @param  Line   起始行(1-4)
 * @param  Column 起始列(1-16)
 * @param  Number 数字(-2147483648~2147483647)
 * @param  Length 显示长度(1-10，含符号位)
 */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);

/**
 * @brief  显示十六进制数字(大写)
 * @param  Line   起始行(1-4)
 * @param  Column 起始列(1-16)
 * @param  Number 数字(0-0xFFFFFFFF)
 * @param  Length 显示长度(1-8)
 */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/**
 * @brief  显示二进制数字
 * @param  Line   起始行(1-4)
 * @param  Column 起始列(1-16)
 * @param  Number 数字(0-0xFFFF)
 * @param  Length 显示长度(1-16)
 */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/**
 * @brief  格式化打印（兼容printf语法）
 * @param  Line   起始行(1-4)
 * @param  Column 起始列(1-16)
 * @param  fmt    格式化字符串
 * @param  ...    可变参数
 */
void oled_printf(uint8_t Line, uint8_t Column, const char *fmt, ...);

#endif
