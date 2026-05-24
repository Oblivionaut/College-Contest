#ifndef __SERIAL_H
#define __SERIAL_H

#include "headfile.h"

extern UART_HandleTypeDef huart4;

/* 串口接收全局变量声明 */
extern uint8_t Serial_RxData;  // 接收数据缓存
extern uint8_t Serial_RxFlag;  // 接收完成标志位

/* 函数原型（保持原接口兼容） */
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);

#endif
