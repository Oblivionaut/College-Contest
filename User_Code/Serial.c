#include "headfile.h"

/* 串口接收全局变量定义 */
uint8_t Serial_RxData;
uint8_t Serial_RxFlag;

#define SERIAL_TX_TIMEOUT_MS 2U

/**
 * @brief  串口发送一个字节
 * @param  Byte 要发送的字节（0~255）
 * @retval 无
 */
void Serial_SendByte(uint8_t Byte)
{
    HAL_UART_Transmit(&huart4, &Byte, 1, SERIAL_TX_TIMEOUT_MS);
}

/**
 * @brief  串口发送数组
 * @param  Array 数组首地址
 * @param  Length 数组长度
 * @retval 无
 */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++)
    {
        Serial_SendByte(Array[i]); // 逐字节发送
    }
}

/**
 * @brief  串口发送字符串
 * @param  String 字符串首地址（以'\0'结尾）
 * @retval 无
 */
void Serial_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        Serial_SendByte(String[i]); // 逐字符发送
    }
}

/**
 * @brief  内部次方函数（静态，仅本文件使用）
 * @param  X 底数
 * @param  Y 指数
 * @retval X^Y的结果
 */
static uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--)
    {
        Result *= X;
    }
    return Result;
}

/**
 * @brief  串口发送数字（十进制）
 * @param  Number 要发送的数字（0~4294967295）
 * @param  Length 数字的固定显示长度
 * @retval 无
 */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        /* 提取每一位数字并转为ASCII发送 */
        Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
 * @brief  重定向fputc以支持printf
 * @note   需要在编译器中开启「Use MicroLIB」
 * @param  ch 要输出的字符
 * @param  f 文件句柄（标准库默认）
 * @retval 输出的字符
 */
int fputc(int ch, FILE *f)
{
    Serial_SendByte((uint8_t)ch); // 绑定printf底层到串口发送
    return ch;
}

/**
 * @brief  自定义格式化打印函数（替代printf，更灵活）
 * @param  format 格式化字符串
 * @param  ... 可变参数列表
 * @retval 无
 */
void Serial_Printf(char *format, ...)
{
    char String[100]; // 缓存格式化后的字符串
    va_list arg;      // 可变参数列表
    va_start(arg, format);
    vsnprintf(String, sizeof(String), format, arg); // 格式化字符串到数组
    va_end(arg);
    Serial_SendString(String); // 发送格式化结果
}

/**
 * @brief  获取串口接收标志位
 * @note   读取后标志位自动清零
 * @retval 1：接收到数据；0：未接收到数据
 */
uint8_t Serial_GetRxFlag(void)
{
    if (Serial_RxFlag == 1)
    {
        Serial_RxFlag = 0;
        return 1;
    }
    return 0;
}

/**
 * @brief  获取串口接收的字节数据
 * @retval 接收到的字节（0~255）
 */
uint8_t Serial_GetRxData(void)
{
    return Serial_RxData;
}

/**
 * @brief  UART4中断服务函数
 * @note   需确保函数名与启动文件（startup_stm32f407xx.s）中的中断向量一致
 * @retval 无
 */
void UART4_IRQHandler(void)
{
    /* 检查UART4接收数据非空中断 */
    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_RXNE) != RESET)
    {
        /* 读取接收数据寄存器（HAL库需手动取DR寄存器值） */
        Serial_RxData = (uint8_t)(huart4.Instance->DR & 0xFF);
        /* 置位接收标志位 */
        Serial_RxFlag = 1;
        /* 清除接收非空标志位 */
        __HAL_UART_CLEAR_FLAG(&huart4, UART_FLAG_RXNE);
    }

    /* 调用HAL库通用中断处理（处理错误/其他中断） */
    HAL_UART_IRQHandler(&huart4);
}
