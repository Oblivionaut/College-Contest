#include "OLED.h"
#include "OLED_Font.h"
#include "i2c.h"  // 依赖CubeMX生成的I2C配置文件
#include "stdio.h"
#include "string.h"

/*************************** 硬件配置 ***************************/
extern I2C_HandleTypeDef hi2c2;  // 硬件I2C2句柄（CubeMX自动生成）
#define OLED_I2C_HANDLE  hi2c2
#define OLED_ADDR        0x78    // I2C地址，99%模块用这个，不行试0x7A

/*************************** 私有函数声明 ***************************/
static void OLED_WriteCommand(uint8_t cmd);
static void OLED_WriteData(uint8_t *data, uint16_t len);
static void OLED_SetCursor(uint8_t page, uint8_t col);
static uint32_t OLED_Pow(uint32_t x, uint32_t y);

/*************************** 私有函数实现 ***************************/

/**
 * @brief  向OLED写入命令
 */
static void OLED_WriteCommand(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    HAL_I2C_Master_Transmit(&OLED_I2C_HANDLE, OLED_ADDR, buf, 2, 100);
}

/**
 * @brief  批量向OLED写入数据（核心优化点）
 * @param  data 数据缓冲区指针
 * @param  len  数据长度(最大128字节)
 */
static void OLED_WriteData(uint8_t *data, uint16_t len)
{
    uint8_t buf[129];  // 1字节控制字 + 128字节数据
    buf[0] = 0x40;     // 连续写数据模式标识
    memcpy(&buf[1], data, len);
    HAL_I2C_Master_Transmit(&OLED_I2C_HANDLE, OLED_ADDR, buf, len+1, 100);
}

/**
 * @brief  设置光标位置
 * @param  page 页地址(0-7)
 * @param  col  列地址(0-127)
 */
static void OLED_SetCursor(uint8_t page, uint8_t col)
{
    OLED_WriteCommand(0xB0 | page);
    OLED_WriteCommand(0x10 | ((col & 0xF0) >> 4));
    OLED_WriteCommand(0x00 | (col & 0x0F));
}

/**
 * @brief  幂运算辅助函数（用于数字位分解）
 */
static uint32_t OLED_Pow(uint32_t x, uint32_t y)
{
    uint32_t res = 1;
    while (y--) res *= x;
    return res;
}

/*************************** 公开函数实现 ***************************/

/**
 * @brief  OLED初始化
 */
void OLED_Init(void)
{
    HAL_Delay(100);  // 上电稳定延时

    // SSD1306标准初始化序列
    OLED_WriteCommand(0xAE);  // 关闭显示
    OLED_WriteCommand(0xD5); OLED_WriteCommand(0x80);  // 时钟分频
    OLED_WriteCommand(0xA8); OLED_WriteCommand(0x3F);  // 多路复用率(1/64)
    OLED_WriteCommand(0xD3); OLED_WriteCommand(0x00);  // 显示偏移
    OLED_WriteCommand(0x40);  // 显示起始行
    OLED_WriteCommand(0x8D); OLED_WriteCommand(0x14);  // 开启电荷泵
    OLED_WriteCommand(0x20); OLED_WriteCommand(0x02);  // 页地址模式
    OLED_WriteCommand(0xA1);  // 段重映射(正常方向)
    OLED_WriteCommand(0xC8);  // COM扫描方向(正常方向)
    OLED_WriteCommand(0xDA); OLED_WriteCommand(0x12);  // COM引脚配置
    OLED_WriteCommand(0x81); OLED_WriteCommand(0xCF);  // 对比度
    OLED_WriteCommand(0xD9); OLED_WriteCommand(0xF1);  // 预充电周期
    OLED_WriteCommand(0xDB); OLED_WriteCommand(0x30);  // VCOMH电平
    OLED_WriteCommand(0xA4);  // 全局显示关闭
    OLED_WriteCommand(0xA6);  // 正常显示(非反显)
    OLED_WriteCommand(0xAF);  // 开启显示

    OLED_Clear();  // 初始化清屏
}

/**
 * @brief  极速全屏清屏（批量发送，8次I2C传输完成）
 */
void OLED_Clear(void)
{
    uint8_t page;
    const uint8_t clear_buf[128] = {0};  // 一整页空白数据

    for (page = 0; page < 8; page++)
    {
        OLED_SetCursor(page, 0);
        OLED_WriteData((uint8_t *)clear_buf, 128);
    }
}

/**
 * @brief  局部清屏（只清除指定字符区域）
 */
void OLED_ClearArea(uint8_t Line, uint8_t Column, uint8_t Len)
{
    uint8_t i;
    const uint8_t clear_buf[8] = {0};  // 单个字符的空白数据

    for (i = 0; i < Len; i++)
    {
        uint8_t page = (Line - 1) * 2;
        uint8_t col = (Column - 1 + i) * 8;

        // 清除上半页
        OLED_SetCursor(page, col);
        OLED_WriteData((uint8_t *)clear_buf, 8);

        // 清除下半页
        OLED_SetCursor(page + 1, col);
        OLED_WriteData((uint8_t *)clear_buf, 8);
    }
}

/**
 * @brief  显示单个字符
 */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t buf[8];
    uint8_t page = (Line - 1) * 2;
    uint8_t col = (Column - 1) * 8;
    uint8_t idx = Char - ' ';  // 字模索引偏移

    // 显示上半部分
    memcpy(buf, &OLED_F8x16[idx][0], 8);
    OLED_SetCursor(page, col);
    OLED_WriteData(buf, 8);

    // 显示下半部分
    memcpy(buf, &OLED_F8x16[idx][8], 8);
    OLED_SetCursor(page + 1, col);
    OLED_WriteData(buf, 8);
}

/**
 * @brief  显示字符串
 */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    while (*String && Column <= 16)
    {
        OLED_ShowChar(Line, Column++, *String++);
    }
}

/**
 * @brief  显示无符号十进制数字
 */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length && Column + i <= 16; i++)
    {
        uint8_t digit = Number / OLED_Pow(10, Length - i - 1) % 10;
        OLED_ShowChar(Line, Column + i, digit + '0');
    }
}

/**
 * @brief  显示有符号十进制数字
 */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint32_t abs_num;

    // 显示符号位
    if (Number >= 0)
    {
        OLED_ShowChar(Line, Column, '+');
        abs_num = Number;
    }
    else
    {
        OLED_ShowChar(Line, Column, '-');
        abs_num = -Number;
    }

    // 显示数字部分
    OLED_ShowNum(Line, Column + 1, abs_num, Length - 1);
}

/**
 * @brief  显示十六进制数字
 */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i, digit;
    for (i = 0; i < Length && Column + i <= 16; i++)
    {
        digit = Number / OLED_Pow(16, Length - i - 1) % 16;
        if (digit < 10)
            OLED_ShowChar(Line, Column + i, digit + '0');
        else
            OLED_ShowChar(Line, Column + i, digit - 10 + 'A');
    }
}

/**
 * @brief  显示二进制数字
 */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length && Column + i <= 16; i++)
    {
        uint8_t bit = Number / OLED_Pow(2, Length - i - 1) % 2;
        OLED_ShowChar(Line, Column + i, bit + '0');
    }
}

/**
 * @brief  格式化打印函数
 */
void oled_printf(uint8_t Line, uint8_t Column, const char *fmt, ...)
{
    char buf[32];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    OLED_ShowString(Line, Column, buf);
}
