#include "OLED.h"
#include "OLED_Font.h"
#include "i2c.h"  // 依赖CubeMX生成的I2C配置文件
#include "stdio.h"
#include "string.h"

/*************************** 硬件配置 ***************************/
extern I2C_HandleTypeDef hi2c2;  // 硬件I2C2句柄（CubeMX自动生成）
#define OLED_I2C_HANDLE  hi2c2
#define OLED_ADDR        0x78    // I2C地址，99%模块用这个，不行试0x7A
#define OLED_I2C_CMD_TIMEOUT_MS  5U
#define OLED_I2C_DATA_TIMEOUT_MS 25U
#define OLED_INIT_DELAY_MS  150U
#define OLED_INIT_RETRY     3U
#define OLED_RETRY_PERIOD_MS 1000U
#define OLED_SCL_PORT       GPIOB
#define OLED_SCL_PIN        GPIO_PIN_10
#define OLED_SDA_PORT       GPIOB
#define OLED_SDA_PIN        GPIO_PIN_11

/*************************** 私有函数声明 ***************************/
static uint8_t OLED_WriteCommand(uint8_t cmd);
static uint8_t OLED_WriteData(uint8_t *data, uint16_t len);
static uint8_t OLED_InitSequence(void);
static void OLED_TryWake(void);
static void OLED_SetCursor(uint8_t page, uint8_t col);
static uint32_t OLED_Pow(uint32_t x, uint32_t y);

static uint8_t OLED_Ready = 0;
static uint32_t OLED_ErrorCount = 0;
static uint32_t OLED_NextRetryMs = 0;

/*************************** 私有函数实现 ***************************/

static void OLED_I2C_RecoverBus(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint8_t i;

    HAL_I2C_DeInit(&OLED_I2C_HANDLE);

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = OLED_SCL_PIN | OLED_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_SET);
    HAL_Delay(1);

    for(i = 0; i < 9U; i++)
    {
        HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_SET);
    HAL_Delay(1);

    MX_I2C2_Init();
}

static void OLED_CheckDevice(void)
{
    if(HAL_I2C_IsDeviceReady(&OLED_I2C_HANDLE,
                             OLED_ADDR,
                             1U,
                             OLED_I2C_CMD_TIMEOUT_MS) == HAL_OK)
    {
        OLED_Ready = 1;
    }
}

static uint8_t OLED_Transmit(uint8_t *buf, uint16_t len)
{
    uint32_t TimeoutMs;

    if(!OLED_Ready)
    {
        return 0;
    }

    TimeoutMs =
        (len > 16U) ? OLED_I2C_DATA_TIMEOUT_MS : OLED_I2C_CMD_TIMEOUT_MS;

    if(HAL_I2C_Master_Transmit(&OLED_I2C_HANDLE,
                               OLED_ADDR,
                               buf,
                               len,
                               TimeoutMs) == HAL_OK)
    {
        return 1;
    }

    OLED_ErrorCount++;
    OLED_Ready = 0;
    OLED_NextRetryMs = HAL_GetTick() + OLED_RETRY_PERIOD_MS;
    OLED_I2C_RecoverBus();
    OLED_CheckDevice();
    return 0;
}

/**
 * @brief  向OLED写入命令
 */
static uint8_t OLED_WriteCommand(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    return OLED_Transmit(buf, 2U);
}

/**
 * @brief  批量向OLED写入数据（核心优化点）
 * @param  data 数据缓冲区指针
 * @param  len  数据长度(最大128字节)
 */
static uint8_t OLED_WriteData(uint8_t *data, uint16_t len)
{
    uint8_t buf[129];  // 1字节控制字 + 128字节数据

    if(len > 128U)
    {
        len = 128U;
    }

    buf[0] = 0x40;     // 连续写数据模式标识
    memcpy(&buf[1], data, len);
    return OLED_Transmit(buf, len + 1U);
}

static uint8_t OLED_InitSequence(void)
{
    // SSD1306标准初始化序列
    if(!OLED_WriteCommand(0xAE)) return 0;  // 关闭显示
    if(!OLED_WriteCommand(0xD5)) return 0;
    if(!OLED_WriteCommand(0x80)) return 0;  // 时钟分频
    if(!OLED_WriteCommand(0xA8)) return 0;
    if(!OLED_WriteCommand(0x3F)) return 0;  // 多路复用率(1/64)
    if(!OLED_WriteCommand(0xD3)) return 0;
    if(!OLED_WriteCommand(0x00)) return 0;  // 显示偏移
    if(!OLED_WriteCommand(0x40)) return 0;  // 显示起始行
    if(!OLED_WriteCommand(0x8D)) return 0;
    if(!OLED_WriteCommand(0x14)) return 0;  // 开启电荷泵
    if(!OLED_WriteCommand(0x20)) return 0;
    if(!OLED_WriteCommand(0x02)) return 0;  // 页地址模式
    if(!OLED_WriteCommand(0xA1)) return 0;  // 段重映射(正常方向)
    if(!OLED_WriteCommand(0xC8)) return 0;  // COM扫描方向(正常方向)
    if(!OLED_WriteCommand(0xDA)) return 0;
    if(!OLED_WriteCommand(0x12)) return 0;  // COM引脚配置
    if(!OLED_WriteCommand(0x81)) return 0;
    if(!OLED_WriteCommand(0xCF)) return 0;  // 对比度
    if(!OLED_WriteCommand(0xD9)) return 0;
    if(!OLED_WriteCommand(0xF1)) return 0;  // 预充电周期
    if(!OLED_WriteCommand(0xDB)) return 0;
    if(!OLED_WriteCommand(0x30)) return 0;  // VCOMH电平
    if(!OLED_WriteCommand(0xA4)) return 0;  // 全局显示关闭
    if(!OLED_WriteCommand(0xA6)) return 0;  // 正常显示(非反显)
    return OLED_WriteCommand(0xAF);         // 开启显示
}

static void OLED_TryWake(void)
{
    uint32_t NowMs;

    if(OLED_Ready)
    {
        return;
    }

    NowMs = HAL_GetTick();

    if((int32_t)(NowMs - OLED_NextRetryMs) < 0)
    {
        return;
    }

    OLED_NextRetryMs = NowMs + OLED_RETRY_PERIOD_MS;
    OLED_I2C_RecoverBus();
    OLED_CheckDevice();

    if(OLED_Ready && !OLED_InitSequence())
    {
        OLED_Ready = 0;
    }
}

/**
 * @brief  设置光标位置
 * @param  page 页地址(0-7)
 * @param  col  列地址(0-127)
 */
static void OLED_SetCursor(uint8_t page, uint8_t col)
{
    if(!OLED_WriteCommand(0xB0 | page))
    {
        return;
    }

    if(!OLED_WriteCommand(0x10 | ((col & 0xF0) >> 4)))
    {
        return;
    }

    (void)OLED_WriteCommand(0x00 | (col & 0x0F));
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
    uint8_t i;

    OLED_Ready = 0;
    OLED_NextRetryMs = 0;
    HAL_Delay(OLED_INIT_DELAY_MS);  // 上电稳定延时

    for(i = 0; i < OLED_INIT_RETRY; i++)
    {
        OLED_I2C_RecoverBus();
        OLED_CheckDevice();

        if(OLED_Ready)
        {
            break;
        }

        OLED_ErrorCount++;
        HAL_Delay(20);
    }

    if(!OLED_Ready)
    {
        OLED_NextRetryMs = HAL_GetTick() + OLED_RETRY_PERIOD_MS;
        return;
    }

    if(!OLED_InitSequence())
    {
        OLED_Ready = 0;
        OLED_NextRetryMs = HAL_GetTick() + OLED_RETRY_PERIOD_MS;
        return;
    }

    OLED_Clear();  // 初始化清屏
}

/**
 * @brief  极速全屏清屏（批量发送，8次I2C传输完成）
 */
void OLED_Clear(void)
{
    uint8_t page;
    const uint8_t clear_buf[128] = {0};  // 一整页空白数据

    if(!OLED_Ready)
    {
        OLED_TryWake();

        if(!OLED_Ready)
        {
            return;
        }
    }

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

    if(!OLED_Ready)
    {
        OLED_TryWake();

        if(!OLED_Ready)
        {
            return;
        }
    }

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

    if(!OLED_Ready)
    {
        OLED_TryWake();

        if(!OLED_Ready)
        {
            return;
        }
    }

    if(Char < ' ' || Char > '~')
    {
        Char = ' ';
        idx = 0;
    }

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
    uint8_t page;
    uint8_t col;
    uint8_t count = 0;
    uint8_t upper_buf[128];
    uint8_t lower_buf[128];

    if(!OLED_Ready)
    {
        OLED_TryWake();

        if(!OLED_Ready)
        {
            return;
        }
    }

    if(Line < 1U || Line > 4U || Column < 1U || Column > 16U)
    {
        return;
    }

    while(String[count] != '\0' && (Column + count) <= 16U)
    {
        char Char = String[count];
        uint8_t idx;

        if(Char < ' ' || Char > '~')
        {
            Char = ' ';
        }

        idx = (uint8_t)(Char - ' ');
        memcpy(&upper_buf[count * 8U], &OLED_F8x16[idx][0], 8U);
        memcpy(&lower_buf[count * 8U], &OLED_F8x16[idx][8], 8U);
        count++;
    }

    if(count == 0U)
    {
        return;
    }

    page = (uint8_t)((Line - 1U) * 2U);
    col = (uint8_t)((Column - 1U) * 8U);

    OLED_SetCursor(page, col);
    OLED_WriteData(upper_buf, (uint16_t)count * 8U);
    OLED_SetCursor((uint8_t)(page + 1U), col);
    OLED_WriteData(lower_buf, (uint16_t)count * 8U);
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
