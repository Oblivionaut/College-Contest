引脚：
tb8266：
PWMA：接PA6，配psc84-1，arr100-1，这样占空比可以是0-1000中选择
PWMB：接PA7，同上

IN脚，控制正反转
AIN1：PB0 (推挽输出，默认低电平，不上拉不下拉)
AIN2：PB1 同上

BIN1：PB4 (推挽输出，默认低电平，不上拉不下拉)
BIN2：PB5 同上


E脚编码器引脚，测速，判断方向

TIM4：
E1A： PD12 编码器模式，上拉推挽输出，最大速度 
E1B： PD13

TIM1：
E2A： PE9 编码器模式，上拉推挽输出，最大速度
E2B： PE11

k230
tx：PC7（USART6_RX）
rx：PC6(USART6_TX)



gy87

SDA:PB7(I2C1_SDA)
SCL:PB6(I2C2 SCL)

OLED
SCL：PB10
SDA：PB11

云台

控制左右电机：USART2
RX：PA2
TX：PA3

控制上下电机：USART3
RX：PD8
TX：PD9

测试USART4


PE2 按键

寻迹（八路）
0~8
PC8，PC9，PC10，PC11，PC12，PC13，PD2，PD3
i2c：E5,E6


声光：PD14， PD15