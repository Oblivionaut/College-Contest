引脚：
tb8266：
PWMA：接PA6，配psc84-1，arr100-1，这样占空比可以是0-1000中选择
PWMB：接PA7，同上

IN脚，控制正反转
AIN1：PB0 (推挽输出，默认低电平，不上拉不下拉)
AIN2：PB1 同上

BIN1：PB2 (推挽输出，默认低电平，不上拉不下拉)
BIN2：PB3 同上


E脚编码器引脚，测速，判断方向

E1A： PD12 编码器模式，上拉推挽输出，最大速度
E1B： PD13

E2A： PE9 编码器模式，上拉推挽输出，最大速度
E2B： PE11

k230
tx：PC7（USART1_RX）
rx：PC6(USART1_TX)



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

