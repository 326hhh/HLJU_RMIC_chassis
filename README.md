# HLJU_RMIC_chassis
黑大2026校内赛小车底盘控制代码

STM32F407VGTx + TB6612 + 麦轮底盘，编码器测速闭环 PID。
用户代码在 `UserLib/Chassis/`，接口说明见 [docs/CHASSIS_LIBRARY.md](docs/CHASSIS_LIBRARY.md)。

## 目录结构

```
INTRA_SCHOOL_COMPETITION_27RM
├── CMakeLists.txt
├── README.md
├── UserLib                                              # 用户库
│   └── Chassis                                          # 底盘库
├── docs                                                 # 文档
│   └── CHASSIS_LIBRARY.md                               # AI修改代码的文档  
├── hardware                                             # 硬件相关
│   └── 底盘扩展板原理图_2026-10-09.pdf                     # 底盘扩展板原理图
├── test_data                                            # VOFA+调参数据
│   └── 说明.md                                           # 说明
```

## 接线

### TB6612

A为前轮，B为后轮，L为左轮，R为右轮。TB6612左右分布。

#### PWM

| MCU | TB6612 |
| --- | --- |
| PA1(TIM5_CH2) | LF_PWMA |
| PA2(TIM5_CH3) | LB_PWMB |
| PA3(TIM5_CH4) | RF_PWMA |
| PE5(TIM9_CH1) | RB_PWMB |

#### AIN和STBY

| MCU | TB6612 |
| --- | --- |
| PA4 | L_STBY |
| PC4 | L_AIN1 |
| PC5 | L_AIN2|
| PB0 | L_BIN1|
| PB1 | L_BIN2 |
| PE7 | R_STBY |
| PE8 | R_AIN1 |
| PE10 | R_AIN2|
| PE12 | R_BIN1|
| PE13 | R_BIN2 |
    
### 电机线

| 颜色 | 引脚 |
| --- | --- |
| 红 | A |
| 黑 | B |
| 黄 | VCC |
| 绿 | GND |
| 白 | M+ |
| 蓝 | M- |

### 编码器

| MCU | 编码器 |
| --- | --- |
| PE9(TIM1_CH1) | RF_A |
| PE11(TIM1_CH2) | RF_B |
| PC6(TIM8_CH1) | RB_A |
| PC7(TIM8_CH2) | RB_B |                              
| PA6(TIM3_CH1) | LF_A |
| PA7(TIM3_CH2) | LF_B |
| PD12(TIM4_CH1) | LB_A |
| PD13(TIM4_CH2) | LB_B |

### USART

| USART | 引脚 |
| --- | --- |
| USART1_RX | PA10 |
| USART1_TX | PA9 |
| USART2_RX | PD6 |
| USART2_TX | PD5 |

波特率 115200，8位数据位，1位停止位，无校验位。

### IIC

| IIC | 引脚 |
| --- | --- |
| IIC1_SCL | PB6 |
| IIC1_SDA | PB7 |

### CAN

| CAN | 引脚 |
| --- | --- |
| CAN1_RX | PB8 |
| CAN1_TX | PB9 |

## 编译

```bash
cmake --preset Debug
cmake --build build/Debug --target INTRA_SCHOOL_COMPETITION_27RM
```

## 烧录

```bash
pyocd flash -t stm32f407vgtx build/Debug/INTRA_SCHOOL_COMPETITION_27RM.elf --connect under-reset
```


## 15. 日志

| 版本 | 日期 | 说明 |
|---|---|---|
| 0.1 | 2025-10-06 | 因为之前接线乱，导致烧了板子，然后打算底盘重新设计布线。现在就想出两种方案，一是PCB扩展板，二直接洞洞板飞线。PCB扩展板方案尽量早点做完，要不然只能洞洞板飞线了。重新在cubemx设计了引脚，现在就是更新一下引脚接线图。现在一个人干工作量好大 |
| 0.2 | 2025-10-09 | 完成了底盘扩展板的设计，新增加了CAN |





