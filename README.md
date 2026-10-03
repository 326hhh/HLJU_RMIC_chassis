# HLJU_RMIC_chassis
黑大2026校内赛小车底盘控制代码

STM32F407VGTx + TB6612 + 麦轮底盘，编码器测速闭环 PID。
用户代码在 `UserLib/Chassis/`，接口说明见 [docs/CHASSIS_LIBRARY.md](docs/CHASSIS_LIBRARY.md)。

## 接线

### TB6612

| MCU | TB6612 |
| --- | --- |
| PE14 | L_PWMB |
| PE9 | L_PWMA |
| PE5 | R_PWMA |
| PE6 | R_PWMB |

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
| PA0(TIM2_CH1) | FR_A |
| PA1(TIM2_CH2) | FR_B |
| PA6(TIM3_CH1) | FL_A |
| PA7(TIM3_CH2) | FL_B |
| PD12(TIM4_CH1) | BL_A |
| PD13(TIM4_CH2) | BL_B |
| PC6(TIM8_CH1) | BR_A |
| PC7(TIM8_CH2) | BR_B |

## 编译

```bash
cmake --preset Debug
cmake --build build/Debug --target INTRA_SCHOOL_COMPETITION_27RM
```

## 烧录

```bash
pyocd flash -t stm32f407vgtx build/Debug/INTRA_SCHOOL_COMPETITION_27RM.elf --connect under-reset
```
