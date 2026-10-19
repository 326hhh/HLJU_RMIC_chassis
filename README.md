# HLJU_RMIC_chassis
黑大2026校内赛小车底盘控制代码

STM32F407VGTx + TB6612 + 麦轮底盘，编码器测速闭环 PID + IMU（LSM6DSV16X）陀螺偏航闭环（小陀螺），
并通过 USART2 与 Jetson 上位机通信。
用户代码在 `UserLib/Chassis/`、`UserLib/Comm/` 与 `UserLib/IMU/`，接口说明见 [docs/CHASSIS_LIBRARY.md](docs/CHASSIS_LIBRARY.md)。

## 功能概览

- **底盘运动**：麦轮正/逆解、四轮独立速度环（速度前馈 + 参考加速度前馈 + PI、条件积分抗饱和）、
  轮速斜坡、编码器滑窗测速、里程计积分。
- **陀螺外环**：LSM6DSV16X 实测 wz 给运动学打补丁 —— 角速度闭环（RATE）、小陀螺定速自旋（SPIN）、
  底盘对准目标角（YAW_TARGET，预留给云台跟随）。
- **上位机链路**：USART2 二进制定长帧（12 B 命令 / 40 B 反馈，100 Hz 回传），CRC-8 校验、
  100 ms 看门狗、急停锁存。
- **调试/整定**：USART1 输出（RawText 文本 / VOFA+ JustFloat 波形）、`chassis_tune` 五种整定实验。
- **CAN**：CAN1 链路验证模块（心跳/回显/自环自检），当前未在 `main.c` 启用，见 [docs/CAN_TEST.md](docs/CAN_TEST.md)。

## 目录结构

```
INTRA_SCHOOL_COMPETITION_27RM/
├── Core/                              # CubeMX 生成：启动、外设初始化、中断
│   ├── Inc/
│   │   ├── main.h                     # 引脚宏定义（TB6612/LED）与 Error_Handler
│   │   ├── gpio.h                     # MX_GPIO_Init 原型
│   │   ├── tim.h                      # TIM1/3/4/5/8/9 句柄与 MX_TIMx_Init 原型
│   │   ├── usart.h                    # USART1/USART2 句柄与 MX_USARTx_UART_Init 原型
│   │   ├── i2c.h                      # I2C1 句柄与 MX_I2C1_Init 原型
│   │   ├── can.h                      # CAN1 句柄与 MX_CAN1_Init 原型
│   │   ├── stm32f4xx_it.h             # 异常/中断处理函数原型
│   │   └── stm32f4xx_hal_conf.h       # HAL 模块裁剪配置（启用了哪些外设、时钟值、断言）
│   └── Src/
│       ├── main.c                     # 应用入口：时钟、外设初始化、1 kHz 主循环
│       ├── gpio.c                     # MX_GPIO_Init：TB6612 方向/STBY 与 LED 引脚
│       ├── tim.c                      # 编码器 TIM1/3/4/8 + PWM TIM5/TIM9
│       ├── usart.c                    # USART1(调试) / USART2(上位机链路)
│       ├── i2c.c                      # I2C1 400 kHz（IMU）
│       ├── can.c                      # CAN1 1 Mbps（PB8/PB9）
│       ├── stm32f4xx_it.c             # 中断入口：SysTick / USART2 / CAN1_RX0
│       ├── stm32f4xx_hal_msp.c        # HAL 全局 MSP（HAL_MspInit）
│       ├── syscalls.c                 # newlib 系统调用桩（_write/_read/…）
│       ├── sysmem.c                   # _sbrk 堆实现（malloc 用）
│       └── system_stm32f4xx.c         # CMSIS SystemInit / SystemCoreClockUpdate
├── Drivers/                           # STM32F4 HAL + CMSIS（原样，不要手改）
├── UserLib/                           # 用户代码库
│   ├── Chassis/                       # 底盘：麦轮解算 / 测速 / PID / 里程计 / 陀螺外环 / 调试 / 整定
│   ├── Comm/                          # 通信：USART2 控制链路 + CAN 链路验证
│   └── IMU/                           # LSM6DSV16X 驱动与姿态处理（零偏/轴映射/yaw 积分）
├── docs/                              # 详细文档
│   ├── CHASSIS_LIBRARY.md             # 底盘库完整文档（原理/标定/整定/实测/协议）
│   └── CAN_TEST.md                    # CAN 链路验证与排障
├── hardware/                          # 底盘扩展板原理图 / 立创 EDA 工程
├── test_data/                         # 实测数据归档（历史 PID / 整定 / IMU 日志）
├── tools/                             # PC 端工具（数据分析 / 协议参考 / 交叉校验）
├── cmake/                             # 工具链文件与 CubeMX 源文件列表
├── build/                             # 构建输出（可删）
├── CMakeLists.txt                     # 顶层构建脚本（列出全部用户源文件）
├── CMakePresets.json                  # Debug / Release 预设
├── INTRA_SCHOOL_COMPETITION_27RM.ioc  # CubeMX 工程配置
├── STM32F407xx_FLASH.ld               # 链接脚本（1 MB Flash / 128 KB RAM）
├── startup_stm32f407xx.s              # 启动汇编（向量表 / Reset_Handler）
└── README.md
```

## Core 源文件说明

CubeMX 生成、在 `/* USER CODE */` 区外**不要手改**（重新生成会被覆盖）。

| 文件 | 行数 | 做了什么 / 实现了什么 |
|---|---|---|
| `Core/Src/main.c` | 230 | 应用入口。`SystemClock_Config()` 把 HSE 8 MHz 经 PLL 倍到 **168 MHz**（APB1 42 MHz、APB2 84 MHz）；按 CubeMX 顺序初始化 GPIO/TIM/USART/I2C/CAN；USER CODE 区依次调 `chassis_init()`、`uart_link_init()`，按开关选择 `chassis_tune_init()` 或演示模式；主循环 `chassis_poll()` + `uart_link_poll()` + 调试/整定轮询；`can_test` 调用当前注释掉。`Error_Handler()` 关中断死循环。 |
| `Core/Src/gpio.c` | 115 | `MX_GPIO_Init()`：使能 GPIOA~E 时钟，把 TB6612 的 STBY / AIN1 / AIN2 / BIN1 / BIN2（左板 PC4/PC5、PB0/PB1 + PA4；右板 PE7/PE8/PE10/PE12/PE13）和 LED（PB2）配成推挽输出，上电电平全部先拉低（电机不转）。PA0 配成输入。 |
| `Core/Src/tim.c` | 607 | **编码器**：TIM1/TIM3/TIM4/TIM8 配成编码器接口模式（TI12 四倍频，32 kHz 计数），对应右前/左前/左后/右后。**PWM**：TIM5（72 MHz/72=1 MHz 计数、周期 100 → 10 kHz）输出 CH2/CH3/CH4 = PA1/PA2/PA3；TIM9（168 MHz/168=1 MHz）输出 CH1 = PE5。还包含各外设的 MspInit/MspDeInit/MspPostInit（时钟与 GPIO 复用）。 |
| `Core/Src/usart.c` | 192 | USART1（PA9/PA10，115200 8N1）作调试输出；USART2（PD5/PD6，115200 8N1）作 Jetson 链路，并在 MspInit 里打开 **USART2 全局中断**（NVIC 优先级 6）——`uart_link.c` 靠它逐字节收帧。 |
| `Core/Src/i2c.c` | 117 | I2C1 配成 **Fast Mode 400 kHz**（7 位地址、无时钟延展），PB6=SCL / PB7=SDA（复用开漏）。IMU（LSM6DSV16X）挂在这条总线上。 |
| `Core/Src/can.c` | 123 | CAN1 配成 1 Mbps：预分频 3、BS1=11TQ、BS2=2TQ、SJW=1TQ（42 MHz/3/14 ≈ 1 Mbps，采样点 85.7%），普通模式、自动重传关闭；PB8=RX / PB9=TX，打开 CAN1_RX0 中断。位时序细节与坑见 [docs/CAN_TEST.md](docs/CAN_TEST.md)。 |
| `Core/Src/stm32f4xx_it.c` | 232 | 异常/中断向量实现：NMI/HardFault/MemManage/BusFault/UsageFault/SVC/DebugMon/PendSV 空实现；`SysTick_Handler()` 调 `HAL_IncTick()`（1 kHz 时基）；`CAN1_RX0_IRQHandler()` 调 `HAL_CAN_IRQHandler`；`USART2_IRQHandler()` 调 `HAL_UART_IRQHandler`（链路接收中断入口）。 |
| `Core/Src/stm32f4xx_hal_msp.c` | 81 | 只实现 `HAL_MspInit()`：使能 SYSCFG 与 PWR 时钟（各外设自己的 Msp 在各外设文件里）。 |
| `Core/Src/syscalls.c` | 244 | newlib 系统调用桩：`_read/_write/_close/_fstat/_isatty/_lseek/_open` 等。`_write` 可重定向到串口（调试 `printf` 的落点），未重定向时走半主机。 |
| `Core/Src/sysmem.c` | 87 | 实现 `_sbrk()`：在 `_end` 与栈顶之间分配堆空间，`malloc` 依赖它。 |
| `Core/Src/system_stm32f4xx.c` | 747 | CMSIS 系统文件：`SystemInit()`（复位后由启动文件调用，初始化 FPU/RCC 等）与 `SystemCoreClockUpdate()` 维护 `SystemCoreClock` 变量。 |
| `Core/Inc/main.h` | 91 | 引脚宏：`L_STBY/L_AIN1/L_AIN2/L_BIN1/L_BIN2`、`R_STBY/R_AIN1/R_AIN2/R_BIN1/R_BIN2`、`LED`（PB2）；声明 `Error_Handler()`。 |
| `Core/Inc/gpio.h` | 49 | `MX_GPIO_Init()` 原型。 |
| `Core/Inc/tim.h` | 69 | `htim1/3/4/5/8/9` 句柄 extern + `MX_TIMx_Init` / `HAL_TIM_MspPostInit` 原型。 |
| `Core/Inc/usart.h` | 55 | `huart1/huart2` 句柄 extern + `MX_USARTx_UART_Init` 原型。 |
| `Core/Inc/i2c.h` | 52 | `hi2c1` 句柄 extern + `MX_I2C1_Init` 原型。 |
| `Core/Inc/can.h` | 52 | `hcan1` 句柄 extern + `MX_CAN1_Init` 原型。 |
| `Core/Inc/stm32f4xx_it.h` | 68 | 上表的异常/中断处理函数原型。 |
| `Core/Inc/stm32f4xx_hal_conf.h` | 495 | HAL 裁剪配置：打开/关闭各 HAL 模块（本工程用到 GPIO/TIM/UART/I2C/CAN/PWR/RCC/CORTEX/FLASH/DMA/SYSCFG 等）、`HSE_VALUE`/`HSI_VALUE` 等时钟常量、`USE_FULL_ASSERT`、`TICK_INT_PRIORITY`。 |

## UserLib 模块说明

| 目录 | 一句话职责 | 详细文档 |
|---|---|---|
| `UserLib/Chassis/` | 麦轮底盘的完整控制链：运动学逆解 → 轮速斜坡 → 四轮 PI + 前馈 → TB6612；编码器滑窗测速 + 正解里程计；陀螺外环（小陀螺）；调试输出与 PID 整定实验。 | [UserLib/Chassis/README.md](UserLib/Chassis/README.md) |
| `UserLib/Comm/` | 通信层：`uart_link` 是 USART2 上位机链路（帧解析/命令执行/看门狗/反馈帧）；`can_test` 是 CAN1 链路验证（自环自检/心跳/回显）。 | [UserLib/Comm/README.md](UserLib/Comm/README.md) |
| `UserLib/IMU/` | LSM6DSV16X 驱动：I2C 事务（硬件/软件双后端）、地址探测、量程/速率/滤波配置、上电零偏标定与静止零偏跟踪、轴映射、yaw 积分、总线卡死恢复。 | [UserLib/IMU/README.md](UserLib/IMU/README.md) |

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
USART1 是调试/整定输出口；USART2 是 Jetson 控制链路（协议见 `UserLib/Comm/README.md`）。

### IIC（IMU）

| IIC | 引脚 |
| --- | --- |
| IIC1_SCL | PB6 |
| IIC1_SDA | PB7 |

插的是正点原子 ATK-MS6DSV（LSM6DSV16X）模块，平放（丝印 X 指车头、Y 指车左、Z 向上）。

扩展板 H5 出厂时 SDA / SCL 与模块是**交叉**的（2026-10-18 实测确认，已飞线改正）。
现在走**硬件 I2C1 400 kHz**（`IMU_USE_SOFT_I2C=0`），实测一次读 213 µs、采样 500 Hz。
代码里另有一套软件 I2C 后端（`IMU_USE_SOFT_I2C=1`，SCL=PB7 / SDA=PB6）作兜底，
换到同样交叉的板子时改一个宏就能跑（那套实测 351 µs / 500 Hz）。
模块 INT 不用（纯轮询 DATA_READY 不看中断）；SA0 由扩展板决定地址 → 代码对 0x6A / 0x6B 自动探测。

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

## 开发日志

| 版本 | 日期 | 说明 |
|---|---|---|
| 0.1 | 2025-10-06 | 地盘步线改用主控板和扩展板的方式，修改了一些引脚 |
| 0.2 | 2025-10-09 | 完成了底盘扩展板的设计，新增加了CAN |
| 0.3 | 2025-10-18 | 调完了地盘PID的速度环 |
| 0.4 | 2026-10-18 | 加入 IMU ，但没有在地上跑来调参 |
| 0.5 | 2026-10-19 | ①接入 **USART2 上位机控制链路 `uart_link`**（12 B 命令 / 40 B 反馈、CRC-8、100 Hz 回传、100 ms 看门狗、急停锁存），并补齐 USART1/USART2 的 NVIC 与中断；②**IMU 链路定型**：硬件 I2C1 400 kHz（213 µs/读、500 Hz）、SA0 焊死 GND（地址固定 0x6A）、20 分钟 soak 零错误，噪声/零偏/漂移全部实测归档（§17）；③修复「电机一卡一卡」——根因是 SA0 弱上拉导致 I2C 地址在 0x6A/0x6B 间漂移，另加地址热切换、9 时钟总线解锁、`healthy` 掉线回退与 `eps/ad/sw/rec` 诊断字段；④小陀螺自旋默认速率 3.5 → **2.0 rad/s**（落地实测抓地极限约 2.0~2.5，3.5 时占空比顶满也到不了） |
