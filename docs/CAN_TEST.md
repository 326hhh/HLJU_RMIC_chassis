# CAN 通信测试文档（CAN Test）

> 本文档是 `UserLib/Comm/can_test.*` 的**唯一权威说明**。
> 接线以项目根目录 `README.md` 为准，本文档只讲通信调试。

- 适用平台：STM32F407VGTx + CAN 收发器扩展板 ↔ Jetson Nano + KCAN-USB Pro
- CAN1 引脚：**PB8 = CAN1_RX**，**PB9 = CAN1_TX**（重映射）
- 位速率：**1 Mbps**（与 Jetson 侧 `can0`/`can1` 一致）
- 总线形态：标准帧（11 位）用于 Jetson ↔ F407；扩展帧（29 位）留给云台 ZDT 步进

---

## 1. 这个模块是干什么的

一个**最小可验证**的 CAN 收发固件，用来回答一个问题：

> **STM32 和 Jetson 到底能不能通信？**

它做三件事：

| # | 行为 | 意义 |
|---|---|---|
| ① | 每 500ms 发一帧心跳（ID `0x7F0`） | Jetson 一 `candump` 就知道 F407 活着 |
| ② | 收到任何帧就原样回一帧（ID+1） | **双向**通信的判定 |
| ③ | 统计 tx/rx/错误码/Bus-Off | 出问题时能定位到哪一层 |

验证通过后就把它换成正式协议实现，**CubeMX 那边不用再动**。

---

## 2. 目录结构

```
UserLib/Comm/
├── can_test.h          对外接口（状态快照 / init / poll / send）
├── can_test.c          实现
└── can_test_config.h   所有可调参数（位时序、心跳周期、ID 分配）
```

---

## 3. 为什么位时序写在代码里，而不是 CubeMX

本工程 `.ioc` 里 **只有 CubeMX 算出来的 `CAN1.Calculate*` 三个键**，没有
`Prescaler` / `BS1` / `BS2` / `SJW` 这几个真正生效的键。意思是从 CubeMX 重新
生成时，位时序会被默认值覆盖。

而默认值（实测生成的 `Core/Src/can.c`）是：

```c
hcan1.Init.Prescaler = 16;
hcan1.Init.TimeSeg1  = CAN_BS1_1TQ;   /* 1 TQ */
hcan1.Init.TimeSeg2  = CAN_BS2_1TQ;   /* 1 TQ */
```

一位只有 **3 个 TQ**，采样点 66.7% —— CAN 需要留出传播段和相位缓冲段来容忍
晶振偏差与线延迟，**这个配置在硬件上不可能稳定通信**。

所以 `can_test_init()` 会拿 `can_test_config.h` 里的值**再 `HAL_CAN_Init()` 一次**，
强制覆盖。因为 `MX_CAN1_Init()` 已经跑过（时钟和 GPIO 都配好了），
第二次 `HAL_CAN_Init()` 不会重复调 `MspInit`，只是覆盖位时序，是安全的。

**实际生效的速率可以运行时读回来**（从 `CAN_BTR` 寄存器反读，不信配置信寄存器）：

```c
can_test_state_t st = can_test_get_state();
printf("bitrate = %lu\r\n", (unsigned long)st.bitrate_hz);   /* 应为 1000000 */
```

---

## 4. 位时序速查

CAN1 挂 APB1。本工程 `PLLM=4 / PLLN=168 / PLLP=2` → HCLK 168MHz，APB1 = /4 = **42MHz**。

| 目标速率 | Prescaler | TQ/bit | BS1 | BS2 | 采样点 | 验算 |
|---|---|---|---|---|---|---|
| **1 Mbps** | **3** | 14 | **11TQ** | **2TQ** | 85.7% | 42M÷3÷14 = **1.000M** |
| 500 kbps | 6 | 14 | 11TQ | 2TQ | 85.7% | 42M÷6÷14 = 500k |
| 250 kbps | 12 | 14 | 11TQ | 2TQ | 85.7% | 42M÷12÷14 = 250k |

> 采样点 = `(1 + BS1) / (1 + BS1 + BS2)`，CAN 推荐 75%~87.5%。
> **改主频就必须重算**，`CANTEST_APB1_HZ` 也要跟着改。

---

## 5. 使用方式

接进 `main.c` 的 USER CODE 区：

```c
/* USER CODE BEGIN Includes */
#include "can_test.h"
/* USER CODE END Includes */
```

```c
  MX_CAN1_Init();          /* CubeMX 生成的，必须在 can_test_init() 之前 */

  /* USER CODE BEGIN 2 */
  can_test_init();         /* 配位时序 + 滤波器 + 启动 CAN */
  /* USER CODE END 2 */
```

```c
  while (1)
  {
    /* USER CODE BEGIN 3 */
    can_test_poll();       /* 发心跳 + 收帧回显 */
  }
```

**顺序是硬要求**：`can_test_init()` 必须在 `MX_CAN1_Init()` 之后，
否则 `hcan1` 还没初始化，`HAL_CAN_Start()` 会失败并卡在 `Error_Handler()`。

---

## 6. Jetson 侧验证

```bash
# ① 看心跳 —— 应该每 0.5 秒一行
candump can0
#   期望:  can0  7F0   [4]  00 CA FE 00
#          can0  7F0   [4]  01 CA FE 00
```

```bash
# ② 双向回显
candump can0 &
cansend can0 123#A1B2C3D4
#   期望:  can0  124   [4]  A1 B2 C3 D4
```

**①② 都通过 = 通信完全正常。**

心跳数据里的 `CA FE` 是识别用的魔数，第 4 字节是 F407 已收帧数的低 8 位，
所以光看心跳就能知道**对面收没收到东西**。

---

## 7. F407 上三个经典坑

### 7.1 滤波器不配 = 一帧都收不到

HAL **不会**自动给默认滤波器。CubeMX 生成的 `can.c` 里只有 `HAL_CAN_Init()`，
没有 `HAL_CAN_ConfigFilter()`，FIFO 会把所有帧拒掉。

本模块用「掩码全 0」的全通过滤波器：

```c
f.FilterMode       = CAN_FILTERMODE_IDMASK;
f.FilterScale      = CAN_FILTERSCALE_32BIT;
f.FilterIdHigh     = 0x0000;
f.FilterMaskIdHigh = 0x0000;   /* 掩码全 0 = 不关心任何位 = 全通过 */
```

### 7.2 CAN1 / CAN2 共用滤波器组

F407 共 14 组滤波器（0~13 给 CAN1，14~27 给 CAN2），
且**用 CAN2 之前必须先初始化 CAN1**。本模块只用 CAN1，占第 0 组，
`SlaveStartFilterBank = 14`。

### 7.3 初始化顺序

必须是 **`HAL_CAN_ConfigFilter()` → `HAL_CAN_Start()` → `HAL_CAN_ActivateNotification()`**。
反过来 `ConfigFilter` 会返回 `HAL_ERROR`。

---

## 8. 中断还是轮询

本工程 CubeMX 里**没开 CAN1 中断**，所以默认走**主循环轮询**
（`CANTEST_USE_RX_INTERRUPT = 0`），把 FIFO0 取空。裸机主循环足够快。

要用中断：

1. CubeMX → `System Core` → `NVIC` → 勾 **CAN1 RX0 interrupts** → Enable
2. `can_test_config.h` 里把 `CANTEST_USE_RX_INTERRUPT` 改成 `1`

> ⚠️ **两者不要同时开**，否则同一帧会被处理两次。
> 另外 `HAL_CAN_RxFifo0MsgPendingCallback` 是 HAL 弱函数，工程里别处也定义会链接冲突，合并即可。

---

## 9. 排查表

| 现象 | 大概率原因 | 查什么 |
|---|---|---|
| **完全没有帧** | CAN 没启动 / 引脚错 | `can_test_get_state().started` 是否为 true；PB8/PB9 上有没有波形 |
| **有心跳，但发帧没回显** | 收不到帧 | 滤波器配了吗；若用中断，NVIC 勾了吗 |
| **卡在 `Error_Handler()`** | `can_test_init()` 在 `MX_CAN1_Init()` 之前 | 看 main.c 调用顺序 |
| **`bus_off_seen == true`** | 总线一直出错 | ①对面没上电/线断（无 ACK）②终端电阻缺失 ③两边位速率不同 |
| **心跳正常但 Jetson 收不到** | 两边速率不一致 | F407 侧看 `bitrate_hz`，Jetson 侧 `ip -details link show can0` |
| **PB8/PB9 上有波形但对面无反应** | 收发器 TXD/RXD 接反 | PB9→TXD、PB8→RXD，**别接反** |

### 串口打印诊断

```c
static uint32_t last_print = 0;
if (HAL_GetTick() - last_print >= 1000) {
    can_test_state_t st = can_test_get_state();
    last_print = HAL_GetTick();
    printf("CAN %lu bps  tx=%lu rx=%lu err=0x%08lX bof=%d\r\n",
           (unsigned long)st.bitrate_hz,
           (unsigned long)st.tx_count,
           (unsigned long)st.rx_count,
           (unsigned long)st.last_error,
           (int)st.bus_off_seen);
}
```

`tx` 在涨 → 发送侧正常。
`rx` 一直 0 → **收不到，查滤波器和中断**。
`bof=1` → **物理层问题**。

---

## 10. 物理层检查（软件查不出来时）

KCAN 适配器**不上报 ACK 错误**（`rec/tec` 恒为 0/0），所以软件侧无法判断
"对方在不在总线上"。物理层必须用万用表确认：

**断电测电阻**

| 测点 | 应该 |
|---|---|
| CAN_H ↔ CAN_L | **~60Ω**（两端各 120Ω 并联） |
| CAN_H ↔ GND | >1MΩ |
| CAN_L ↔ GND | >1MΩ |

**通电测电压**（黑表笔接 GND）

| 测点 | 空闲时 |
|---|---|
| 收发器 VCC | 5V 或 3.3V |
| CAN_H / CAN_L | 各 **~2.5V**（隐性电平） |
| MCU PB9 (CAN1_TX) | 3.3V 高 |
| MCU PB8 (CAN1_RX) | 3.3V 高 |

> CAN 是差分总线，**物理层不通软件怎么调都是白费**。

---

## 11. 测试通过之后

把 `can_test.c` 换成正式协议实现即可，`can_test_config.h` 的位时序部分可以
挪进正式的通信模块。规划的 ID 分配：

| ID | 方向 | 内容 |
|---|---|---|
| `0x000` | 广播 | 急停 |
| `0x010 \| src` | 各节点 | 心跳 |
| `0x020 \| dst` | Jetson → F407 | 底盘速度 `vx(i16 mm/s) \| vy(i16) \| wz(i16) \| flags \| seq` |
| `0x110 \| src` | F407 → Jetson | 里程计 / IMU 反馈 |

⚠️ 云台的 ZDT 步进走 **29-bit 扩展帧**（`(Addr<<8)|Packet`），
和上面的 **11-bit 标准帧**天然隔离，**同一条总线混跑不会冲突**。

另外一个提醒：正式接上去之前，`chassis_demo` 那套演示代码建议先关掉，
免得调 CAN 的时候轮子乱转。
