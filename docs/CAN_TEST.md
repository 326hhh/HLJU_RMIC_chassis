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

> ⚠️ **当前 `main.c` 的集成方式已改为编译开关**（不再注释代码）：
> `can_test_config.h` 里 `#define CANTEST_ENABLE 1` 时启用，`0` 时关闭；
> `main.c` 用 `#if CANTEST_ENABLE` 包住 `can_test_init()` / `can_test_poll()`。
> 该开关与 `can_link_config.h` 的 `CAN_LINK_ENABLE` **编译期互斥**（两者都要初始化 CAN1，
> 同时启用会被 `can_link.c` 顶部的 `#error` 拦下）。
> 注意：正式链路 `can_link` 已实现，本模块只作**链路验证/排障**用，见
> [docs/CAN_CONTROL_PROTOCOL.md](CAN_CONTROL_PROTOCOL.md)。

接进 `main.c` 的 USER CODE 区：

```c
/* USER CODE BEGIN Includes */
#include "can_test.h"
/* USER CODE END Includes */
```

```c
  MX_CAN1_Init();          /* CubeMX 生成的，必须在 can_test_init() 之前 */

  /* USER CODE BEGIN 2 */
#if CANTEST_ENABLE
  can_test_init();         /* 配位时序 + 滤波器 + 启动 CAN */
#endif
  /* USER CODE END 2 */
```

```c
  while (1)
  {
    /* USER CODE BEGIN 3 */
#if CANTEST_ENABLE
    can_test_poll();       /* 发心跳 + 收帧回显 */
#endif
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

> ✅ **已完成**：正式协议实现是 `UserLib/Comm/can_link`（`0x000`/`0x020`/`0x021` +
> `0x110~0x114` 五帧反馈），协议字段、CRC、看门狗与串口版 `uart_link` **逐字段一致**。
> 帧表 / 组完整性规则 / 联调记录见 [`docs/CAN_CONTROL_PROTOCOL.md`](CAN_CONTROL_PROTOCOL.md)，
> 模块用法见 [`../UserLib/Comm/README.md`](../UserLib/Comm/README.md)。
>
> 本模块（`can_test`）保留作**链路验证与排障**工具：`CANTEST_ENABLE=1` 时启用，
> 与 `CAN_LINK_ENABLE=1` 互斥（编译期 `#error` 拦下）。

规划的 ID 分配（已按此实现）：

| ID | 方向 | 内容 |
|---|---|---|
| `0x000` | 广播 | 急停（DLC 任意，Jetson 发、F407 只收） |
| `0x020` | Jetson → F407 | 底盘速度 `cmd \| seq \| vx(i16 mm/s) \| vy(i16) \| wz(i16) 100 Hz` |
| `0x021` | Jetson → F407 | flags 保活帧（10 Hz + 变化即发） |
| `0x110~0x114` | F407 → Jetson | 40 B 反馈按 8 字节切 5 帧 |

⚠️ 云台的 ZDT 步进走 **29-bit 扩展帧**（`(Addr<<8)|Packet`），
和上面的 **11-bit 标准帧**天然隔离，**同一条总线混跑不会冲突**。

另外一个提醒：正式接上去之前，`chassis_demo` 那套演示代码建议先关掉，
免得调 CAN 的时候轮子乱转。

---

## 12. 联调排障记录：2026-11-01 全静默（CAN_H/CAN_L 反接）

本节记录一次「Jetson 侧 `candump` 一帧都收不到」的完整定位过程，留作以后同类问题的模板。

### 12.1 现场

| 项 | 实际情况 |
|---|---|
| 底盘 | STM32F407 + 扩展板（TJA1050，5V），已上电，5V 正常 |
| 上位机 | Jetson Nano，USB-CAN 是 **CANable2**（`16d0:117e`，SLCAN 固件）→ `/dev/ttyACM0` |
| 接线 | 适配器 `CAN_H/CAN_L` → 扩展板 `H6` 端子（2-pin，1=CANH / 2=CANL），万用表通断正常 |
| 固件 | `CAN_LINK_ENABLE = 1`（正式链路，100 Hz 无条件发 `0x110~0x114`） |

> ⚠️ 和本文档开头「KCAN-USB Pro」的平台描述不同：这次接的是 CANable2。
> Jetson 上那个 `kcan-usbpro.service` 是给**鲲弘 KCAN-USB Pro（`0483:000d`）**用的，
> `kcan.ko` 的 USB ID 表里没有 CANable2，**开机自启也创建不出 `can0`**，
> 必须手动 `slcand`。两者别混。

### 12.2 建立 `can0`

```bash
DEV=/dev/serial/by-id/usb-Openlight_Labs_CANable2_...-if00
sudo setsid nohup slcand -o -s8 -t hw "$DEV" can0 </dev/null >/tmp/slcand.log 2>&1 &
sleep 2
sudo ip link set can0 up
ip -details -br link show can0      # 期望 UNKNOWN,UP,LOWER_UP
```

`slcand -s8` = 1 Mbps（`s0..s8` 依次 10k/20k/50k/100k/125k/250k/500k/800k/1M）。

### 12.3 一个容易误判的现象：`cansend` 能收到自己

```bash
candump -tz can0 &                 # 先起监听
cansend can0 123#DEADBEEF
# 会看到: can0  123  [4]  DE AD BE EF
```

这是 **SocketCAN 本地回环**（raw socket 默认回显本机发出的帧），
**不代表总线上有节点**。同理，`can_test` 的回显测试必须靠「ID+1」来判定，
不能靠「收到了自己发的 ID」。

### 12.4 分层排除

| 层 | 检查 | 结果 |
|---|---|---|
| 适配器 | `ls /dev/serial/by-id/`、`dmesg \| grep slcan`、`ip -br link show can0` | ✅ 正常，`can0` 起来，RX/TX 无 UART 错误 |
| 适配器速率 | `slcand -s8` vs 底盘 `BTR` 反读 | ✅ 都是 1 Mbps |
| 总线流量 | `candump -tz can0` 连续 10 s | ❌ RX = 0 帧 |
| 交叉验证速率 | 故意用 `-s6`（500k）再听，看错误计数 | ❌ 错误仍为 0 → **线上根本没有显性位** |
| 底盘固件 | 对比 `.bin` 与源码时间戳 | ⚠️ 板上是 10-18 的旧 `.bin`（26 KB），`can_link.c` 是 10-21 才写 |
| 底盘固件（复测） | 重新编译（51 KB）→ `openocd ... program ... verify reset exit` | ✅ `Verified OK`，但线上仍静默 |

### 12.5 关键证据：SWD 直接读 CAN1 寄存器

固件是否真的在发，光看现象已经分不清了，于是用 DAPLink 通过 SWD 读寄存器：

```bash
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
  -c "init; reset run; sleep 1500; halt" \
  -c "mdw 0x40006400 8"      # MCR MSR TSR RF0R RF1R IER ESR BTR
  -c "mdw 0x40006580 12"     # 三个 TX 邮箱 TI0R/TDL0R/TDH0R...
  -c "resume; exit"
```

> 必须 **先 `reset run` 让它跑一会儿再 `halt`**。复位瞬间读到的全是默认值
> （`MCR.SLEEP=1`、`MSR.INAK=0`），会误判成「CAN 没初始化」。

读到的值：

| 寄存器 | 值 | 含义 |
|---|---|---|
| `MCR` | `0x00010040` | `INRQ=0`（正常模式）、`ABOM=1` |
| `MSR` | `0x00000908` | `INAK=0`、`SLAK=0` → 控制器确实启动了 |
| `BTR` | `0x001A0002` | BRP=3 / TS1=11TQ / TS2=2TQ → **1 Mbps ✓** |
| `ESR.LEC` | `3` | **Acknowledgment error：没有节点应答** |
| `ESR.TEC` | `128` | 已进入 error-passive |
| `ESR.EWGF` | `1` | 错误告警 |
| `TSR.TME0/1/2` | 全 `0` | **三个发送邮箱被占满，帧发不出去** |
| `TI0R/TI1R/TI2R` | STID = `0x110/0x111/0x112` | 排队的就是反馈帧，内容正确 |

结论链：**固件正常启动（`s_started=1`、主循环在跑、帧已排进邮箱）→ 但总线上没人 ACK
→ 同时 Jetson 侧看到一条完全空闲的线**。两个方向都「看不见对方」，只可能是**物理层**。

### 12.6 根因：CAN_H / CAN_L 接反

接反后的电气表现（不是「收到乱码」，而是「什么都收不到」）：

- 发送节点把「显性位」反着打到线上，对端读到的仍是**隐性** → 对端看到一条**永远空闲**的总线
  → `candump` 0 帧、`ip -s` 错误计数 0（这与「线断了」表现一样，**不能靠这个区分**）
- 发送节点收不到自己的 ACK → `LEC=3`、`TEC` 爬到 128、邮箱一直不释放

**所以只量通断、量 5V 是查不出来的**，一定要确认 `H`↔`H`、`L`↔`L` 的对应关系。

### 12.7 处理与验证

1. 把 `CAN_H`/`CAN_L` 对调（适配器端或扩展板端任选一边）
2. 重插 CANable2 的 USB（拔插后 `/dev/ttyACM0` 与 `by-id` 会重建）
3. 重建 `can0`：`slcand -o -s8` + `ip link set can0 up`
4. 监听：

```bash
candump -tz can0
# 期望 100 Hz 刷出五帧轮转：
#   can0  110  [8]  ...   # 反馈字节 0..7
#   can0  111  [8]  ...   # 8..15
#   can0  112  [8]  ...   # 16..23
#   can0  113  [8]  ...   # 24..31
#   can0  114  [8]  ...   # 32..39
```

看到 `0x110~0x114` 轮转 = 底盘→Jetson 通了；再 `cansend can0 020#...` 看底盘是否响应，
即双向可用。

### 12.8 本次顺带确认的几件事

- **ZDT 步进出厂 500 kbps**：在 1 Mbps 总线上**不会应答**，属于正常现象，
  **不能用它来判断总线好坏**。必须先改成 1 Mbps（见 `CAN_CONTROL_PROTOCOL.md` §1）。
- `can_link` 的反馈是 `can_link_poll()` 里**无条件 100 Hz 发**的（`can_link.c`），
  只要 `HAL_CAN_Start()` 成功就该有流量 —— 所以「静默」几乎等价于「没跑这套固件」或「物理层不通」。
- 笔记本侧 `openocd` 需要 root（USB 权限）；`sudo` 被 `no_new_privs` 拦的环境下烧不了，
  必须换到能提权的终端。DAPLink 用 `interface/cmsis-dap.cfg`，ST-Link 用 `interface/stlink.cfg`。
