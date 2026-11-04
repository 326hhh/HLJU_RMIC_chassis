# Comm —— 通信层（USART2 链路 + CAN 正式链路 + CAN 验证）

> 本文与代码同步于 2026-10-21（含工作区未提交的 `can_link` 正式链路）。
> 本文只讲**通信层**：`uart_link` 协议细节见 §18、CAN 双链路与排障速查见 §15。
> 合并后的完整文档在 [`../../docs/README.md`](../../docs/README.md)
> （底盘库 §1~§14 · CAN §15 · 上位机 §16 · IMU §17 · uart_link §18 · 版本记录 §19）。

## 1. 定位

本目录有**三条通信线**，相互解耦：

- **`uart_link`（串口正式链路）**：Jetson Nano / 笔记本 ↔ STM32F407 USART2，二进制定长帧
  （12 B 命令 + 40 B 反馈），CRC-8 校验，100 Hz 回传，带看门狗与急停。这是最早跑车用的链路。
- **`can_link`（CAN 正式链路）**：Jetson ↔ STM32F407 CAN1（1 Mbps 标准帧）。**协议与串口版逐字段一致**
  ——把同一套 12 B 命令帧 / 40 B 反馈帧「搬运」到 CAN 上：命令 1 帧（`0x020`）+ 标志 1 帧（`0x021`），
  反馈 40 B 按 8 字节切 5 帧（`0x110~0x114`）。命令落地 / CRC / 看门狗全部复用 `uart_link`，
  两条链路行为完全一致。**与 `can_test` 编译期互斥。**
- **`can_test`（链路验证）**：只回答"F407 能不能跟 Jetson 通上话"，不是业务协议：
  强制位时序、全通过滤波器、心跳、点灯应答、回显、silent-loopback 自检、TEC/REC/LEC 诊断。
  由 `CANTEST_ENABLE` 编译开关控制。

> `uart_link` 与 `can_link` 可以**同时启用**（Jetson 侧 `--device` 选串口或 `--chassis can0` 选 CAN），
> 谁的命令先到谁生效（last writer wins）；`can_link` 与 `can_test` 都要初始化 CAN1，**不能同时开**。

**三条线都与底盘控制解耦**：`chassis_*` 不依赖任何通信模块，各模块也不碰底盘内部实现。

## 2. 快速接入

```c
#include "uart_link.h"
#include "can_link.h"     /* 只 include 本头文件即可拿到 CAN_LINK_ENABLE */
#include "can_test.h"

/* main() 的 USER CODE BEGIN 2 —— 必须在 MX_USART2_UART_Init() / MX_CAN1_Init() 之后 */
uart_link_init();      /* 已接入 main.c */

while (1) {
    chassis_poll();        /* 1 kHz 控制 */
    uart_link_poll();      /* 串口：收帧解析 + 看门狗 + 反馈帧（内部限速） */

#if CAN_LINK_ENABLE
    can_link_poll();       /* CAN：收帧 + 看门狗 + 100 Hz 反馈 + TX pump */
#endif
#if CANTEST_ENABLE
    can_test_poll();       /* 轮询模式，调得越勤越好 */
#endif
}
```

> `main.c` 里两条 CAN 分支都用 `#if` 包住，编译开关见 `can_link_config.h` / `can_test_config.h`，
> 切换时**不需要改注释**，也不怕 CubeMX 重新生成。
>
> ⚠️ `can_test` 的自诊断日志会往 USART1 打，和 `chassis_debug` 的调参数据抢同一个串口，
> 所以整定期间 `CANTEST_ENABLE=0`（默认）。
> ⚠️ `uart_link` 依赖 CubeMX 在 NVIC 里勾选 **USART2 global interrupt**（否则收不到字节）。

## 3. 目录结构与文件详解

### 3.1 目录结构

```
UserLib/Comm/
├── uart_link_config.h    # 串口协议常量：帧格式/命令字/flags/status 位/超时/环形缓冲/单位换算
├── uart_link_codec.h     # 纯函数编解码：CRC-8/ATM、小端读写、命令帧解析、反馈帧组装
├── uart_link.h           # 串口链路对外接口 + 状态快照 + 帧格式说明 + 供 CAN 复用的 4 个入口
├── uart_link.c           # 串口链路实现：USART2 中断收帧、环形缓冲、看门狗、反馈帧、HAL 回调
├── can_link_config.h     # CAN 链路参数：帧 ID、位时序、队列深度、错误阈值、CAN_LINK_ENABLE 开关
├── can_link.h            # CAN 链路对外接口（init/poll/state）+ 状态快照 can_link_state_t
├── can_link.c            # CAN 链路实现：CAN 帧 ↔ 12B/40B 协议帧搬运、TX 队列、错误监测
├── can_test_config.h     # CAN 验证参数：位时序/波特率/四档联动开关/帧 ID/诊断/CANTEST_ENABLE 开关
├── can_test.h            # CAN 测试接口 + 状态快照 can_test_state_t
├── can_test.c            # CAN 测试实现：强制位时序、全通过滤波器、心跳/点灯/回显/自环自检/Bus-Off 恢复
└── README.md
```

### 3.2 逐个文件详解

#### `uart_link_config.h` —— 串口协议常量（唯一需要常改的文件）

纯宏定义，无代码。内容包括：

- **帧结构**：命令帧 `0xA5 0x5A`、反馈帧 `0x5A 0xA5`（帧头方向相反）、帧长 12 / 40 B、
  反馈类型码 `0x81`、帧尾 `0x0D`。
- **命令字**：`0x01` 速度、`0x02` 急停、`0x03` 小陀螺、`0x04` 清零。
- **flags**：`ULINK_FLAG_EN`（bit0，允许驱动）。
- **status 位**：`IMU_ONLINE / LINK_OK / WATCHDOG / ESTOP / DRIVER_ON / SPIN / GYRO_ON / IMU_CAL`，
  以及 **`CAN_ERR = 0x0100`**（CAN 链路注入，TEC/REC > 96 时置位；串口链路下恒 0）。
- **时序**：反馈 10 ms（100 Hz）、控制帧超时 100 ms、链路丢失 500 ms。
- **缓冲与换算**：RX 环形缓冲 256 B（必须是 2 的幂）、`mm/s → m/s`、`0.01 rad/s` 单位换算。

#### `uart_link_codec.h` —— 纯函数编解码（无硬件依赖）

- `ulink_cmd_t` / `ulink_fb_t`：命令帧与反馈帧的字段视图（结构体）。
- `ulink_crc8()`：CRC-8/ATM（poly 0x07、init 0x00，校验值 "123456789" = 0xF4）。
- `ulink_put_u16/i16/u32/i32` 与 `ulink_get_*`：小端读写工具。
- `ulink_parse_cmd()`：12 B 字节流 → 命令结构体（校验帧头/CRC）。
- `ulink_build_fb()`：反馈结构体 → 40 B 字节流（含 CRC 与帧尾）。
- 之所以单独抽出来：帧格式错误（偏移/大小端/CRC 覆盖范围）在实机上只表现为"收不到/乱跳"，
  抽成纯函数后可以拿 gcc 在 PC 上编译，用 `tools/uart_link_test.py vectors` 生成的向量做双向交叉校验
  （见 `tools/codec_test.c`）。

#### `uart_link.h` —— 串口链路接口与状态结构

- `uart_link_state_t`：接收统计（`rx_frames/rx_bytes/rx_crc_err/rx_sync_err/rx_hw_err/rx_overflow`）、
  发送统计（`tx_frames/tx_drop`）、命令统计（`cmd_speed/cmd_unknown/wdg_trips`）、
  当前状态（`last_seq/last_cmd_ms/link_ok/estop`）。
- 接口：`uart_link_init()` / `uart_link_poll()` / `uart_link_state()`。
- **供 CAN 链路复用的 4 个入口**（`can_link.c` 调用，保证两条链路语义只有一份实现）：

  | 函数 | 作用 |
  |---|---|
  | `ulink_apply_cmd12(const uint8_t *f)` | 施加一帧 12 B 命令（帧头 + CRC 校验 + 命令落地） |
  | `ulink_build_fb_frame(uint8_t *out, uint32_t now)` | **只组** 40 B 反馈，不发送 |
  | `ulink_tick_watchdog(uint32_t now)` | 看门狗（两条链路共用同一套超时状态，重复调用无害） |
  | `ulink_set_extra_status(uint16_t bits)` | 传输层向 `status` 注入额外位（供 `CAN_ERR` 用） |

- 头文件顶部有完整帧格式图（命令帧 12 B、反馈帧 40 B）与看门狗约定。

#### `uart_link.c` —— 串口链路实现（设计约束全部写在文件头注释里）

- **接收路径（不阻塞）**：`HAL_UART_RxCpltCallback` 逐字节把数据推进 256 B 环形缓冲（SPSC，
  无需临界区），主循环 `ulink_parse_rx()` 找帧头（对不上滚 1 字节）→ 校验 CRC → 执行命令。
  帧解析绝不在中断里做。
- **命令执行 `ulink_apply_cmd12()`**（串口和 CAN 共用）：
  - `SPEED`：`vx/vy`（mm/s）×0.001、`wz`（0.01 rad/s）×0.01 → `chassis_set_velocity()`；
    EN=0 时 `chassis_stop()` + 清外环修正；
  - `ESTOP`：先 `chassis_spin(0)` 退出小陀螺 → `chassis_stop()` → `chassis_driver_enable(0)`（拉低 STBY）→ 锁存急停；
  - `SPIN`：`wz` 当自旋速率调 `chassis_spin()`；
  - `CLEAR`：里程计 + 陀螺积分 yaw 归零。
- **发送路径（不阻塞）**：`ulink_send()` 用 `HAL_UART_Transmit_IT` 排队，上一帧没发完
  （`s_tx_busy`）就丢帧计 `tx_drop`；发完由 `HAL_UART_TxCpltCallback` 清标志。
- **反馈帧 `ulink_build_fb_frame()`**（串口与 CAN 共用组帧）：100 Hz 组装 x/y/yaw、正解速度、
  陀螺 yaw/wz、status 位、命令计数、反馈序号、四轮实测转速（0.01 RPM）。
- **看门狗 `ulink_tick_watchdog()`**：控制帧超时 → 目标速度清零（走斜坡）+ 清外环修正量，
  但**不拉低 STBY**（松轴比继续转更危险）；`LINK_OK` 用 500 ms 更宽的窗口区分"抖动"和"断了"。
- **HAL 回调**：`RxCplt` 重新挂接收；`ErrorCallback`（ORE/FE/NE）计数并重挂，否则从此哑巴。

#### `can_link_config.h` —— CAN 链路参数

- **启用开关**：`CAN_LINK_ENABLE`（联调期 1，整定期 0）。
- **位时序**：`CAN_LINK_PRESCALER=3` → 42 MHz/3/14 TQ = 1 Mbps，采样点 85.7%。
  换速率只改 Prescaler（42 MHz/14 = 3 MHz，1M→3、500k→6、250k→12）。
- **帧 ID**：`0x000` 广播急停、`0x020` 命令、`0x021` flags、`0x110~0x114` 反馈（5 帧切分）。
- **时序**：反馈 10 ms（100 Hz）；TX 队列 20 帧深；`CAN_ERR` 阈值 96（error-passive 预警线）。

#### `can_link.h` —— CAN 链路对外接口

- `can_link_state_t`：`rx_frames / rx_ignored / rx_short / tx_frames / tx_queue_full / tx_add_fail /
  last_flags / bitrate_hz（从 BTR 寄存器反读）/ started / can_err`。
- 接口：`can_link_init()` / `can_link_poll()` / `can_link_state()`。
- 头文件顶部有完整 CAN 帧表（`0x000`/`0x020`/`0x021`/`0x110~0x114`）与「协议零重写」的关系说明。

#### `can_link.c` —— CAN 链路实现（核心设计写在文件头注释里）

- **协议零重写**：命令帧 / 反馈帧、CRC、看门狗、命令落地全部复用 `uart_link`，
  本模块只做「CAN 8 字节 ↔ 协议字节」的搬运：
  `0x020 (8B) → 拼出 12B 命令帧（flags 取自 0x021）→ ulink_apply_cmd12()`；
  `40B 反馈 → 按 8 字节切 5 帧 → 0x110~0x114`。
- **收帧 `can_link_handle_rx()`**：
  - `0x000`：DLC 任意（含 0）→ 按 ESTOP 处理（退小陀螺 → 停车 → 断 STBY → 锁存）；
  - `0x020`：DLC 必须 8，`[cmd, seq, vx(2), vy(2), wz(2)]` → 12 B 帧第 4~9 字节；
    DLC < 8 直接丢并计 `rx_short`；
  - `0x021`：只更新 `flags`；EN=0 时**立即** `chassis_spin(0)` + `chassis_stop()` +
    `chassis_gyro_reset_correction()`——但**不走命令通道**（不刷新 `cmd_cnt`、不喂看门狗）；
  - 29-bit 扩展帧（ZDT 云台）一律 `rx_ignored` 跳过，与标准帧天然隔离。
- **反馈发送 `can_link_send_feedback()`**：`ulink_build_fb_frame()` 组出 40 B，整组 5 帧要么
  全部进 TX 队列、要么整组丢弃（不发送残缺组）。
- **TX 队列 `can_link_pump_tx()`**：20 帧深环形队列；每轮 `poll()` 把队列尽量塞进 3 个发送邮箱，
  **只有 `AddTxMessage` 成功才推进 tail**；邮箱满 → 下轮再试，不丢帧。
- **错误监测 `can_link_update_err_flag()`**：读 `CAN_ESR` 的 TEC/REC，任一 > 96 即置
  `ulink_set_extra_status(ULINK_ST_CAN_ERR)`；计数回落自动清零（实时位，非锁存位）。
- **初始化 `can_link_init()`**（顺序硬要求：强制位时序 → 配滤波器 → Start）：
  不信任 CubeMX，按 `can_link_config.h` 重配 1 Mbps；全通过滤波器（掩码全 0）；
  `AutoBusOff=ENABLE`（硬件 ABOM 自动恢复，不锁死）、`AutoRetransmission=ENABLE`（有真节点应答，正常重传）。

#### `can_test_config.h` —— CAN 验证参数

- **启用开关**：`CANTEST_ENABLE`（联调/排障期 1，整定期 0），与 `CAN_LINK_ENABLE` **编译期互斥**。
- **位时序**：`CANTEST_PRESCALER=3` → 42 MHz/3/14 TQ = 1 Mbps，采样点 85.7%（改 6 为 500 kbps）。
- **四档联动开关**（只改一个会出现诡异现象）：`LOOPBACK_SELFTEST / HEARTBEAT_ENABLE /
  AUTO_RETRANSMIT / ECHO_ENABLE`。
- **帧 ID**：心跳 `0x7F0`、LED 控制 `0x200`、LED 应答 `0x201`、自检帧；灯极性宏、诊断输出开关、
  Bus-Off 自动恢复开关、轮询/中断选择。

#### `can_test.h` —— 对外接口

- `can_test_state_t`：`started / tx_count / rx_count / last_error / bus_off_seen / bitrate_hz`。
- 接口：`can_test_init()` / `can_test_poll()` / `can_test_send()` / `can_test_get_state()` /
  `can_test_is_started()` / `can_test_last_error()`。

#### `can_test.c` —— CAN 验证实现

- **强制位时序**：`can_test_init()` 不信任 CubeMX，拿 `can_test_config.h` 的值再 `HAL_CAN_Init()` 一次
  （`.ioc` 里的位时序键保不住）。
- **全通过滤波器**：自己 `ConfigFilter`（掩码全 0 = 全通过），占第 0 组——F407 收不到帧的头号原因。
- **行为**：500 ms 心跳（`0x7F0`，带序号）、LED 控制帧 `0x200` 应答 `0x201`、收到任意帧把 ID+1 回显；
  `LOOPBACK_SELFTEST=1` 时用 silent-loopback 片内自环（不上总线）；Bus-Off 自动重启 CAN；
  每秒往 USART1 打一行 TEC/REC/LEC/收发计数诊断。

## 4. 帧格式约定

### 4.1 uart_link（串口正式链路）

命令帧（Jetson → F407，12 字节，小端）：

| off | 0 | 1 | 2 | 3 | 4..5 | 6..7 | 8..9 | 10 | 11 |
|---|---|---|---|---|---|---|---|---|---|
| 字段 | 0xA5 | 0x5A | cmd | seq | vx(mm/s) | vy(mm/s) | wz(0.01rad/s) | flags | crc8 |

反馈帧（F407 → Jetson，40 字节，小端，100 Hz）：

| off | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0..1 | 0x5A 0xA5 | u8×2 | 帧头（与命令帧相反） |
| 2 | 0x81 | u8 | 反馈类型码 |
| 3 | seq | u8 | 回显最近一次合法命令的 seq |
| 4..7 / 8..11 | x / y | i32 | 里程计位置，mm |
| 12..13 / 14..15 | yaw / vx | i16 | 0.01° / mm/s |
| 16..17 / 18..19 | vy / wz | i16 | mm/s / 0.01 rad/s |
| 20..21 / 22..23 | gyro_yaw / gyro_wz | i16 | 0.01° / 0.01 rad/s |
| 24..25 / 26..27 / 28..29 | status / cmd_cnt / fb_seq | u16×3 | 状态位 / 命令计数 / 反馈序号 |
| 30..37 | wheel_rpm[4] | i16×4 | 四轮实测转速，0.01 RPM，FL FR BL BR |
| 38 / 39 | crc8 / 0x0D | u8×2 | 校验 / 帧尾 |

> 坐标约定：`vx` 前进 / `vy` 左移 / `wz` 逆时针为正（与底盘库一致，**不是**上位机
> `ChassisCommand` 的「右/前」老约定）。

### 4.2 CAN 链路（正式协议，`can_link`）

全部 **11-bit 标准帧**；ZDT 云台的 29-bit 扩展帧在同一总线上天然隔离。

| ID | 方向 | 内容 | 数据 |
|---|---|---|---|
| `0x000` | Jetson → F407 | 广播急停（DLC 任意，含 0） | 无（ID 最小 = 仲裁最高优先级，可插队打断一切流量） |
| `0x020` | Jetson → F407 | 速度/模式命令（100 Hz） | `[cmd, seq, vx(i16), vy(i16), wz(i16)]` |
| `0x021` | Jetson → F407 | flags 标志帧（10 Hz 保活 + 变化即发） | `[flags]`（bit0 = EN） |
| `0x110` | F407 → Jetson | 反馈第 0 帧（bytes 0..7） | `[5A A5 81 seq x[0..3]]` |
| `0x111` | F407 → Jetson | 反馈第 1 帧（bytes 8..15） | `[x[4..7] y[0..3]]` |
| `0x112` | F407 → Jetson | 反馈第 2 帧（bytes 16..23） | `[y[4..7] yaw vx vy]` |
| `0x113` | F407 → Jetson | 反馈第 3 帧（bytes 24..31） | `[wz gyro_yaw gyro_wz status cmd_cnt fb_seq[0]]` |
| `0x114` | F407 → Jetson | 反馈第 4 帧（bytes 32..39） | `[fb_seq[1] 四轮RPM×4 crc8 0D]` |

`cmd`：`0x01` 速度 / `0x02` 应用级急停（保留）/ `0x03` 小陀螺 / `0x04` 清零。
**接收端必须实现组完整性规则**（否则会出现"偶发状态跳变"）：
`0x110` 开新组 → `0x111~0x114` 按序填入 → 下一个 `0x110` 冲掉未满旧组 → 组超时 5 ms 丢弃 →
`crc8` 失败整组丢弃。

**语义细节**：`0x021` 是标志帧不是命令帧——EN=0 立即停车，但**不计入 `cmd_cnt`**、不刷新看门狗；
`cmd_cnt` 只统计 `0x020` 与 `0x000`。`CAN_ERR` 为实时位（TEC/REC > 96 亮，回落即灭）。

### 4.3 can_test（链路验证）

标准帧（11 位）给 Jetson ↔ F407；扩展帧（29 位）留给云台 ZDT 步进，两者天然隔离不冲突。

| ID | 方向 | 内容 | 数据 |
|---|---|---|---|
| `0x7F0` | F407 → 上位机 | 心跳（500 ms） | `[seq, 0xCA, 0xFE, rx_count & 0xFF]` |
| `0x200` | 上位机 → F407 | LED 控制 | `[cmd]`，`0`=灭 `1`=亮 `2`=翻转 |
| `0x201` | F407 → 上位机 | LED 应答 | `[cmd, 当前状态, rx_count & 0xFF, 0xA5]` |
| 收到的 ID + 1 | F407 → 上位机 | 回显 | 原样转发 |

未知 `cmd` 不改灯但仍回 `0x201`（让上位机确认链路活着）；`0xA5` 是应答帧专属魔数。
收到扩展帧统一用固定标准帧 `0x123` 回。

## 5. 配置开关

### 5.1 CAN 总线（can_link 与 can_test 互斥）

```c
/* can_link_config.h —— 联调期 1，整定期 0 */
#define CAN_LINK_ENABLE  1

/* can_test_config.h —— 排障期 1，平时 0 */
#define CANTEST_ENABLE   0
```

两者**编译期互斥**（都要初始化 CAN1）：

```c
/* can_link.c 顶部 */
#if CAN_LINK_ENABLE && CANTEST_ENABLE
#error "can_link 与 can_test 不能同时启用：两者都要初始化 CAN1，会互相打架"
#endif
```

`can_test_config.h` 是 can_test 的唯一配置入口。注意它是"四档联动"的，只改一个会出现诡异现象：

| 用途 | `LOOPBACK_SELFTEST` | `HEARTBEAT_ENABLE` | `AUTO_RETRANSMIT` | `ECHO_ENABLE` |
|---|---|---|---|---|
| 正常联调（接真适配器） | `0` | `1` | `1` | `1` |
| 诊断（对面不响应） | `0` | `0` | `0` | `0` |
| **自检（无适配器）← 当前默认** | `1` | `0` | `0` | `0` |

其余常改项：

| 宏 | 默认 | 说明 |
|---|---|---|
| `CANTEST_PRESCALER` | `3` | 42 MHz / 3 / 14 TQ = **1 Mbps**，采样点 85.7%。改 `6` 就是 500 kbps |
| `CANTEST_HEARTBEAT_MS` | `500` | 心跳周期 |
| `CANTEST_LED_ACTIVE_HIGH` | `1` | PB2 极性；**灯亮灭反了就只改这一个宏** |
| `CANTEST_USE_RX_INTERRUPT` | `0` | 轮询收帧；要改中断得先在 CubeMX 里勾 NVIC，**两者不能同时开** |
| `CANTEST_AUTO_BUSOFF_RECOVER` | `1` | Bus-Off 自动重启 CAN |
| `CANTEST_SERIAL_DEBUG` | `1` | 每秒往 USART1 打一行诊断（TEC/REC/LEC/tx/rx） |

uart_link 的常改项在 `uart_link_config.h`：`ULINK_FB_PERIOD_MS`（反馈周期，100 Hz）、
`ULINK_CMD_TIMEOUT_MS`（100 ms 看门狗）、`ULINK_FB_FRAME_LEN`、命令字/status 位等。

## 6. 上位机验证

### 6.1 uart_link

```bash
# 笔记本 / Jetson 接 USART2（USB-TTL）后：
python3 tools/uart_link_test.py listen -p /dev/ttyUSB0          # 只看反馈帧（验证 F407 在发）
python3 tools/uart_link_test.py drive  -p /dev/ttyUSB0 --vx 100 --secs 3   # 前进 0.1 m/s
python3 tools/uart_link_test.py spin   -p /dev/ttyUSB0 --rate 350 --secs 3 # 小陀螺 3.5 rad/s
python3 tools/uart_link_test.py estop  -p /dev/ttyUSB0          # 急停
python3 tools/uart_link_test.py clear  -p /dev/ttyUSB0          # 里程计清零
python3 tools/uart_link_test.py selfcheck                       # 纯软件自检，不需要串口
```

### 6.2 can_test（链路验证）

```bash
# ① 看心跳 —— 正常联调档下应每 0.5 秒一行
candump can0
#    期望: can0  7F0   [4]  00 CA FE 00
#          can0  7F0   [4]  01 CA FE 00

# ② 双向回显
cansend can0 123#A1B2C3D4
#    期望: can0  124   [4]  A1 B2 C3 D4
```

> 自检档用的是 silent-loopback：帧在片内自己绕一圈回来，**不上总线、不影响总线上的其他节点**，
> 所以总线上抓包的邻居节点看不到任何帧，这是正常的，不是坏了。想要 `candump` 看见心跳，先按第 5 节翻到「正常联调」档。

### 6.3 can_link（正式协议，Jetson 侧运行 gimbal 上位机）

```bash
# Jetson：CANable 2.0 先起 can0（编号随插拔变化，以 ls /dev/ttyACM* 为准）
sudo slcand -o -s8 -t hw /dev/ttyACM1 can0 && sudo ip link set can0 up
ip -br link show can0                      # 应看到 <NOARP,UP,LOWER_UP>

# 只抓包核对：0x110~0x114 应每 10ms 一组、组内连续
candump -td can0

# 手发命令（注意：速度命令必须先发 EN=1 且 F407 的 CAN_LINK_ENABLE=1）
cansend can0 021#01                        # EN 开 → 允许驱动
cansend can0 020#01 01 64 00 00 00 00 00  # cmd=01 seq=01 vx=100mm/s vy=0 wz=0
candump can0 | grep 113                    # 从 0x113 看 status 位翻转与 cmd_cnt

# 急停（DLC=0 也认）
cansend can0 000#
```

> 组规则、组内延迟、`CAN_ERR`、实测记录见
> `docs/README.md` §15.7.2 / §15.12。

## 7. 现状与阻塞

| 项 | 状态 |
|---|---|
| USART2 链路（uart_link） | ✅ 已接入 `main.c`，协议与 `tools/uart_link_test.py` 逐字节一致，有 PC 端交叉校验（`tools/codec_test.c`） |
| CAN 正式链路（can_link） | ✅ 已实现（命令 `0x000`/`0x020`/`0x021` + 反馈 `0x110~0x114` 五帧 + TX 队列 + `CAN_ERR`），Jetson 侧 `CanChassisLink` 实测自检全绿（`docs/README.md` §15.10.1） |
| CAN 软件链路 | ✅ 已验证：silent-loopback 自检 `tx`/`rx` 每秒同步 +2，`TEC=0 REC=0 LEC=none` |
| CAN 物理层 | ✅ 已验证：TJA1050 VCC=5 V，发帧时 CANH/CANL = 3.10/1.704 V，断电测终端 118.9 Ω |
| `can_test` 在 `main.c` 集成 | ✅ 改为 `CANTEST_ENABLE` 编译开关（默认 0，整定期关，不污染 USART1 调参数据），与 `can_link` 编译期互斥 |
| Jetson 侧 CAN 适配器 | ✅ CANable 2.0（SLCAN 固件，内核自带 `slcan.ko`）已打通，`candump`/`cansend` 实测正常 |
| CAN 总线波特率 | ✅ 两侧统一 **1 Mbps**（F407 与 Jetson `can0`） |
| 云台 ZDT 切 CAN 模式 | ⏳ 待做：面板/上位机切 CAN 模式 + 1 Mbps，切换后 `candump` 应看到扩展帧应答 |
| 全 CAN 端到端联调 | ⏳ 待做：底盘 `0x020`/`0x110~0x114` + 云台 ZDT 扩展帧同总线混跑 |

## 8. 经典坑

### 8.1 uart_link

| 坑 | 现象 | 本模块怎么处理的 |
|---|---|---|
| 在中断里做帧解析 | 主循环卡顿、丢字节 | ISR 只推环形缓冲，解析全在 `poll()` 里 |
| 阻塞式发送反馈帧 | 1 kHz 控制环掉拍 | 用 `HAL_UART_Transmit_IT` 排队，发不完就丢帧 |
| 看门狗直接拉 STBY | 云台/底盘松轴掉下来，比继续转危险 | 只清速度、保持 STBY，只有显式 ESTOP 才断输出 |
| 忘了重挂 `HAL_UART_Receive_IT` | 收到的字节永远不完整 | RxCplt/ErrorCallback 里都自动重挂 |
| 环形缓冲长度不是 2 的幂 | 取模错乱 | 编译期 `typedef char[...]` 静态断言 |
| 速度指令帧没带 EN 位 | 底盘不动 | EN=0 是"停车保驱动"，要动就置 `ULINK_FLAG_EN` |

### 8.2 can_link

| 坑 | 现象 | 本模块怎么处理的 |
|---|---|---|
| 5 帧反馈不是原子的 | 偶发"底盘状态跳变" | 接收端按 §15.7.2 组规则拼装；坏组/超时整组丢弃 |
| F407 只有 3 个发送邮箱 | 一帧都发不出去、`err=0x00200000` | 20 帧深 TX 队列 + 能发就发（成功才出队），组内 ≤2 ms |
| `0x021` EN=0 被当成命令帧 | 反馈 `cmd_cnt` 莫名 +1、丢包率统计错乱 | EN=0 直接停车但不走命令通道（不刷新 `cmd_cnt`、不喂看门狗） |
| 两组都初始化 CAN1 | 编译期互相打架 | `#if CAN_LINK_ENABLE && CANTEST_ENABLE` → `#error` |
| 接收端把 ZDT 扩展帧也拼进反馈组 | 组规则错乱 | 只处理 11-bit 标准帧，29-bit 扩展帧计 `rx_ignored` |
| 用严格 `>= 10ms` 判反馈周期 | 因抖动隔一拍才发（~20ms/帧） | Jetson 侧加 0.9x 容差，保持 ~100Hz |

### 8.3 can_test

| 坑 | 现象 | 本模块怎么处理的 |
|---|---|---|
| 滤波器不配 | **一帧都收不到**，看起来像硬件坏了 | HAL 不会给默认滤波器，`init()` 里自己 `ConfigFilter`，掩码全 0 = 全通过 |
| CAN1/CAN2 共用滤波器组 | 用 CAN2 前必须先初始化 CAN1，否则行为诡异 | 只用 CAN1，固定占滤波器第 0 组 |
| 位时序被 CubeMX 覆盖 | 重新生成代码后变成 3 TQ、采样点 66.7%，通信必然不稳 | 位时序写在 `can_test_config.h`，`init()` 里拿它**再 Init 一次**；`.ioc` 里那几个键保不住 |

附加一条：**轮询和中断不要同时开**，否则同一帧被处理两次。

## 9. 详细文档

| 想看什么 | 去哪 |
|---|---|
| uart_link 帧级细节/上位机对接 | `../../docs/README.md` **§18** |
| CAN 正式协议：帧表 / 组规则 / 语义细节 / 联调记录 | `../../docs/README.md` **§15.7 ~ §15.10** |
| 为什么要自己配位时序、位时序速查表 | `../../docs/README.md` §15.1 / §15.2 |
| 完整排障表（TEC/REC/LEC 怎么读） | `../../docs/README.md` §15.5 |
| 物理层检查清单（万用表怎么量） | `../../docs/README.md` §15.5 |
| 适配器选型踩坑（哪些 Linux 下不可用） | `../../docs/README.md` §15.11 |
| CAN 全静默（CAN_H/L 接反）复盘 | `../../docs/README.md` §15.12 |
| 与 Jetson `ChassisLink` 的对接点与协议 | `../../docs/README.md` §16.7 |
| 协议参考实现与台架测试工具 | `../../tools/README.md` |
