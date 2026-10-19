# Comm —— 通信层（USART2 控制链路 + CAN 链路验证）

> 本文与代码同步于 2026-10-18（git `889bae7`）。
> CAN 排障方法、物理层实测、适配器选型教训见 [`../../docs/CAN_TEST.md`](../../docs/CAN_TEST.md)，
> 帧协议与上下位机分工见 [`../../docs/CHASSIS_LIBRARY.md`](../../docs/CHASSIS_LIBRARY.md) **§18**（uart_link 协议全表）/ §15（CAN）。

## 1. 定位

本目录有**两条完全解耦的通信线**，不要混用：

- **`uart_link`（正式控制链路）**：Jetson Nano / 笔记本 ↔ STM32F407 USART2，二进制定长帧
  （12 B 命令 + 40 B 反馈），CRC-8 校验，100 Hz 回传，带看门狗与急停。这是实际跑车用的链路。
- **`can_test`（链路验证）**：只回答"F407 能不能跟 Jetson 通上话"，不是业务协议：
  强制位时序、全通过滤波器、心跳、点灯应答、回显、silent-loopback 自检、TEC/REC/LEC 诊断。

**两条线都与底盘控制解耦**：`chassis_*` 不依赖任何通信模块，两个模块也不碰底盘内部实现。

## 2. 快速接入

```c
#include "uart_link.h"
#include "can_test.h"

/* main() 的 USER CODE BEGIN 2 —— 必须在 MX_USART2_UART_Init() / MX_CAN1_Init() 之后 */
uart_link_init();      /* 已接入 main.c */
can_test_init();       /* 当前 main.c 里是注释掉的，见 §7 */

while (1) {
    chassis_poll();        /* 1 kHz 控制 */
    uart_link_poll();      /* 收帧解析 + 看门狗 + 反馈帧（内部限速） */
    can_test_poll();       /* 轮询模式，调得越勤越好 */
}
```

> ⚠️ `can_test` 的自诊断日志会往 USART1 打，和 `chassis_debug` 的调参数据抢同一个串口，
> 所以整定期间 `main.c` 里 `can_test_init()` 是注释掉的。
> ⚠️ `uart_link` 依赖 CubeMX 在 NVIC 里勾选 **USART2 global interrupt**（否则收不到字节）。

## 3. 目录结构与文件详解

### 3.1 目录结构

```
UserLib/Comm/
├── uart_link_config.h    # 协议常量：帧格式/命令字/flags/status 位/超时/环形缓冲/单位换算
├── uart_link_codec.h     # 纯函数编解码：CRC-8/ATM、小端读写、命令帧解析、反馈帧组装
├── uart_link.h           # 链路对外接口 + 状态快照 uart_link_state_t + 帧格式说明
├── uart_link.c           # 链路实现：USART2 中断收帧、环形缓冲、看门狗、反馈帧、HAL 回调
├── can_test_config.h     # CAN 验证参数：位时序/波特率/四档联动开关/帧 ID/诊断输出
├── can_test.h            # CAN 测试接口 + 状态快照 can_test_state_t
├── can_test.c            # CAN 测试实现：强制位时序、全通过滤波器、心跳/点灯/回显/自环自检/Bus-Off 恢复
└── README.md
```

### 3.2 逐个文件详解

#### `uart_link_config.h` —— 协议常量（唯一需要常改的文件）

纯宏定义，无代码。内容包括：

- **帧结构**：命令帧 `0xA5 0x5A`、反馈帧 `0x5A 0xA5`（帧头方向相反）、帧长 12 / 40 B、
  反馈类型码 `0x81`、帧尾 `0x0D`。
- **命令字**：`0x01` 速度、`0x02` 急停、`0x03` 小陀螺、`0x04` 清零。
- **flags**：`ULINK_FLAG_EN`（bit0，允许驱动）。
- **status 位**：`IMU_ONLINE / LINK_OK / WATCHDOG / ESTOP / DRIVER_ON / SPIN / GYRO_ON / IMU_CAL`。
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

#### `uart_link.h` —— 链路接口与状态结构

- `uart_link_state_t`：接收统计（`rx_frames/rx_bytes/rx_crc_err/rx_sync_err/rx_hw_err/rx_overflow`）、
  发送统计（`tx_frames/tx_drop`）、命令统计（`cmd_speed/cmd_unknown/wdg_trips`）、
  当前状态（`last_seq/last_cmd_ms/link_ok/estop`）。
- 接口：`uart_link_init()` / `uart_link_poll()` / `uart_link_state()`。
- 头文件顶部有完整帧格式图（命令帧 12 B、反馈帧 40 B）与看门狗约定。

#### `uart_link.c` —— 链路实现（设计约束全部写在文件头注释里）

- **接收路径（不阻塞）**：`HAL_UART_RxCpltCallback` 逐字节把数据推进 256 B 环形缓冲（SPSC，
  无需临界区），主循环 `ulink_parse_rx()` 找帧头（对不上滚 1 字节）→ 校验 CRC → 执行命令。
  帧解析绝不在中断里做。
- **命令执行 `ulink_handle_frame()`**：
  - `SPEED`：`vx/vy`（mm/s）×0.001、`wz`（0.01 rad/s）×0.01 → `chassis_set_velocity()`；
    EN=0 时 `chassis_stop()` + 清外环修正；
  - `ESTOP`：先 `chassis_spin(0)` 退出小陀螺 → `chassis_stop()` → `chassis_driver_enable(0)`（拉低 STBY）→ 锁存急停；
  - `SPIN`：`wz` 当自旋速率调 `chassis_spin()`；
  - `CLEAR`：里程计 + 陀螺积分 yaw 归零。
- **发送路径（不阻塞）**：`ulink_send()` 用 `HAL_UART_Transmit_IT` 排队，上一帧没发完
  （`s_tx_busy`）就丢帧计 `tx_drop`；发完由 `HAL_UART_TxCpltCallback` 清标志。
- **反馈帧 `ulink_send_feedback()`**：100 Hz 组装 x/y/yaw、正解速度、陀螺 yaw/wz、status 位、
  命令计数、反馈序号、四轮实测转速（0.01 RPM）。
- **看门狗 `ulink_watchdog()`**：控制帧超时 → 目标速度清零（走斜坡）+ 清外环修正量，
  但**不拉低 STBY**（松轴比继续转更危险）；`LINK_OK` 用 500 ms 更宽的窗口区分"抖动"和"断了"。
- **HAL 回调**：`RxCplt` 重新挂接收；`ErrorCallback`（ORE/FE/NE）计数并重挂，否则从此哑巴。

#### `can_test_config.h` —— CAN 验证参数

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

### 4.1 uart_link（正式链路）

命令帧（Jetson → F407，12 字节，小端）：

| off | 0 | 1 | 2 | 3 | 4..5 | 6..7 | 8..9 | 10 | 11 |
|---|---|---|---|---|---|---|---|---|---|---|
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

### 4.2 can_test（链路验证）

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

`can_test_config.h` 是唯一入口。**注意它是"四档联动"的**，只改一个会出现诡异现象：

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

### 6.2 can_test

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

## 7. 现状与阻塞

| 项 | 状态 |
|---|---|
| USART2 链路（uart_link） | ✅ 已接入 `main.c`，协议与 `tools/uart_link_test.py` 逐字节一致，有 PC 端交叉校验（`tools/codec_test.c`） |
| CAN 软件链路 | ✅ 已验证：silent-loopback 自检 `tx`/`rx` 每秒同步 +2，`TEC=0 REC=0 LEC=none` |
| CAN 物理层 | ✅ 已验证：TJA1050 VCC=5 V，发帧时 CANH/CANL = 3.10/1.704 V，断电测终端 118.9 Ω |
| `can_test` 在 `main.c` 集成 | ⚠️ **当前是注释掉的**，理由：自诊断打 USART1 污染调参数据 |
| Jetson 侧 CAN 适配器 | ❌ **唯一卡点**：KCAN 系列在 Linux 下不可用，推荐换 CANable 2.0 / candleLight（内核自带 `gs_usb`） |
| CAN 总线波特率 | ⚠️ **两侧不一致**：F407 实测跑 1 Mbps，Jetson 侧 `chassis_link.hpp` 默认 500 kbps → **落地前必须统一** |
| CAN 业务协议 | ❌ 未落地，`0x020`/`0x000`/`0x110` 只是预留（§16.10） |

**恢复顺序**：选定适配器 → 按 §15.3 把四个宏翻到「正常联调」档 → 统一两侧速率 →
取消 `main.c` 的注释 → 先只看心跳，通了再试回显。**别一上来就发业务帧。**

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

### 8.2 can_test

| 坑 | 现象 | 本模块怎么处理的 |
|---|---|---|
| 滤波器不配 | **一帧都收不到**，看起来像硬件坏了 | HAL 不会给默认滤波器，`init()` 里自己 `ConfigFilter`，掩码全 0 = 全通过 |
| CAN1/CAN2 共用滤波器组 | 用 CAN2 前必须先初始化 CAN1，否则行为诡异 | 只用 CAN1，固定占滤波器第 0 组 |
| 位时序被 CubeMX 覆盖 | 重新生成代码后变成 3 TQ、采样点 66.7%，通信必然不稳 | 位时序写在 `can_test_config.h`，`init()` 里拿它**再 Init 一次**；`.ioc` 里那几个键保不住 |

附加一条：**轮询和中断不要同时开**，否则同一帧被处理两次。

## 9. 详细文档

| 想看什么 | 去哪 |
|---|---|
| uart_link 帧级细节/上位机对接 | `../../docs/CHASSIS_LIBRARY.md` **§18** |
| 为什么要自己配位时序、位时序速查表 | `docs/CAN_TEST.md` §3 / §4，本文档表见 §15.4 |
| 完整排障表（TEC/REC/LEC 怎么读） | §9、§15.6 |
| 物理层检查清单（万用表怎么量） | §10 |
| 适配器选型踩坑（哪些 Linux 下不可用） | §15.8 |
| 与 Jetson `ChassisLink` 的对接点、预留协议 | §16.9 / §16.10 |
| 总线波特率待决事项 | §16.11 |
| 协议参考实现与台架测试工具 | `../../tools/README.md` |
