# Comm —— CAN 链路验证模块

> 本文与代码同步于 2026-10-18（git `889bae7`）。
> 完整文档：排障方法、物理层实测、适配器选型教训见 [`../../docs/CAN_TEST.md`](../../docs/CAN_TEST.md)，
> 帧协议与上下位机分工见 [`../../docs/CHASSIS_LIBRARY.md`](../../docs/CHASSIS_LIBRARY.md) §15 / §16。

## 1. 定位

`can_test` **只验证"F407 能不能跟 Jetson 通上话"，不是业务协议**。它回答三个问题：

1. 位时序对不对（1 Mbps / 采样点 85.7%）—— 不信 CubeMX，`init()` 里强制重配一次
2. 滤波器配了没有 —— F407 上收不到帧的头号原因，这里用「掩码全 0 = 全通过」
3. 收发链路通不通 —— 心跳 / 点灯 / 回显 / silent-loopback 自检 / TEC·REC·LEC 分级诊断

**CAN 与底盘控制目前是彻底解耦的两条线**：`chassis_*` 不依赖 CAN，`can_test` 也不碰底盘。

## 2. 快速接入

```c
#include "can_test.h"

/* main() 的 USER CODE BEGIN 2 —— 必须在 MX_CAN1_Init() 之后（那里才开时钟和 GPIO） */
can_test_init();

while (1) {
    can_test_poll();          /* 轮询模式，调得越勤越好 */
    /* ... */
}
```

> ⚠️ 这两行**当前在 `main.c` 里是注释掉的**，原因见第 6 节：自诊断日志会往 USART1 打，
> 和 `chassis_debug` 的调参数据抢同一个串口。

## 3. 文件清单

| 文件 | 行数 | 职责 |
|---|---|---|
| `can_test_config.h` | 156 | **唯一需要改的文件**：位时序、四档行为开关、帧 ID |
| `can_test.c` | 454 | 实现：强制位时序、全通过滤波器、心跳、点灯应答、回显、自检、Bus-Off 自动恢复 |
| `can_test.h` | 74 | 公开接口 + 状态快照 `can_test_state_t` |

依赖 CubeMX 生成的 `Core/Src/can.c`（提供 `hcan1`）与 `main.h`（`LED_Pin` = PB2）。

## 4. 帧格式约定

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

## 6. 上位机验证

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
| F407 软件链路 | ✅ 已验证：silent-loopback 自检 `tx`/`rx` 每秒同步 +2，`TEC=0 REC=0 LEC=none` |
| 物理层 | ✅ 已验证：TJA1050 VCC=5 V，发帧时 CANH/CANL = 3.10/1.704 V，断电测终端 118.9 Ω |
| `main.c` 集成 | ⚠️ **当前是注释掉的**，理由：自诊断打 USART1 污染调参数据 |
| Jetson 侧 CAN 适配器 | ❌ **唯一卡点**：KCAN 系列在 Linux 下不可用，推荐换 CANable 2.0 / candleLight（内核自带 `gs_usb`） |
| 总线波特率 | ⚠️ **两侧不一致**：F407 实测跑 1 Mbps，Jetson 侧 `chassis_link.hpp` 默认 500 kbps → **落地前必须统一** |
| 业务协议 | ❌ 未落地，`0x020`/`0x000`/`0x110` 只是预留（§16.10） |

**恢复顺序**：选定适配器 → 按 §15.3 把四个宏翻到「正常联调」档 → 统一两侧速率 →
取消 `main.c` 的注释 → 先只看心跳，通了再试回显。**别一上来就发业务帧。**

## 8. 三个经典坑（`can_test.c` 里都已处理）

| 坑 | 现象 | 本模块怎么处理的 |
|---|---|---|
| 滤波器不配 | **一帧都收不到**，看起来像硬件坏了 | HAL 不会给默认滤波器，`init()` 里自己 `ConfigFilter`，掩码全 0 = 全通过 |
| CAN1/CAN2 共用滤波器组 | 用 CAN2 前必须先初始化 CAN1，否则行为诡异 | 只用 CAN1，固定占滤波器第 0 组 |
| 位时序被 CubeMX 覆盖 | 重新生成代码后变成 3 TQ、采样点 66.7%，通信必然不稳 | 位时序写在 `can_test_config.h`，`init()` 里拿它**再 Init 一次**；`.ioc` 里那几个键保不住 |

附加一条：**轮询和中断不要同时开**，否则同一帧被处理两次。

## 9. 详细文档

| 想看什么 | 去哪 |
|---|---|
| 为什么要自己配位时序、位时序速查表 | `docs/CAN_TEST.md` §3 / §4，本文档表见 §15.4 |
| 完整排障表（TEC/REC/LEC 怎么读） | §9、§15.6 |
| 物理层检查清单（万用表怎么量） | §10 |
| 适配器选型踩坑（哪些 Linux 下不可用） | §15.8 |
| 与 Jetson `ChassisLink` 的对接点、预留协议 | §16.9 / §16.10 |
| 总线波特率待决事项 | §16.11 |
