/**
  ******************************************************************************
  * @file    can_test_config.h
  * @brief   CAN 通信测试参数（所有可调参数集中在这里，改完重新编译即可）
  *
  * 用途：验证 STM32(F407) <-> Jetson(Nano + KCAN-USB Pro) 的 CAN 通信
  *
  * 约定：
  *   CAN1 引脚 = PB8(CAN1_RX) / PB9(CAN1_TX)，APB1 = 42MHz
  *   标准帧 11 位；扩展帧留给云台 ZDT 步进（29 位，天然隔离不冲突）
  *
  * ⚠️ 位时序为什么写在代码里而不是 CubeMX：
  *   本工程 .ioc 里只有 CubeMX 算出来的 Calculate* 值，没有 Prescaler/BS1/BS2/SJW
  *   这几个键，重新生成时保不住。can_test_init() 会用这里的值强制重配一次，
  *   所以不管 CubeMX 里怎么点，实际生效的都是本文件。
  *   实际生效的速率可以运行时读回来：can_test_get_state().bitrate_hz
  ******************************************************************************
  */
#ifndef USERLIB_COMM_CAN_TEST_CONFIG_H
#define USERLIB_COMM_CAN_TEST_CONFIG_H

#include "stm32f4xx_hal.h"

/* ============================== 时间基准 ================================== */
/** CAN1 挂 APB1。本工程 PLLM=4/PLLN=168/PLLP=2 → HCLK 168MHz，APB1 = /4 = 42MHz */
#define CANTEST_APB1_HZ              42000000UL

/* ============================== 位时序 ====================================
 * 目标 1 Mbps，与 Jetson 侧 can0/can1 一致。
 *
 * 验算：42MHz / 3 = 14MHz；一位 = 1(同步) + 11(BS1) + 2(BS2) = 14 TQ
 *       位速率 = 14MHz / 14 = 1.000 Mbps
 *       采样点 = (1+11)/14 = 85.714%     （CAN 推荐 75%~87.5%）
 *
 * 改成 500kbps：把 CANTEST_PRESCALER 改成 6 即可（采样点不变）。
 *
 * 注意 BS1_TQ / BS2_TQ 这两个【纯数字】只是给人看的注释性常量，
 * 真正生效的是下面两个 HAL 宏。实际速率由 can_test_read_bitrate()
 * 从 CAN_BTR 寄存器反读验证，不依赖这两个数字。
 */
#define CANTEST_PRESCALER            3U
#define CANTEST_BS1                  CAN_BS1_11TQ   /* = 11 TQ */
#define CANTEST_BS2                  CAN_BS2_2TQ    /* =  2 TQ */
#define CANTEST_SJW                  CAN_SJW_1TQ

#define CANTEST_BS1_TQ               11U
#define CANTEST_BS2_TQ               2U

/* ============================ 测试行为 ==================================== */
/** 心跳周期 (ms)：每这么久发一帧，让 Jetson 一 candump 就知道板子活着 */
#define CANTEST_HEARTBEAT_MS         500U

/** 心跳帧 ID（标准帧 11 位）
 *  ⚠️ 回显用的是「收到的 ID + CANTEST_ECHO_OFFSET」，所以心跳 ID 放高段，
 *     只有对方发 0x7EF 时才会撞上。 */
#define CANTEST_HEARTBEAT_ID         0x7F0U

/** 回显 ID 偏移：回显 ID = 收到的 ID + 该值 */
#define CANTEST_ECHO_OFFSET          1U

/** 收到扩展帧时，统一用这个标准帧 ID 回（方便 Jetson 侧识别） */
#define CANTEST_EXT_ECHO_ID          0x123U

/** 1 = 用 CAN1_RX0 中断收；0 = 在主循环里轮询收（默认）
 *  ⚠️ 本工程 CubeMX 里没开 CAN1 中断，所以默认用轮询。
 *     要用中断：CubeMX → System Core → NVIC → 勾 "CAN1 RX0 interrupts"，
 *     再把这里改成 1。两者不要同时开，否则同一帧会被处理两次。 */
#define CANTEST_USE_RX_INTERRUPT     0

/** 1 = 进入 Bus-Off 时自动重启 CAN（推荐开） */
#define CANTEST_AUTO_BUSOFF_RECOVER  1

/* ============================ LED 控制（PB2） =============================
 * Jetson 发一帧就点灯，用来验证「指令真的被 F407 收到并执行了」——
 * 比看回显更硬，因为回显只能证明收到，证明不了执行。
 *
 *   Jetson 发:  cansend can0 200#01      亮
 *               cansend can0 200#00      灭
 *               cansend can0 200#02      翻转（第一次验极性用这个）
 *   F407 回:    201#01 01 1F A5          [cmd, 当前状态, 已收帧数低8位, 魔数]
 *
 * 未知 cmd 不改灯，但仍回 ACK 报告当前状态，方便 Jetson 侧确认链路活着。
 */
#define CANTEST_LED_ID               0x200U   /* 控制帧 ID（Jetson → F407，标准帧） */
#define CANTEST_LED_ACK_ID           0x201U   /* 应答帧 ID（F407 → Jetson） */

#define CANTEST_LED_CMD_OFF          0U
#define CANTEST_LED_CMD_ON           1U
#define CANTEST_LED_CMD_TOGGLE       2U

/** PB2 的 LED 极性：1 = 写高电平灯亮；0 = 写低电平灯亮。
 *  ⚠️ 原理图 PDF 抽不出网络名，极性没查实。先用 200#02 翻转试一下，
 *     如果灯的行为整体反了，把这里改成 0 重新编译即可（只影响这一处）。 */
#define CANTEST_LED_ACTIVE_HIGH      1

/* ======================= 诊断开关（排查通信故障用） =======================
 * 症状：F407 的 tx 卡在 3、err=0x00200000（三个发送邮箱全满）、rx 恒为 0，
 *       总线上量到 CANH=3.1V / CANL=1.7V（显性），说明 F407 在无限重传、霸占总线。
 *
 * 打开这两个开关后，F407 变成「纯监听」：不说话、不吵架，只老老实实报错误计数器。
 * 这样才能看清对面到底在发什么：
 *
 *   TEC > 0        -> 自己发送出错（正常，关重传后每试一次 +8）
 *   REC > 0        -> !!! 听到了波形但解不出来 = 位速率不一致 !!!
 *   REC = 0        -> 听不到任何东西 = 物理层断的
 *   LEC = ACK      -> 只有应答错误 = 总线是空的，没人在应答
 *   LEC = STUFF/FORM/CRC -> 收到的位流解不出来 = 速率/采样点不匹配
 *
 * 通信调通后，把这两个改回 1 和 1（或直接删掉这段）。
 */
#define CANTEST_HEARTBEAT_ENABLE     0   /* 心跳：自检时关掉，免得 0x7F0 也在回环里添乱 */
#define CANTEST_AUTO_RETRANSMIT      0   /* 0 = 关自动重传（否则无限重传霸占总线） */

/* ===================== Loopback 自检（不依赖任何适配器） =====================
 * 1 = CAN 进 CAN_MODE_SILENT_LOOPBACK：控制器自己发、自己收，
 *     帧不出总线口，也完全不需要外部节点应答 ACK。
 *
 * 同时每 CANTEST_LOOPBACK_SELFTEST_MS 自己发一帧 0x200（LED 翻转命令），
 * 让下面这条链路被完整走一遍：
 *
 *   发送 0x200 → 自己的 RX FIFO → 滤波器 → can_test_handle_rx()
 *   → 翻转 PB2 → 回 0x201 ACK → (回环) → 0x201 又进 RX → 关掉回显后被忽略
 *
 * 验通 = 滤波器 / FIFO / 中断或轮询 / LED 写入 / ACK 发送 全部正确。
 * 判读：串口里 tx 和 rx 应该同步每秒 +1，同时板上的 PB2 LED 每秒闪一次。
 *
 * ⚠️ 自检时必须把 CANTEST_ECHO_ENABLE 设成 0，否则回环里
 *    0x201 会被回显成 0x202、再回显成 0x203……无限套娃把邮箱塞爆。
 */
#define CANTEST_LOOPBACK_SELFTEST    1
#define CANTEST_LOOPBACK_SELFTEST_MS 1000U
#define CANTEST_ECHO_ENABLE          0   /* 0 = 收到非 0x200 的帧不回显 */

/* ============================ 串口自诊断 ==================================
 * 1 = 通过 USART1 周期打印 CAN 状态（默认开，调通后可以关掉）
 * 用的是 chassis_debug 同一路 USART1，波特率看 CubeMX 里 USART1 的配置。
 *
 * 打印格式：
 *   CAN 1000000bps | tx=12 rx=0 err=0x00000000 bof=0 started=1
 *
 * 怎么读：
 *   started=0        -> CAN 没启动（大概率卡在 can_test_init 的 Error_Handler）
 *   bitrate 不是 1M  -> 位时序没生效
 *   tx 在涨 rx=0     -> 发送正常但收不到（滤波器/中断/物理层）
 *   bof=1            -> 出现过 Bus-Off，物理层大概率有问题
 *   串口一个字符都没有 -> MCU 根本没跑到主循环（卡在 Error_Handler / 没复位 / 没烧进去）
 */
#define CANTEST_SERIAL_DEBUG         1

/** 自诊断打印周期 (ms) */
#define CANTEST_DEBUG_PERIOD_MS      1000U

/** 自诊断用哪个串口（默认和 chassis_debug 一致用 USART1） */
#define CANTEST_DEBUG_UART           huart1

#endif /* USERLIB_COMM_CAN_TEST_CONFIG_H */
