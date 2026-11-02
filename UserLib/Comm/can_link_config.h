/**
  ******************************************************************************
  * @file    can_link_config.h
  * @brief   CAN 控制链路（Jetson <-> F407）的协议常量与可调参数
  *
  *  与串口版的关系：
  *    本模块**不重新定义协议**，而是把 `uart_link` 的 12B 命令帧 / 40B 反馈帧
  *    原样映射到 11-bit 标准帧上——命令 1 帧（0x020）+ 1 帧标志（0x021），
  *    反馈 40 字节按 8 字节切 5 帧（0x110~0x114）。
  *    字段语义、CRC 算法、看门狗逻辑全部复用 `uart_link`，见
  *    `docs/CAN_CONTROL_PROTOCOL.md` v0.3。
  *
  *  为什么按字节切而不是重新设计精简帧：
  *    编解码 / CRC / 校验全部复用，联调时 `candump` 看到的字节和串口完全对应，
  *    零学习成本；代价是 40B 要 5 帧，接收端必须做组完整性校验（见 §3.2）。
  ******************************************************************************
  */
#ifndef USERLIB_COMM_CAN_LINK_CONFIG_H
#define USERLIB_COMM_CAN_LINK_CONFIG_H

#include "stm32f4xx_hal.h"

/* ============================== 启用开关 ==================================
 * ⚠️ 与 `CANTEST_ENABLE` **互斥**：两者都要初始化 CAN1，
 *    同时启用会在编译期报错拦下（见 can_link.c 顶部）。
 */
#define CAN_LINK_ENABLE              1   /* 1 = 启用 CAN 控制链路（联调期）/ 0 = 关（整定期） */

/* ============================== 位时序 ====================================
 * CAN1 挂 APB1 = 42 MHz。14 TQ（1 + 11 + 2），采样点 85.7%，1 Mbps。
 * 与 `can_test` 完全一致；换速率只改 CAN_LINK_PRESCALER（42MHz/14 = 3 MHz，
 * 故 位速率 = 3 Mbps / Prescaler：1M→3、500k→6、250k→12）。
 */
#define CAN_LINK_APB1_HZ             42000000UL
#define CAN_LINK_PRESCALER           3U
#define CAN_LINK_BS1                 CAN_BS1_11TQ
#define CAN_LINK_BS2                 CAN_BS2_2TQ
#define CAN_LINK_SJW                 CAN_SJW_1TQ

/* ============================== 帧 ID =====================================
 * 全部 11-bit 标准帧；ZDT 云台用 29-bit 扩展帧，在同一总线上天然隔离。
 */
#define CANLINK_ID_ESTOP             0x000U   /* Jetson → F407，广播急停（ID 最小 = 仲裁最高优先级） */
#define CANLINK_ID_CMD               0x020U   /* Jetson → F407，速度/模式命令 */
#define CANLINK_ID_FLAGS             0x021U   /* Jetson → F407，flags（EN 使能位） */
#define CANLINK_ID_FB_BASE           0x110U   /* F407 → Jetson，反馈第 0 帧；+1..+4 共 5 帧 */

/* 反馈帧拆分：40 字节 / 8 字节每帧 = 5 帧 */
#define CANLINK_FB_SPLIT             5U
#define CANLINK_FB_SPLIT_BYTES       8U

/* ============================== 时序参数 =================================== */
/** 反馈帧周期 (ms)：100 Hz。5 帧在 10 ms 内连续发完（组内 ≤2 ms） */
#define CANLINK_FB_PERIOD_MS         10U

/** TX 队列深度：至少能装下一整组 5 帧，留余量吸收突发 */
#define CANLINK_TX_QUEUE_SIZE        20U

/** CAN 错误标志阈值：TEC/REC 超过它就置 ULINK_ST_CAN_ERR（96 = error-passive 预警线） */
#define CANLINK_ERR_COUNT_THRESHOLD  96U

/* ====================== 诊断帧（排查用，正式版默认关） ======================
 * 1 = 每个反馈周期额外发两帧（0x115 / 0x116），把下位机内部状态暴露出来。
 *    专门用来定位「轮速周期性掉速」这类只有固件内部才看得见的问题：
 *      cmd_vx 掉了              -> 有人把速度指令清零了
 *      cmd_vx 正常但 duty 掉了  -> 控制/输出链路被动过
 *      stop_cnt 在涨            -> chassis_stop() 被调用（看门狗 / EN=0 / 急停）
 * 帧格式（全部小端）：
 *   0x115: [duty_FL, duty_FR, duty_BL, duty_BR]，各 i16，千分比
 *   0x116: [cmd_vx(i16 mm/s), wz_eff(i16 mrad/s), stop_cnt(u16), 位: b0=driver b1=imu]
 *   0x117: [uart_rx_frames(u16), uart_crc_err(u16), wdg_trips(u16), uart_overflow(u8), can_rx_short(u8)]
 *          —— 用来判定：UART 链路是否在偷收帧 / 看门狗是否触发 / 噪声计数
 * ⚠️ 打开后总线每秒多 200 帧，正式跑车请保持 0。 */
#define CANLINK_DIAG_FB_ENABLE       0
#define CANLINK_ID_DIAG_DUTY         0x115U
#define CANLINK_ID_DIAG_STATE        0x116U
#define CANLINK_ID_DIAG_STATE2       0x117U
#endif /* USERLIB_COMM_CAN_LINK_CONFIG_H */
