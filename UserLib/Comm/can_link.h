/**
  ******************************************************************************
  * @file    can_link.h
  * @brief   底盘控制链路：CAN 版本（Jetson Nano <-> STM32F407）
  *
  *  用法（裸机）：
  *     // main() 的 USER CODE BEGIN 2，必须在 MX_CAN1_Init() 之后
  *     #if CAN_LINK_ENABLE
  *       can_link_init();
  *     #endif
  *
  *     while (1) {
  *         chassis_poll();          // 1 kHz 控制环
  *         uart_link_poll();        // 串口版（可留作备份）
  *     #if CAN_LINK_ENABLE
  *         can_link_poll();         // CAN：收帧 + 看门狗 + 100 Hz 反馈
  *     #endif
  *     }
  *
  *  与串口版的关系：
  *     协议字段、CRC、命令落地、看门狗**全部复用 uart_link**，
  *     本模块只负责「CAN 帧 <-> 12B/40B 协议帧」的搬运与拆分重组。
  *
  *  帧表（全部 11-bit 标准帧，见 can_link_config.h）：
  *     Jetson → F407
  *       0x000  DLC=0   广播急停（最高优先级，可打断一切）
  *       0x020  DLC=8   [cmd, seq, vx(i16), vy(i16), wz(i16)]  100 Hz
  *       0x021  DLC>=1  [flags]  （EN 使能位）  10 Hz 或变化即发
  *     F407 → Jetson
  *       0x110~0x114  DLC=8   40B 反馈帧按字节切 5 段，100 Hz
  *                             接收端按 0x110→0x114 拼接，CRC 失败丢弃整组
  ******************************************************************************
  */
#ifndef USERLIB_COMM_CAN_LINK_H
#define USERLIB_COMM_CAN_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "can_link_config.h"   /* main.c 只 include 本头文件即可拿到 CAN_LINK_ENABLE */

/** 运行统计（调试 / 诊断用，只读） */
typedef struct
{
    /* ---- 接收 ---- */
    uint32_t rx_frames;       /* 合法的 0x020 / 0x021 / 0x000 帧数 */
    uint32_t rx_ignored;      /* 收到但不关心（含 ZDT 扩展帧）的帧数 */
    uint32_t rx_short;        /* DLC 不足被丢弃的帧数 */

    /* ---- 发送 ---- */
    uint32_t tx_frames;       /* 成功写入发送邮箱的帧数 */
    uint32_t tx_queue_full;   /* 队列满而丢帧的次数（总线堵塞的直接证据） */
    uint32_t tx_add_fail;     /* HAL_CAN_AddTxMessage 失败的次数 */

    /* ---- 状态 ---- */
    uint16_t last_flags;      /* 最近一次 0x021 收到的 flags */
    uint32_t bitrate_hz;      /* 从 CAN_BTR 寄存器反读的实际位速率（不信配置信寄存器） */
    uint8_t  started;         /* CAN 是否已启动 */
    uint8_t  can_err;         /* 1 = TEC/REC 超阈值，status 的 CAN_ERR 位来源 */
} can_link_state_t;

/**
 * @brief 初始化：强制 1 Mbps 位时序 -> 全通过滤波器 -> 启动 CAN1 -> 复位统计
 * @note  必须在 MX_CAN1_Init() 之后调用（那里才开了 CAN1 时钟和 GPIO）
 */
void can_link_init(void);

/**
 * @brief 主循环高频调用：收帧下发 -> 看门狗 -> 每 10ms 组一组反馈 -> 推 TX 队列
 * @note  TX 队列在每次调用里"能发就发"，所以调用越勤，组内延迟越低（目标 ≤2ms）
 */
void can_link_poll(void);

/** 只读统计 */
const can_link_state_t *can_link_state(void);
/** 最近一次 0x021 收到的 flags（诊断用：bit0 = EN 允许驱动） */
uint8_t can_link_flags(void);

#endif /* USERLIB_COMM_CAN_LINK_H */
