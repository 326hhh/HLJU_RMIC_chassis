/**
  ******************************************************************************
  * @file    uart_link.h
  * @brief   底盘控制链路：USART2 二进制协议（Jetson Nano <-> STM32F407）
  *
  * 用法（裸机）：
  *   // main() 的 USER CODE BEGIN 2，必须在 MX_USART2_UART_Init() 之后
  *   uart_link_init();
  *
  *   while (1) {
  *       chassis_poll();       // 1 kHz 控制环
  *       uart_link_poll();     // 收帧解析 + 看门狗 + 反馈帧发送（内部限速）
  *   }
  *
  * 命令帧（Jetson -> F407，12 字节，小端）
  * ┌──────┬────┬─────┬─────┬─────┬──────┬──────┬──────┬───────┬──────┐
  * │ off  │ 0  │  1  │  2  │  3  │ 4..5 │ 6..7 │ 8..9 │  10   │  11  │
  * ├──────┼────┼─────┼─────┼─────┼──────┼──────┼──────┼───────┼──────┤
  * │ 字段 │0xA5│0x5A │ cmd │ seq │  vx  │  vy  │  wz  │ flags │ crc8 │
  * │ 类型 │ u8 │ u8  │ u8  │ u8  │ i16  │ i16  │ i16  │  u8   │  u8  │
  * └──────┴────┴─────┴─────┴─────┴──────┴──────┴──────┴───────┴──────┘
  *   vx / vy : mm/s，+x 前进 / +y 左移（与 chassis 库 §4 一致，不是上位机
  *             ChassisCommand 的那套「右/前」约定！）
  *   wz      : 0.01 rad/s，逆时针为正
  *   crc8    : CRC-8/ATM(poly=0x07, init=0)，覆盖 offset 0~10
  *
  * 反馈帧（F407 -> Jetson，40 字节，小端，100 Hz；v2 在 30..37 追加了四轮实测转速）
  * ┌──────┬─────┬─────┬──────┬──────┬───────┬───────┬───────┬───────┐
  * │ off  │  0  │  1  │  2   │  3   │ 4..7  │ 8..11 │ 12..13│ 14..15│
  * │ 字段 │0x5A │0xA5 │ 0x81 │ seq回显│ x(mm) │ y(mm) │yaw cdeg│vx mm/s│
  * │ 类型 │ u8  │ u8  │  u8  │  u8  │  i32  │  i32  │  i16  │  i16  │
  * ├──────┼──────┬──────┬─────────┬─────────┼───────┬───────┬───────┤
  * │ off  │ 16   │ 17   │  18..19 │ 20..21  │ 22..23│ 24..25│ 26..27│
  * │ 字段 │ vy   │ wz   │ gyro_yaw│ gyro_wz │ status│cmd_cnt│ fb_seq│
  * │ 类型 │ i16  │ i16  │  i16    │  i16    │  u16  │  u16  │  u16  │
  * ├──────┼──────┴──────┴─────────┴─────────┴───────┼───────┼───────┤
  * │ off  │            28..29                     │  30   │  31   │
  * │ 字段 │              fb_seq                   │ crc8  │ 0x0D  │
  * │ 类型 │               u16                     │  u8   │  u8   │
  * └──────┴───────────────────────────────────────┴───────┴───────┘
  *   yaw / gyro_yaw : 0.01°，逆时针为正（±327.67° 内够用，因为已归一化到 ±180°）
  *   wz / gyro_wz   : 0.01 rad/s
  *   status         : 见 uart_link_config.h 的 ULINK_ST_*
  *   cmd_cnt        : 累计收到的合法命令帧数（低 16 位），上位机用它算丢包
  *   fb_seq         : 反馈帧自身的递增序号（也用于上位机算接收率）
  *
  * 看门狗约定（安全第一）：
  *   - 100 ms 没收到合法速度/自旋帧 -> 目标速度强制清零（斜坡减速），
  *     置 ULINK_ST_WATCHDOG；但**不失能驱动、不松开 STBY**（云台/底盘松轴更危险）
  *   - 只有显式 ESTOP 命令才会拉低 STBY
  ******************************************************************************
  */
#ifndef USERLIB_COMM_UART_LINK_H
#define USERLIB_COMM_UART_LINK_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    /* ---- 接收统计 ---- */
    uint32_t rx_frames;     /* 累计解析出的合法命令帧 */
    uint32_t rx_bytes;      /* 累计收到的字节数 */
    uint32_t rx_crc_err;    /* CRC 校验失败次数（丢帧/干扰的直接证据） */
    uint32_t rx_sync_err;   /* 帧头对不上的字节数（重同步次数） */
    uint32_t rx_hw_err;     /* 硬件错误次数（ORE/FE/NE，含自动恢复） */
    uint32_t rx_overflow;   /* 环形缓冲溢出丢字节次数（主循环被卡住才会出现） */

    /* ---- 发送统计 ---- */
    uint32_t tx_frames;     /* 已成功排入发送的反馈帧 */
    uint32_t tx_drop;       /* 因上一帧还没发完而丢掉的反馈帧 */

    /* ---- 命令统计 ---- */
    uint32_t cmd_speed;     /* 速度指令帧计数 */
    uint32_t cmd_unknown;   /* 未知命令字计数 */
    uint32_t wdg_trips;     /* 看门狗触发次数 */

    /* ---- 当前状态 ---- */
    uint8_t  last_seq;      /* 最近一次合法命令帧的 seq */
    uint32_t last_cmd_ms;   /* 最近一次合法控制帧的时刻 (ms) */
    uint8_t  link_ok;       /* 1 = 控制帧未超时 */
    uint8_t  estop;         /* 1 = 急停锁存中 */
} uart_link_state_t;

/**
 * @brief 初始化：启动 USART2 单字节中断接收，复位所有状态
 * @note  依赖 CubeMX 在 NVIC 里勾选 USART2 global interrupt
 *        （否则 stm32f4xx_it.c 里没有 USART2_IRQHandler，中断永远进不来）
 */
void uart_link_init(void);

/**
 * @brief 主循环高频调用：解析接收帧 -> 下发到底盘 -> 看门狗 -> 发反馈帧
 * @note  内部按 ULINK_FB_PERIOD_MS 限速发反馈，收帧解析本身无阻塞。
 *        必须比 1000 Hz 慢不到哪去地调用（主循环里调即可）。
 */
void uart_link_poll(void);

/** 只读统计（调试/上位机诊断用） */
const uart_link_state_t *uart_link_state(void);

#endif /* USERLIB_COMM_UART_LINK_H */
