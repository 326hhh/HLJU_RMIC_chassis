/**
  ******************************************************************************
  * @file    uart_link_config.h
  * @brief   USART2 底盘控制链路的协议常量与可调参数（唯一需要常改的文件）
  *
  * 链路拓扑：
  *   Jetson Nano (USB 转 TTL) --USART2(PD5/PD6, 115200 8N1)--> STM32F407
  *
  * 帧格式（全部小端，定长帧，帧头 + CRC8）：
  *
  *   命令帧（Jetson -> F407），12 字节
  *   ┌─────┬─────┬─────┬─────┬──────┬──────┬──────┬───────┬──────┐
  *   │ off │  0  │  1  │  2  │  3   │ 4..5 │ 6..7 │ 8..9  │ 10,11│
  *   ├─────┼─────┼─────┼─────┼──────┼──────┼──────┼───────┼──────┤
  *   │ 含义│ 0xA5│ 0x5A│ cmd │ seq  │ vx   │ vy   │ wz    │flags,│
  *   │     │     │     │     │      │ i16  │ i16  │ i16   │ crc8 │
  *   └─────┴─────┴─────┴─────┴──────┴──────┴──────┴───────┴──────┘
  *     vx / vy 单位 mm/s（i16），wz 单位 0.01 rad/s（i16）
  *     crc8 = CRC-8/ATM（poly=0x07, init=0x00）覆盖第 0~10 字节
  *
  *   反馈帧（F407 -> Jetson），40 字节（v2：末尾加了四轮转速）
  *   ┌─────┬─────┬─────┬─────┬───────┬───────┬───────┬──────┬──────┐
  *   │ off │  0  │  1  │  2  │  3    │  4..7 │  8..11│ 12..13│ ... │
  *   ├─────┼─────┼─────┼─────┼───────┼───────┼───────┼──────┼──────┤
  *   │ 含义│ 0x5A│ 0xA5│0x81 │seq 回显│ x mm  │ y mm  │ yaw  │ ... │
  *   └─────┴─────┴─────┴─────┴───────┴───────┴───────┴──────┴──────┘
  *   30..37 = 四轮实测转速 (i16, 0.01 RPM)：FL FR BL BR；38 = crc8；39 = 0x0D
  *   完整字段表见 uart_link.h 顶部注释。
  ******************************************************************************
  */
#ifndef USERLIB_COMM_UART_LINK_CONFIG_H
#define USERLIB_COMM_UART_LINK_CONFIG_H

/* ============================ 帧结构常量 =================================== */
/** 命令帧帧头 */
#define ULINK_CMD_HEAD0            0xA5u
#define ULINK_CMD_HEAD1            0x5Au
/** 反馈帧帧头（与命令帧相反，一眼能认出方向） */
#define ULINK_FB_HEAD0             0x5Au
#define ULINK_FB_HEAD1             0xA5u

#define ULINK_CMD_FRAME_LEN        12u
#define ULINK_FB_FRAME_LEN         40u
#define ULINK_FB_TYPE              0x81u   /* 反馈帧类型码 */
#define ULINK_FB_TAIL              0x0Du   /* 反馈帧尾（CRC 之后那一字节） */

/* ============================== 命令字 ===================================== */
/** 速度指令：vx/vy/wz 有效；flags 的 EN 位决定是否真正驱动 */
#define ULINK_CMD_SPEED            0x01u
/** 急停：清零指令速度 + 拉低 STBY（电机自由）；收到带 EN 的下一帧恢复 */
#define ULINK_CMD_ESTOP            0x02u
/** 小陀螺：wz 字段（0.01 rad/s）当自旋速率；发 SPEED 退出 */
#define ULINK_CMD_SPIN             0x03u
/** 清零：里程计 (x,y,yaw) + 陀螺积分 yaw */
#define ULINK_CMD_CLEAR            0x04u

/* ============================== flags 位 =================================== */
/** bit0 = 允许驱动（0 = 目标速度清零、驱动保持但不动；1 = 正常接收速度指令） */
#define ULINK_FLAG_EN              0x01u

/* ========================= 反馈帧 status 位 ================================ */
#define ULINK_ST_IMU_ONLINE        0x0001u  /* IMU 在线且配置成功 */
#define ULINK_ST_LINK_OK           0x0002u  /* 最近 ULINK_CMD_TIMEOUT_MS 内收到过合法控制帧 */
#define ULINK_ST_WATCHDOG          0x0004u  /* 看门狗已触发（控制帧超时） */
#define ULINK_ST_ESTOP             0x0008u  /* 急停锁存中 */
#define ULINK_ST_DRIVER_ON         0x0010u  /* TB6612 STBY 高（输出使能） */
#define ULINK_ST_SPIN              0x0020u  /* 小陀螺模式 */
#define ULINK_ST_GYRO_ON           0x0040u  /* 陀螺外环保留（非 OFF 模式） */
#define ULINK_ST_IMU_CAL           0x0080u  /* 陀螺零偏标定有效 */

/* ============================== 时序参数 =================================== */
/** 反馈帧周期 (ms)：100 Hz 下发，115200 下占约 2.8 ms/帧 */
#define ULINK_FB_PERIOD_MS         10u
/** 控制帧超时 (ms)：超过这么久没收到合法速度/自旋帧 -> 底盘自动停车并置看门狗位。
 *  Jetson 端 100 Hz 发帧，连续丢 10 帧才判超时，兼顾丢包容忍与响应速度。 */
#define ULINK_CMD_TIMEOUT_MS       100u
/** 链路丢失 (ms)：超过这么久连一帧合法命令都没有 -> 置 LINK_LOST（由上位机显示）。
 * 注意：停车已经在 ULINK_CMD_TIMEOUT_MS 处完成，这里只是状态位，不重复动作。 */
#define ULINK_CMD_LOST_MS          500u

/** RX 环形缓冲大小（字节，必须是 2 的幂以便用 & 取模） */
#define ULINK_RX_RING_SIZE         256u

/* ============================== 单位换算 =================================== */
/** 命令帧 vx/vy：1 LSB = 1 mm/s = 0.001 m/s（底盘库用 m/s） */
#define ULINK_MM_S_TO_M_S          0.001f
/** 命令帧 wz：1 LSB = 0.01 rad/s */
#define ULINK_WZ_UNIT_RAD_S        0.01f

#endif /* USERLIB_COMM_UART_LINK_CONFIG_H */
