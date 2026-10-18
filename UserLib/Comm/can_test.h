/**
  ******************************************************************************
  * @file    can_test.h
  * @brief   CAN1 通信测试模块：验证 STM32(F407) <-> Jetson 的 CAN 链路
  *
  * 使用方式（裸机，接在 main.c 的 USER CODE 区）：
  *   // USER CODE BEGIN Includes
  *   #include "can_test.h"
  *
  *   // USER CODE BEGIN 2   （必须在 MX_CAN1_Init() 之后）
  *   can_test_init();
  *
  *   while (1) {
  *       can_test_poll();    // 发心跳 + 收帧回显
  *       ... 其他应用逻辑 ...
  *   }
  *
  * 行为：
  *   ① 强制把 CAN1 配成 1Mbps（不依赖 CubeMX 的位时序设置）
  *   ② 配【全通过滤波器】—— 这是 F407 上收不到帧的头号原因
  *   ③ 每 500ms 发一帧心跳（ID 0x7F0，数据带序号）
  *   ④ 收到任何帧就原样回一帧，ID 加 1
  *      → Jetson 发 123#A1B2C3D4 应该收到 124#A1B2C3D4
  *
  * Jetson 侧验证：
  *   candump can0                      # 应每 0.5s 看到 7F0 心跳
  *   cansend can0 123#A1B2C3D4         # 应收到 124 回显
  ******************************************************************************
  */
#ifndef USERLIB_COMM_CAN_TEST_H
#define USERLIB_COMM_CAN_TEST_H

#include <stdbool.h>
#include <stdint.h>

/** 运行状态与统计（只读快照） */
typedef struct
{
    bool     started;      /* CAN 是否已成功启动 */
    uint32_t tx_count;     /* 累计发送帧数 */
    uint32_t rx_count;     /* 累计接收帧数 */
    uint32_t last_error;   /* 最近一次 HAL CAN 错误码（HAL_CAN_ERROR_*，0 = 无错） */
    bool     bus_off_seen; /* 是否出现过 Bus-Off（真 = 物理层大概率有问题） */
    uint32_t bitrate_hz;   /* 实际生效的位速率 */
} can_test_state_t;

/** 初始化：强制配位时序 -> 配全通过滤波器 -> 启动 CAN
 *  @note 必须在 MX_CAN1_Init() 之后调用（那里才开了 CAN1 时钟和 GPIO） */
void can_test_init(void);

/** 主循环周期调用：发心跳、收帧并回显
 *  @note 轮询模式（CANTEST_USE_RX_INTERRUPT = 0）下，调用得越勤越好；
 *        本工程主循环是裸机 while(1)，直接放进去即可。 */
void can_test_poll(void);

/** 手动发一帧标准帧
 *  @param std_id 标准帧 ID（11 位，0x000~0x7FF）
 *  @param data   数据指针，可为 NULL
 *  @param len    数据长度 0~8，超过 8 会被截断
 *  @return true 表示已成功写入发送邮箱 */
bool can_test_send(uint16_t std_id, const uint8_t *data, uint8_t len);

/** 取运行状态快照 */
can_test_state_t can_test_get_state(void);

/** 是否已成功启动 */
bool can_test_is_started(void);

/** 最近一次 HAL CAN 错误码（0 = 无错）。
 *  常见值：HAL_CAN_ERROR_BOF(总线关闭) / HAL_CAN_ERROR_ACK(无节点应答) /
 *          HAL_CAN_ERROR_BIT / HAL_CAN_ERROR_STF / HAL_CAN_ERROR_CRC */
uint32_t can_test_last_error(void);

#endif /* USERLIB_COMM_CAN_TEST_H */
