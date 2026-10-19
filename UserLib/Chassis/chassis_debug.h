/**
  ******************************************************************************
  * @file    chassis_debug.h
  * @brief   底盘调试打印：通过 USART1 周期性输出四轮状态与里程计
  *
  * 仅当 chassis_config.h 中 CHASSIS_DEBUG_ENABLE=1 时生效。
  * 波特率 115200-8-N-1，PA9(TX)/PA10(RX)。
  ******************************************************************************
  */
#ifndef USERLIB_CHASSIS_CHASSIS_DEBUG_H
#define USERLIB_CHASSIS_CHASSIS_DEBUG_H

#include "main.h"   /* UART_HandleTypeDef */

/** 主循环中高频调用，内部按 250ms 节拍打印（非阻塞发送） */
void chassis_debug_poll(void);

/** 在 HAL_UART_TxCpltCallback 里转发调用：清 USART1 调试发送忙标志 */
void chassis_debug_on_tx_cplt(UART_HandleTypeDef *huart);

#endif /* USERLIB_CHASSIS_CHASSIS_DEBUG_H */
