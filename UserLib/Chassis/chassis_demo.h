/**
  ******************************************************************************
  * @file    chassis_demo.h
  * @brief   底盘演示：非阻塞状态机，依次执行前进/左移/左转90°/后退/右移/右转90°
  *
  * 用途：验证全向运动、运动学符号与里程计；实际应用可整文件删除。
  ******************************************************************************
  */
#ifndef USERLIB_CHASSIS_CHASSIS_DEMO_H
#define USERLIB_CHASSIS_CHASSIS_DEMO_H

void chassis_demo_init(void);
void chassis_demo_poll(void);

#endif /* USERLIB_CHASSIS_CHASSIS_DEMO_H */
