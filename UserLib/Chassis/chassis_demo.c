/**
  ******************************************************************************
  * @file    chassis_demo.c
  * @brief   底盘演示状态机实现（上电 1.5s 后自动开始循环）
  ******************************************************************************
  */
#include "chassis_demo.h"
#include "chassis.h"
#include "chassis_math.h"

#include <math.h>

typedef enum
{
    DEMO_IDLE = 0,          /* 上电等待，随后进入前进 */
    DEMO_FORWARD,           /* 前进 0.5 m/s x 2s */
    DEMO_STRAFE_LEFT,       /* 左移 0.4 m/s x 2s */
    DEMO_ROTATE_CCW,        /* 原地逆时针转到 90°（里程计闭环） */
    DEMO_BACKWARD,          /* 后退 0.5 m/s x 2s */
    DEMO_STRAFE_RIGHT,      /* 右移 0.4 m/s x 2s */
    DEMO_ROTATE_CW,         /* 原地顺时针转回 90°（里程计闭环） */
    DEMO_HOLD,              /* 静止 3s 后重来 */
} demo_phase_t;

#define DEMO_STARTUP_MS    1500u
#define DEMO_MOVE_MS       2000u
#define DEMO_HOLD_MS       3000u
#define DEMO_ROTATE_RAD_S  1.57f
#define DEMO_YAW_TARGET    (CHASSIS_PI / 2.0f)     /* 90° */

static demo_phase_t s_phase = DEMO_IDLE;
static uint32_t s_phase_start;
static uint32_t s_last_tick;
static float s_yaw_entry;

static void demo_move_to(uint32_t now, demo_phase_t next)
{
    s_phase_start = now;
    s_phase = next;
}

void chassis_demo_init(void)
{
    s_phase = DEMO_IDLE;
    s_phase_start = 0u;
    s_last_tick = chassis_tick_ms();
    s_yaw_entry = 0.0f;
}

void chassis_demo_poll(void)
{
    uint32_t now = chassis_tick_ms();
    if (now == s_last_tick)
    {
        return;             /* 与 chassis_poll() 同节拍执行 */
    }
    s_last_tick = now;

    const chassis_state_t *st = chassis_state();
    float yaw_delta;

    switch (s_phase)
    {
    case DEMO_IDLE:
        chassis_stop();
        if (now - s_phase_start >= DEMO_STARTUP_MS)
        {
            demo_move_to(now, DEMO_FORWARD);
        }
        break;

    case DEMO_FORWARD:
        chassis_set_velocity(0.5f, 0.0f, 0.0f);
        if (now - s_phase_start >= DEMO_MOVE_MS)
        {
            demo_move_to(now, DEMO_STRAFE_LEFT);
        }
        break;

    case DEMO_STRAFE_LEFT:
        chassis_set_velocity(0.0f, 0.4f, 0.0f);
        if (now - s_phase_start >= DEMO_MOVE_MS)
        {
            s_yaw_entry = st->yaw;
            demo_move_to(now, DEMO_ROTATE_CCW);
        }
        break;

    case DEMO_ROTATE_CCW:
        chassis_set_velocity(0.0f, 0.0f, DEMO_ROTATE_RAD_S);
        yaw_delta = chassis_wrap_pi(st->yaw - s_yaw_entry);
        if (fabsf(yaw_delta) >= DEMO_YAW_TARGET)
        {
            demo_move_to(now, DEMO_BACKWARD);
        }
        break;

    case DEMO_BACKWARD:
        chassis_set_velocity(-0.5f, 0.0f, 0.0f);
        if (now - s_phase_start >= DEMO_MOVE_MS)
        {
            demo_move_to(now, DEMO_STRAFE_RIGHT);
        }
        break;

    case DEMO_STRAFE_RIGHT:
        chassis_set_velocity(0.0f, -0.4f, 0.0f);
        if (now - s_phase_start >= DEMO_MOVE_MS)
        {
            s_yaw_entry = st->yaw;
            demo_move_to(now, DEMO_ROTATE_CW);
        }
        break;

    case DEMO_ROTATE_CW:
        chassis_set_velocity(0.0f, 0.0f, -DEMO_ROTATE_RAD_S);
        yaw_delta = chassis_wrap_pi(st->yaw - s_yaw_entry);
        if (fabsf(yaw_delta) >= DEMO_YAW_TARGET)
        {
            demo_move_to(now, DEMO_HOLD);
        }
        break;

    case DEMO_HOLD:
    default:
        chassis_stop();
        if (now - s_phase_start >= DEMO_HOLD_MS)
        {
            demo_move_to(now, DEMO_FORWARD);
        }
        break;
    }
}
