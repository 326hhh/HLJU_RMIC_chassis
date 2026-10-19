/**
  ******************************************************************************
  * @file    uart_link_codec.h
  * @brief   协议编解码（纯函数，无硬件依赖）
  *
  * 为什么单独抽出来：
  *   帧格式最容易出错的地方是「字节偏移 / 小端序 / CRC 覆盖范围」，
  *   而这些错误在实机上表现为「收不到帧 / 数据乱跳」，极难定位。
  *   抽成纯函数后可以拿 gcc 在 PC 上编译，用 Python 生成的向量做双向交叉校验：
  *       python3 tools/uart_link_test.py vectors > /tmp/vec.h
  *       gcc -o /tmp/codec_test tools/codec_test.c -I/tmp
  *       /tmp/codec_test
  *   字节对不上会在 PC 上先炸，不用等上板。
  ******************************************************************************
  */
#ifndef USERLIB_COMM_UART_LINK_CODEC_H
#define USERLIB_COMM_UART_LINK_CODEC_H

#include "uart_link_config.h"

#include <stdbool.h>
#include <stdint.h>

/** 命令帧（12 B，Jetson -> F407）的字段视图 */
typedef struct
{
    uint8_t cmd;
    uint8_t seq;
    int16_t vx_mm;      /* mm/s，+x 前进 */
    int16_t vy_mm;      /* mm/s，+y 左移 */
    int16_t wz_cs;      /* 0.01 rad/s，逆时针为正 */
    uint8_t flags;
} ulink_cmd_t;

/** 反馈帧（40 B，F407 -> Jetson；v2 含四轮转速）的字段视图 */
typedef struct
{
    uint8_t  seq;               /* 回显最近一次合法命令的 seq */
    int32_t  x_mm, y_mm;        /* 里程计位置 mm */
    int16_t  yaw_cdeg;          /* 里程计 yaw，0.01° */
    int16_t  vx_mm, vy_mm;      /* 运动学正解速度 mm/s */
    int16_t  wz_cs;             /* 正解角速度，0.01 rad/s */
    int16_t  gyro_yaw_cdeg;     /* 陀螺积分 yaw，0.01° */
    int16_t  gyro_wz_cs;        /* 陀螺实测 wz，0.01 rad/s */
    uint16_t status;            /* ULINK_ST_* 位 */
    uint16_t cmd_cnt;           /* 累计合法命令帧数（低 16 位） */
    uint16_t fb_seq;            /* 反馈帧自身序号 */
    int16_t  wheel_rpm[4];      /* 四轮实测转速，0.01 RPM，顺序 FL FR BL BR */
} ulink_fb_t;

/* ================================ CRC ====================================== */

/** CRC-8/ATM：poly=0x07, init=0x00, 不反射（校验值 "123456789" = 0xF4） */
static inline uint8_t ulink_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00u;
    while (len-- > 0u)
    {
        crc ^= *data++;
        for (uint8_t i = 0u; i < 8u; i++)
        {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* ============================== 小端读写 =================================== */

static inline void ulink_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static inline void ulink_put_i16(uint8_t *p, int16_t v)
{
    ulink_put_u16(p, (uint16_t)v);
}

static inline void ulink_put_i32(uint8_t *p, int32_t v)
{
    ulink_put_u16(p, (uint16_t)((uint32_t)v & 0xFFFFu));
    ulink_put_u16(p + 2, (uint16_t)(((uint32_t)v >> 16) & 0xFFFFu));
}

static inline uint16_t ulink_get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline int16_t ulink_get_i16(const uint8_t *p)
{
    return (int16_t)ulink_get_u16(p);
}

static inline int32_t ulink_get_i32(const uint8_t *p)
{
    uint32_t lo = (uint32_t)ulink_get_u16(p);
    uint32_t hi = (uint32_t)ulink_get_u16(p + 2);
    return (int32_t)(lo | (hi << 16));
}

/* ============================== 命令帧 ===================================== */

/** 组命令帧，返回帧长（12）；out 至少 ULINK_CMD_FRAME_LEN 字节 */
static inline uint16_t ulink_build_cmd(uint8_t *out, const ulink_cmd_t *c)
{
    out[0] = ULINK_CMD_HEAD0;
    out[1] = ULINK_CMD_HEAD1;
    out[2] = c->cmd;
    out[3] = c->seq;
    ulink_put_i16(out + 4, c->vx_mm);
    ulink_put_i16(out + 6, c->vy_mm);
    ulink_put_i16(out + 8, c->wz_cs);
    out[10] = c->flags;
    out[11] = ulink_crc8(out, ULINK_CMD_FRAME_LEN - 1u);
    return ULINK_CMD_FRAME_LEN;
}

/** 解命令帧：帧头 + CRC 都对才返回 true */
static inline bool ulink_parse_cmd(const uint8_t *in, ulink_cmd_t *c)
{
    if ((in[0] != ULINK_CMD_HEAD0) || (in[1] != ULINK_CMD_HEAD1))
    {
        return false;
    }
    if (ulink_crc8(in, ULINK_CMD_FRAME_LEN - 1u) != in[ULINK_CMD_FRAME_LEN - 1u])
    {
        return false;
    }
    c->cmd   = in[2];
    c->seq   = in[3];
    c->vx_mm = ulink_get_i16(in + 4);
    c->vy_mm = ulink_get_i16(in + 6);
    c->wz_cs = ulink_get_i16(in + 8);
    c->flags = in[10];
    return true;
}

/* ============================== 反馈帧 ===================================== */

/** 组反馈帧，返回帧长（32）；out 至少 ULINK_FB_FRAME_LEN 字节 */
static inline uint16_t ulink_build_fb(uint8_t *out, const ulink_fb_t *fb)
{
    out[0] = ULINK_FB_HEAD0;
    out[1] = ULINK_FB_HEAD1;
    out[2] = ULINK_FB_TYPE;
    out[3] = fb->seq;

    ulink_put_i32(out + 4, fb->x_mm);
    ulink_put_i32(out + 8, fb->y_mm);
    ulink_put_i16(out + 12, fb->yaw_cdeg);
    ulink_put_i16(out + 14, fb->vx_mm);
    ulink_put_i16(out + 16, fb->vy_mm);
    ulink_put_i16(out + 18, fb->wz_cs);
    ulink_put_i16(out + 20, fb->gyro_yaw_cdeg);
    ulink_put_i16(out + 22, fb->gyro_wz_cs);

    ulink_put_u16(out + 24, fb->status);
    ulink_put_u16(out + 26, fb->cmd_cnt);
    ulink_put_u16(out + 28, fb->fb_seq);
    for (int i = 0; i < 4; i++)
    {
        ulink_put_i16(out + 30 + 2 * i, fb->wheel_rpm[i]);
    }

    out[38] = ulink_crc8(out, ULINK_FB_FRAME_LEN - 2u);   /* 覆盖 0~37 */
    out[39] = ULINK_FB_TAIL;
    return ULINK_FB_FRAME_LEN;
}

/** 解反馈帧：帧头 + 类型 + 帧尾 + CRC 全对才返回 true */
static inline bool ulink_parse_fb(const uint8_t *in, ulink_fb_t *fb)
{
    if ((in[0] != ULINK_FB_HEAD0) || (in[1] != ULINK_FB_HEAD1) ||
        (in[2] != ULINK_FB_TYPE) || (in[ULINK_FB_FRAME_LEN - 1u] != ULINK_FB_TAIL))
    {
        return false;
    }
    if (ulink_crc8(in, ULINK_FB_FRAME_LEN - 2u) != in[ULINK_FB_FRAME_LEN - 2u])
    {
        return false;
    }

    fb->seq            = in[3];
    fb->x_mm           = ulink_get_i32(in + 4);
    fb->y_mm           = ulink_get_i32(in + 8);
    fb->yaw_cdeg       = ulink_get_i16(in + 12);
    fb->vx_mm          = ulink_get_i16(in + 14);
    fb->vy_mm          = ulink_get_i16(in + 16);
    fb->wz_cs          = ulink_get_i16(in + 18);
    fb->gyro_yaw_cdeg  = ulink_get_i16(in + 20);
    fb->gyro_wz_cs     = ulink_get_i16(in + 22);
    fb->status         = ulink_get_u16(in + 24);
    fb->cmd_cnt        = ulink_get_u16(in + 26);
    fb->fb_seq         = ulink_get_u16(in + 28);
    for (int i = 0; i < 4; i++)
    {
        fb->wheel_rpm[i] = ulink_get_i16(in + 30 + 2 * i);
    }
    return true;
}

#endif /* USERLIB_COMM_UART_LINK_CODEC_H */
