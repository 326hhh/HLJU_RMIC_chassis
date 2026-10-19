/**
  ******************************************************************************
  * @file    codec_test.c
  * @brief   协议编解码的 PC 端交叉校验（不需要任何硬件）
  *
  * 目的：证明「C 侧编解码」与「Python 参考实现」逐字节一致。
  * 帧格式写错（字节偏移、大小端、CRC 覆盖范围）在实机上只会表现为
  * 「收不到 / 数据乱跳」，很难定位；这里在 PC 上先把它按死。
  *
  * 用法：
  *   python3 tools/uart_link_test.py vectors > /tmp/ulink_vectors.h
  *   gcc -std=c11 -Wall -Wextra -IUserLib/Comm -I/tmp \
  *       -o /tmp/codec_test tools/codec_test.c && /tmp/codec_test
  ******************************************************************************
  */
#include "uart_link_codec.h"
#include "ulink_vectors.h"      /* 由 Python 生成 */

#include <stdio.h>
#include <string.h>

static int g_fail;

static void check(const char *name, int ok)
{
    if (!ok)
    {
        g_fail++;
        printf("  [FAIL] %s\n", name);
    }
}

static void dump(const char *tag, const uint8_t *a, const uint8_t *b, unsigned n)
{
    unsigned i;
    printf("  %s\n      C : ", tag);
    for (i = 0; i < n; i++) printf("%02X ", a[i]);
    printf("\n      PY: ");
    for (i = 0; i < n; i++) printf("%02X ", b[i]);
    printf("\n");
}

int main(void)
{
    unsigned i;

    printf("[1] 命令帧：C 编码 == Python 编码\n");
    for (i = 0; i < N_CMD_CASES; i++)
    {
        uint8_t frame[ULINK_CMD_FRAME_LEN];
        unsigned len = ulink_build_cmd(frame, &CMD_CASES[i]);
        int ok = (len == ULINK_CMD_FRAME_LEN) &&
                 (memcmp(frame, CMD_FRAMES[i], ULINK_CMD_FRAME_LEN) == 0);
        if (!ok) dump("cmd frame mismatch", frame, CMD_FRAMES[i], ULINK_CMD_FRAME_LEN);
        check("cmd encode", ok);
    }

    printf("[2] 命令帧：C 解码 Python 帧 == 字段\n");
    for (i = 0; i < N_CMD_CASES; i++)
    {
        ulink_cmd_t c;
        int ok = ulink_parse_cmd(CMD_FRAMES[i], &c) &&
                 (c.cmd == CMD_CASES[i].cmd) &&
                 (c.seq == CMD_CASES[i].seq) &&
                 (c.vx_mm == CMD_CASES[i].vx_mm) &&
                 (c.vy_mm == CMD_CASES[i].vy_mm) &&
                 (c.wz_cs == CMD_CASES[i].wz_cs) &&
                 (c.flags == CMD_CASES[i].flags);
        check("cmd decode", ok);
    }

    printf("[3] 反馈帧：C 编码 == Python 编码\n");
    for (i = 0; i < N_FB_CASES; i++)
    {
        uint8_t frame[ULINK_FB_FRAME_LEN];
        unsigned len = ulink_build_fb(frame, &FB_CASES[i]);
        int ok = (len == ULINK_FB_FRAME_LEN) &&
                 (memcmp(frame, FB_FRAMES[i], ULINK_FB_FRAME_LEN) == 0);
        if (!ok) dump("fb frame mismatch", frame, FB_FRAMES[i], ULINK_FB_FRAME_LEN);
        check("fb encode", ok);
    }

    printf("[4] 反馈帧：C 解码 Python 帧 == 字段\n");
    for (i = 0; i < N_FB_CASES; i++)
    {
        ulink_fb_t f;
        int ok = ulink_parse_fb(FB_FRAMES[i], &f) &&
                 (f.seq == FB_CASES[i].seq) &&
                 (f.x_mm == FB_CASES[i].x_mm) &&
                 (f.y_mm == FB_CASES[i].y_mm) &&
                 (f.yaw_cdeg == FB_CASES[i].yaw_cdeg) &&
                 (f.vx_mm == FB_CASES[i].vx_mm) &&
                 (f.vy_mm == FB_CASES[i].vy_mm) &&
                 (f.wz_cs == FB_CASES[i].wz_cs) &&
                 (f.gyro_yaw_cdeg == FB_CASES[i].gyro_yaw_cdeg) &&
                 (f.gyro_wz_cs == FB_CASES[i].gyro_wz_cs) &&
                 (f.status == FB_CASES[i].status) &&
                 (f.cmd_cnt == FB_CASES[i].cmd_cnt) &&
                 (f.fb_seq == FB_CASES[i].fb_seq);
        check("fb decode", ok);
    }

    printf("[5] 抗干扰：任一字节被改都必须解不出来\n");
    {
        uint8_t bad[ULINK_FB_FRAME_LEN];
        unsigned off;
        int all_rejected = 1;
        for (off = 0; off < ULINK_FB_FRAME_LEN; off++)
        {
            ulink_fb_t f;
            memcpy(bad, FB_FRAMES[1], sizeof(bad));
            bad[off] ^= 0xFFu;
            if (ulink_parse_fb(bad, &f))
            {
                printf("  [FAIL] 改了第 %u 字节仍被接受\n", off);
                all_rejected = 0;
            }
        }
        check("fb 单字节破坏全部拒绝", all_rejected);
    }
    {
        uint8_t bad[ULINK_CMD_FRAME_LEN];
        unsigned off;
        int all_rejected = 1;
        for (off = 2; off < ULINK_CMD_FRAME_LEN; off++)   /* 帧头之外的字节 */
        {
            ulink_cmd_t c;
            memcpy(bad, CMD_FRAMES[1], sizeof(bad));
            bad[off] ^= 0xFFu;
            if (ulink_parse_cmd(bad, &c))
            {
                printf("  [FAIL] 改了命令帧第 %u 字节仍被接受\n", off);
                all_rejected = 0;
            }
        }
        check("cmd 单字节破坏全部拒绝", all_rejected);
    }

    printf("\n结果：%s（命令 %u 组 / 反馈 %u 组）\n",
           g_fail ? "存在失败 [FAIL]" : "全部通过 [OK]", N_CMD_CASES, N_FB_CASES);
    return g_fail ? 1 : 0;
}
