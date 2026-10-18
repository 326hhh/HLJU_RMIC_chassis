#!/usr/bin/env python3
"""陀螺自旋实验（chassis_tune 实验 5）数据解析

用法:
    python3 tools/spin_analysis.py <raw_log> [--startup-profile]

raw_log 是串口原始抓取，含若干段：
    SET idx=N spin ref=RRR mrad_s imu=1
    STEP_BEGIN n=2000 ch=8 dt_us=1000
    <n*16 字节 int16 小端>
    STEP_END cksum=..

通道（小端 int16）:
    ch0 参考 wz (mrad/s)      ch1 陀螺实测 wz (mrad/s)
    ch2 里程计 wz (mrad/s)    ch3 陀螺积分 yaw (0.01rad)
    ch4 里程计 yaw (0.01rad)  ch5 FL 实测轮速 (RPM)
    ch6 BR 实测轮速 (RPM)     ch7 FL 占空比 (‰)

输出每段的：稳态误差、纹波、超调、进入 ±5% 的上升时间、陀螺/里程计 yaw 的速度差。
"""
import sys

import numpy as np

CH = 8
SAMPLES_PER_SEG = 2000  # 与 chassis_config.h 的 SPIN_SEG_TICKS 无关，从 STEP_BEGIN 读
STEADY_TAIL = 0.5  # 每段最后 50% 当稳态


def parse(path):
    raw = open(path, "rb").read()
    segs = []
    pos = 0
    while True:
        i = raw.find(b"SET idx=", pos)
        j = raw.find(b"STEP_BEGIN", i)
        if i < 0 or j < 0:
            break
        tag = raw[i:raw.find(b"\n", i)].decode(errors="replace").strip()
        nl = raw.find(b"\n", j)
        n = int(raw[j:nl].split(b"n=")[1].split()[0])
        nbytes = n * CH * 2
        blk = raw[nl + 1: nl + 1 + nbytes]
        if len(blk) != nbytes:
            print(f"!! {tag} 数据不完整 ({len(blk)}/{nbytes})")
            break
        a = np.frombuffer(blk, dtype="<i2").astype(np.float64).reshape(n, CH)
        segs.append((tag, a))
        pos = nl + 1 + nbytes
    return segs


def one(tag, a):
    t = np.arange(len(a)) * 0.001
    wz_ref = a[:, 0] / 1000.0
    wz_gyro = a[:, 1] / 1000.0
    wz_odom = a[:, 2] / 1000.0
    yaw_gyro = a[:, 3] / 100.0
    yaw_odom = a[:, 4] / 100.0
    rpm_fl = a[:, 5]
    rpm_br = a[:, 6]
    duty_fl = a[:, 7]

    target = float(np.median(wz_ref))
    print(f"\n=== {tag}")
    print(f"    目标 {target:+.3f} rad/s ({np.degrees(target):+.1f} deg/s)")

    if abs(target) < 0.05:
        half = len(a) // 2
        still = slice(half, len(a))
        print(f"    静止段: 陀螺均值 {wz_gyro[still].mean():+.4f} "
              f"标准差 {wz_gyro[still].std():.4f} rad/s | "
              f"里程计均值 {wz_odom[still].mean():+.4f}")
        dyaw = yaw_gyro[-1] - yaw_gyro[half]
        print(f"    静止 {0.5 * (t[-1] - t[half]):.2f}s 内陀螺 yaw 漂移 {dyaw:+.3f} rad "
              f"({np.degrees(dyaw):+.2f} deg)，折算 {np.degrees(dyaw) / (t[-1] - t[half]):+.3f} deg/s")
        return

    half = int(len(a) * STEADY_TAIL)
    ss = slice(half, len(a))
    err = wz_gyro[ss] - target
    print(f"    稳态误差 {err.mean():+.4f} rad/s ({100 * err.mean() / target:+.2f}%) | "
          f"纹波 std {err.std():.4f} / pk-pk {np.ptp(wz_gyro[ss]):.4f} rad/s")
    peak = wz_gyro.max() if target > 0 else wz_gyro.min()
    print(f"    峰值 {peak:+.3f} rad/s (超调 {100 * (peak - target) / target:+.1f}%)")
    band = 0.05 * abs(target)
    hit = np.where(np.abs(wz_gyro - target) < band)[0]
    if len(hit):
        print(f"    进入 ±5% 带: {t[hit[0]] * 1000:.0f} ms")
    print(f"    轮速 FL {rpm_fl[ss].mean():.1f} RPM / BR {rpm_br[ss].mean():.1f} RPM | "
          f"FL 占空比 {duty_fl[ss].mean():.0f}‰")
    d_odom = yaw_odom[-1] - yaw_odom[0]
    d_gyro = yaw_gyro[-1] - yaw_gyro[0]
    print(f"    本段总转角: 陀螺 {d_gyro:+.3f} rad / 里程计 {d_odom:+.3f} rad "
          f"(里程计/陀螺比 {(d_odom / d_gyro if abs(d_gyro) > 1e-3 else float('nan')):.3f})")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    segs = parse(sys.argv[1])
    if not segs:
        print("没解析到数据块（是不是没开 CHASSIS_TUNE_ENABLE / EXPERIMENT=5？）")
        sys.exit(1)
    print(f"共 {len(segs)} 段")
    for tag, a in segs:
        one(tag, a)


if __name__ == "__main__":
    main()
