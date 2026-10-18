#!/usr/bin/env python3
"""阶跃响应数据解析 + 指标计算（支持一次抓取里多个参数组）

用法:
    python3 tools/step_analysis.py <raw_log> [--png out.png]

raw_log 是串口原始抓取，包含若干段：
    SET idx=.. kp=.. ki=.. rpm=..
    STEP_BEGIN n=3000 ch=12 dt_us=1000
    <n*24 字节 int16 小端>
    STEP_END cksum=..
"""
import sys
import numpy as np

WHEELS = ["FL", "FR", "BL", "BR"]


def parse(path):
    raw = open(path, "rb").read()
    sets = []
    pos = 0
    while True:
        i = raw.find(b"SET idx=", pos)
        j = raw.find(b"STEP_BEGIN", i)
        if i < 0 or j < 0:
            break
        tag = raw[i:raw.find(b"\n", i)].decode(errors="replace").strip()
        nl = raw.find(b"\n", j)
        n = int(raw[j:nl].split(b"n=")[1].split()[0])
        blk = raw[nl + 1: nl + 1 + n * 24]
        if len(blk) != n * 24:
            print(f"!! {tag} 数据不完整 ({len(blk)}/{n*24})")
            break
        a = np.frombuffer(blk, dtype="<i2").astype(np.float64).reshape(n, 12)
        sets.append((tag, a))
        pos = nl + 1 + n * 24
    return sets


def smooth(x, k=5):
    return np.convolve(x, np.ones(k) / k, mode="same")


def segs_of(ref):
    """把 ref 切成分段 [(start, end, value)]，忽略值为 0 的段"""
    out = []
    n = len(ref)
    s = 0
    while s < n:
        e = s
        while e < n and ref[e] == ref[s]:
            e += 1
        if abs(ref[s]) > 1:
            out.append((s, e, ref[s]))
        s = e
    return out


def metrics(meas, duty, t0, t1, A):
    seg = meas[t0:t1]
    sm = smooth(seg)
    tail = seg[max(0, len(seg) - 100):]
    ss = float(np.mean(tail))
    noise = float(np.std(tail))
    # 以目标值为基准的 5% 带
    band = abs(A) * 0.05
    # 上升时间 10%->90%（用平滑曲线）
    base = float(np.mean(meas[max(0, t0 - 100):t0]))
    span = ss - base
    frac = (sm - base) / span if abs(span) > 1e-6 else np.zeros_like(sm)
    t10 = int(np.argmax(frac >= 0.1))
    t90 = int(np.argmax(frac >= 0.9)) if (frac >= 0.9).any() else -1
    rise = (t90 - t10) if t90 >= 0 else float("nan")
    ovs = (float(np.max(sm)) - ss) / abs(ss) * 100 if abs(ss) > 1e-6 else 0.0
    # 稳定时间：最后一次离开 ±5%·A 带之后
    ok = np.abs(sm - A) <= band
    settle = len(seg)
    for k in range(len(seg) - 1, -1, -1):
        if not ok[k]:
            settle = k + 1
            break
    # 低频振荡：平滑后 ±5% 带外的时间占比
    outside = 100.0 * (1.0 - np.mean(ok))
    return dict(ss=ss, err=ss - A, noise=noise, rise=rise, ovs=ovs,
                settle=settle, outside=outside,
                duty=float(np.mean(duty[max(0, t1 - 100):t1])))


def main():
    sets = parse(sys.argv[1])
    if not sets:
        raise SystemExit("没解析到数据块")
    for tag, a in sets:
        print(f"\n=== {tag} ===")
        ref = a[:, 0]
        for (t0, t1, A) in segs_of(ref):
            print(f"-- 目标 {A:+.0f} RPM ({t1-t0} ms)")
            for i, w in enumerate(WHEELS):
                m = metrics(a[:, 4 + i], a[:, 8 + i], t0, t1, A)
                print(f"   {w}: ss={m['ss']:+7.1f} err={m['err']:+6.1f} "
                      f"noise={m['noise']:4.2f} rise={m['rise']:5.0f}ms "
                      f"ovs={m['ovs']:+5.1f}% settle={m['settle']:4.0f}ms "
                      f"out5%={m['outside']:4.1f}% duty={m['duty']:+6.0f}")


if __name__ == "__main__":
    main()
