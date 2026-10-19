#!/usr/bin/env python3
"""
USART2 底盘链路：协议参考实现 + 台架测试工具

链路：  Jetson Nano / 笔记本 --USB-TTL--> STM32F407 USART2 (PD5/PD6, 115200 8N1)

      ./uart_link_test.py listen  -p /dev/ttyUSB0        # 只听反馈（验证 F407 在发）
      ./uart_link_test.py drive   -p /dev/ttyUSB0 --vx 100 --secs 3
      ./uart_link_test.py spin    -p /dev/ttyUSB0 --rate 350 --secs 3
      ./uart_link_test.py estop   -p /dev/ttyUSB0
      ./uart_link_test.py clear   -p /dev/ttyUSB0
      ./uart_link_test.py selfcheck                      # 纯软件，不需要串口

帧格式与 C 侧（UserLib/Comm/uart_link_config.h）逐字节一致，也是上位机
C++ 工程（Jetson ~/gimbal 的 UartChassisLink）的参考实现，改协议时三处一起改。

⚠ 坐标约定是**底盘库 §4** 那套：vx 前进 / vy 左移 / wz 逆时针为正。
   不是 ~/gimbal 里 ChassisCommand 的老约定（右/前），别混。
"""

import argparse
import struct
import sys
import time

# ============================ 协议常量 ============================

CMD_HEAD = b"\xA5\x5A"
FB_HEAD = b"\x5A\xA5"
FB_TYPE = 0x81
FB_TAIL = 0x0D
WHEEL_ORDER = ("FL", "FR", "BL", "BR")

CMD_FRAME_LEN = 12
FB_FRAME_LEN = 40

CMD_SPEED = 0x01
CMD_ESTOP = 0x02
CMD_SPIN = 0x03
CMD_CLEAR = 0x04

FLAG_EN = 0x01

STATUS_BITS = [
    (0x0001, "IMU_ONLINE"),
    (0x0002, "LINK_OK"),
    (0x0004, "WATCHDOG"),
    (0x0008, "ESTOP"),
    (0x0010, "DRIVER_ON"),
    (0x0020, "SPIN"),
    (0x0040, "GYRO_ON"),
    (0x0080, "IMU_CAL"),
]

VX_UNIT = 0.001      # m/s per LSB
WZ_UNIT = 0.01       # rad/s per LSB


def crc8(data: bytes) -> int:
    """CRC-8/ATM: poly=0x07, init=0x00, 校验值("123456789")=0xF4"""
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


# ============================ 编码 ============================

def build_cmd(cmd: int, seq: int, vx_mm: int = 0, vy_mm: int = 0,
              wz_cs: int = 0, flags: int = FLAG_EN) -> bytes:
    """组装命令帧（12 B）。vx/vy 单位 mm/s，wz 单位 0.01 rad/s"""
    body = (CMD_HEAD
            + bytes([cmd & 0xFF, seq & 0xFF])
            + struct.pack("<hhh", vx_mm, vy_mm, wz_cs)
            + bytes([flags & 0xFF]))
    assert len(body) == CMD_FRAME_LEN - 1, len(body)
    return body + bytes([crc8(body)])


def build_speed(seq, vx_m_s=0.0, vy_m_s=0.0, wz_rad_s=0.0,
                enable=True) -> bytes:
    """按物理单位组命令帧（最常用）"""
    return build_cmd(CMD_SPEED, seq,
                     int(round(vx_m_s / VX_UNIT)),
                     int(round(vy_m_s / VX_UNIT)),
                     int(round(wz_rad_s / WZ_UNIT)),
                     FLAG_EN if enable else 0)


def build_fb(seq, x_mm, y_mm, yaw_cdeg, vx_mm, vy_mm, wz_cs,
             gyro_yaw_cdeg, gyro_wz_cs, status, cmd_cnt, fb_seq,
             wheels=(0, 0, 0, 0)) -> bytes:
    """组装反馈帧（40 B）——测试/交叉校验用，F407 侧用 C 的同名逻辑"""
    body = (FB_HEAD + bytes([FB_TYPE, seq & 0xFF])
            + struct.pack("<ii", x_mm, y_mm)
            + struct.pack("<hhhhhh", yaw_cdeg, vx_mm, vy_mm, wz_cs,
                          gyro_yaw_cdeg, gyro_wz_cs)
            + struct.pack("<HHH", status, cmd_cnt, fb_seq)
            + struct.pack("<hhhh", *wheels))
    assert len(body) == FB_FRAME_LEN - 2, len(body)
    return body + bytes([crc8(body), FB_TAIL])


def build_spin(seq, rate_rad_s, enable=True) -> bytes:
    return build_cmd(CMD_SPIN, seq, 0, 0, int(round(rate_rad_s / WZ_UNIT)),
                     FLAG_EN if enable else 0)


def build_estop(seq) -> bytes:
    return build_cmd(CMD_ESTOP, seq)


def build_clear(seq) -> bytes:
    return build_cmd(CMD_CLEAR, seq)


# ============================ 解码 ============================

def decode_status(status: int):
    return [name for bit, name in STATUS_BITS if status & bit]


def parse_fb(f: bytes):
    """解析一帧 32 B 反馈，失败抛 ValueError"""
    if len(f) != FB_FRAME_LEN:
        raise ValueError("len")
    if f[0:2] != FB_HEAD or f[2] != FB_TYPE or f[FB_FRAME_LEN - 1] != FB_TAIL:
        raise ValueError("head/tail")
    if crc8(f[:38]) != f[38]:
        raise ValueError("crc")

    x_mm, y_mm = struct.unpack_from("<ii", f, 4)
    (yaw_cdeg, vx_mm, vy_mm, wz_cs,
     gyaw_cdeg, gwz_cs) = struct.unpack_from("<hhhhhh", f, 12)
    status, cmd_cnt, fb_seq = struct.unpack_from("<HHH", f, 24)
    wheels = struct.unpack_from("<hhhh", f, 30)

    return {
        "seq": f[3],
        "x_m": x_mm / 1000.0,
        "y_m": y_mm / 1000.0,
        "yaw_deg": yaw_cdeg / 100.0,
        "vx": vx_mm / 1000.0,
        "vy": vy_mm / 1000.0,
        "wz": wz_cs / 100.0,
        "gyro_yaw_deg": gyaw_cdeg / 100.0,
        "gyro_wz": gwz_cs / 100.0,
        "status": status,
        "status_names": decode_status(status),
        "cmd_cnt": cmd_cnt,
        "fb_seq": fb_seq,
        "wheel": dict(zip(WHEEL_ORDER, (w / 100.0 for w in wheels))),
    }


class StreamDecoder:
    """字节流 -> 反馈帧。允许丢字节/噪声，自动重同步"""

    def __init__(self):
        self.buf = bytearray()
        self.crc_err = 0
        self.sync_err = 0

    def feed(self, data: bytes):
        self.buf.extend(data)
        out = []
        while len(self.buf) >= FB_FRAME_LEN:
            if self.buf[0:2] != FB_HEAD:
                self.buf.pop(0)
                self.sync_err += 1
                continue
            frame = bytes(self.buf[:FB_FRAME_LEN])
            try:
                out.append(parse_fb(frame))
            except ValueError:
                self.buf.pop(0)
                self.crc_err += 1
                continue
            del self.buf[:FB_FRAME_LEN]
        return out


def fmt_fb(fb) -> str:
    st = "|".join(fb["status_names"]) or "-"
    return (f"seq={fb['seq']:3d} fb={fb['fb_seq']:5d} cmd={fb['cmd_cnt']:5d} | "
            f"pos=({fb['x_m']:+.3f},{fb['y_m']:+.3f})m yaw={fb['yaw_deg']:+7.2f}° | "
            f"v=({fb['vx']:+.3f},{fb['vy']:+.3f})m/s wz={fb['wz']:+.2f} "
            f"gyro_yaw={fb['gyro_yaw_deg']:+7.2f}° gyro_wz={fb['gyro_wz']:+.2f} | "
            f"轮速 FL{fb['wheel']['FL']:+7.1f} FR{fb['wheel']['FR']:+7.1f} "
            f"BL{fb['wheel']['BL']:+7.1f} BR{fb['wheel']['BR']:+7.1f} | {st}")


# ============================ 串口会话 ============================

class Link:
    def __init__(self, port, baud=115200, hz=100.0, verbose=True):
        try:
            import serial  # pyserial
        except ImportError:
            sys.exit("需要 pyserial：pip3 install pyserial")
        self.ser = serial.Serial(port, baud, timeout=0)
        self.dec = StreamDecoder()
        self.seq = 0
        self.period = 1.0 / hz
        self.last_tx = 0.0
        self.last_fb = None
        self.verbose = verbose

    def rx(self):
        data = self.ser.read(4096)
        frames = self.dec.feed(data) if data else []
        if frames:
            self.last_fb = frames[-1]
            if self.verbose:
                for fb in frames:
                    print(fmt_fb(fb))
        return frames

    def tx(self, frame: bytes):
        self.ser.write(frame)

    def tick(self, frame_builder, stop_event=lambda: False):
        """按 hz 发帧 + 收反馈，直到 stop_event() 返回 True"""
        t0 = time.time()
        while not stop_event():
            now = time.time()
            if now - self.last_tx >= self.period:
                self.last_tx = now
                self.seq = (self.seq + 1) & 0xFF
                self.tx(frame_builder(self.seq))
            self.rx()
            if time.time() - t0 > 3600:
                break
            time.sleep(0.001)


# ============================ 自检 ============================

def selfcheck() -> int:
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        print(("  ✅ " if cond else "  ❌ ") + name + (f"  {detail}" if detail else ""))
        ok = ok and cond

    print("[1] CRC-8/ATM 标准校验值")
    check('crc8("123456789") == 0xF4', crc8(b"123456789") == 0xF4,
          hex(crc8(b"123456789")))

    print("[2] 命令帧结构")
    f = build_speed(7, 0.5, -0.2, 1.0)
    check("长度 12", len(f) == CMD_FRAME_LEN, len(f))
    check("帧头 A5 5A", f[0:2] == CMD_HEAD)
    check("cmd/seq", (f[2], f[3]) == (CMD_SPEED, 7))
    check("vx=+500mm/s", struct.unpack_from("<h", f, 4)[0] == 500)
    check("vy=-200mm/s", struct.unpack_from("<h", f, 6)[0] == -200)
    check("wz=+100cs(0.01rad/s 单位)", struct.unpack_from("<h", f, 8)[0] == 100)
    check("flags EN", f[10] == FLAG_EN)
    check("crc 覆盖 0..10", f[11] == crc8(f[:11]))

    print("[3] 反馈帧往返（用构造帧冒充 F407）")
    body = (FB_HEAD + bytes([FB_TYPE, 33])
            + struct.pack("<ii", -1500, 2500)          # x=-1.5m y=+2.5m
            + struct.pack("<hhhhhh", -9000, 500, -200, 350, 12000, 250)
            + struct.pack("<HHH", 0x0001 | 0x0002 | 0x0010, 1234, 77)
            + struct.pack("<hhhh", 1200, -800, 300, 0))   # 四轮 RPM×100
    frame = body + bytes([crc8(body), FB_TAIL])
    check("构造帧 40B", len(frame) == FB_FRAME_LEN, len(frame))
    fb = parse_fb(frame)
    check("x=-1.5m", abs(fb["x_m"] + 1.5) < 1e-9)
    check("yaw=-90.00°", abs(fb["yaw_deg"] + 90.0) < 1e-9)
    check("vx=+0.5m/s", abs(fb["vx"] - 0.5) < 1e-9)
    check("gyro_yaw=+120°", abs(fb["gyro_yaw_deg"] - 120.0) < 1e-9)
    check("status 解码", fb["status_names"] == ["IMU_ONLINE", "LINK_OK", "DRIVER_ON"],
          fb["status_names"])
    check("四轮转速解码", abs(fb["wheel"]["FL"] - 12.0) < 1e-9 and
          abs(fb["wheel"]["BR"]) < 1e-9, fb["wheel"])

    print("[4] 流式解码：噪声 + 坏帧 + 两帧粘连")
    dec = StreamDecoder()
    good = frame
    bad = bytearray(frame)
    bad[20] ^= 0xFF          # 破坏 payload -> CRC 必须拒
    stream = b"\x00\x11\x22" + bytes(bad) + good + good
    got = dec.feed(stream)
    check("解出 2 帧合法反馈", len(got) == 2, len(got))
    check("CRC 错误被拒（坏帧不冒充好帧）", dec.crc_err >= 1, dec.crc_err)

    print()
    print("自检结果：", "全部通过 ✅" if ok else "存在失败 ❌")
    return 0 if ok else 1


# ============================ 向量导出（C 交叉校验） ============================

CMD_CASES = [
    # (cmd, seq, vx_mm, vy_mm, wz_cs, flags)
    (CMD_SPEED, 0, 0, 0, 0, FLAG_EN),          # 静止
    (CMD_SPEED, 1, 500, -200, 100, FLAG_EN),   # 典型前进+左移+逆时针
    (CMD_SPEED, 255, -1000, 1000, -400, 0),    # 极限值 + 禁能
    (CMD_ESTOP, 7, 0, 0, 0, 0),                # 急停
    (CMD_SPIN, 128, 0, 0, 350, FLAG_EN),       # 小陀螺 3.5 rad/s
    (CMD_CLEAR, 42, 0, 0, 0, 0),               # 清零
    (0x7F, 9, 32767, -32768, 123, 0xAA),       # 未知命令 + i16 边界
]

FB_CASES = [
    # (seq, x_mm, y_mm, yaw_cdeg, vx_mm, vy_mm, wz_cs,
    #  gyro_yaw_cdeg, gyro_wz_cs, status, cmd_cnt, fb_seq, wheels)
    (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, (0, 0, 0, 0)),                    # 全零
    (33, -1500, 2500, -9000, 500, -200, 350, 12000, -250,
     0x0001 | 0x0002 | 0x0010, 1234, 77, (1200, -800, 300, 0)),           # 典型值
    (255, 2147483647, -2147483648, 32767, -32768, 32767, -32768,
     32767, -32768, 0xFF7F, 65535, 65535, (32767, -32768, 1, -1)),        # 全量程边界
    (128, -1000, -2000, 18000, -1000, 1000, 400, -18000, 400,
     0x0008 | 0x0020, 1, 1, (-500, -500, 900, 900)),                      # 急停 + 自旋
]


def emit_vectors() -> str:
    """输出 C 头文件：Python 生成的帧 + 对应字段，供 codec_test.c 双向校验"""
    out = ["/* 由 tools/uart_link_test.py vectors 生成，勿手改 */",
           "/* 用途：让 PC 上的 C 编解码实现与 Python 参考实现逐字节对齐 */",
           "", "static const ulink_cmd_t CMD_CASES[] = {"]
    for c in CMD_CASES:
        out.append("    { %d, %d, %d, %d, %d, %d }," % c)
    out += ["};", "", "static const uint8_t CMD_FRAMES[][ULINK_CMD_FRAME_LEN] = {"]
    for c in CMD_CASES:
        fr = build_cmd(c[0], c[1], c[2], c[3], c[4], c[5])
        out.append("    { " + ", ".join("0x%02X" % b for b in fr) + " },")
    out += ["};", "", "static const ulink_fb_t FB_CASES[] = {"]
    for f in FB_CASES:
        w = f[12]
        out.append("    { %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, "
                   "{ %d, %d, %d, %d } }," % (f[:12] + (w[0], w[1], w[2], w[3])))
    out += ["};", "", "static const uint8_t FB_FRAMES[][ULINK_FB_FRAME_LEN] = {"]
    for f in FB_CASES:
        fr = build_fb(*f)
        out.append("    { " + ", ".join("0x%02X" % b for b in fr) + " },")
    out += ["};", "", "static const unsigned N_CMD_CASES = sizeof(CMD_CASES) / sizeof(CMD_CASES[0]);",
            "static const unsigned N_FB_CASES = sizeof(FB_CASES) / sizeof(FB_CASES[0]);", ""]
    return "\n".join(out)


# ============================ 链路体检（真机） ============================
#
# 逐项验证「Jetson/PC <-> F407」这条链路，不依赖云台：
#   [1] 基础连通：反馈帧率 / 命令计数 / seq 回显 / 状态位 / CRC·同步错
#   [2] 急停与恢复：ESTOP 后 DRIVER_ON 必须灭，再发 EN 必须恢复
#   [3] 里程计清零：CLEAR 后位姿归零
#   [4] 低速行驶（--move 才做）：给 +vx，看正解 vx 符号是否也变正

def run_check(args) -> int:
    link = Link(args.port, args.baud, args.hz, verbose=False)
    ok_all = True

    def verdict(cond, detail=""):
        nonlocal ok_all
        ok_all = ok_all and cond
        print(("  ✅ " if cond else "  ❌ ") + detail)

    def pump(seconds, builder=None):
        """跑 seconds 秒：按 hz 发帧（builder 为空则只收不发），返回 (帧列表, 期间统计)"""
        t_end = time.time() + seconds
        frames = []
        t0 = time.time()
        while time.time() < t_end:
            now = time.time()
            if builder and now - link.last_tx >= link.period:
                link.last_tx = now
                link.seq = (link.seq + 1) & 0xFF
                link.tx(builder(link.seq))
            frames.extend(link.rx())
            time.sleep(0.001)
        return frames, time.time() - t0

    # ---------------- [1] 基础连通 ----------------
    print("[1] 基础连通（静止，2.5 s，EN=1 速度全 0）")
    pump(0.3)                      # 先只听 0.3 s，拿到基准计数
    f0 = link.last_fb
    base_cmd = f0["cmd_cnt"] if f0 else None
    frames, dur = pump(2.5, lambda s: build_speed(s, 0.0, 0.0, 0.0))
    if not frames:
        verdict(False, "一帧反馈都没收到（检查串口/波特率/接线）")
        return 1
    hz = len(frames) / dur
    last = frames[-1]
    verdict(abs(hz - 100) < 15, f"反馈帧率 {hz:.1f} Hz（期望 ~100）")
    if base_cmd is not None:
        dm = (last["cmd_cnt"] - base_cmd) & 0xFFFF
        verdict(abs(dm / dur - 100) < 20, f"命令计数增量 {dm} 帧 / {dur:.1f}s ≈ {dm/dur:.1f} Hz")
    verdict(last["seq"] == link.seq or (link.seq - last["seq"]) & 0xFF <= 3,
            f"seq 回显 期望{link.seq} 实际{last['seq']}")
    names = set(last["status_names"])
    verdict("LINK_OK" in names, "状态位 LINK_OK 亮")
    verdict("IMU_ONLINE" in names, "状态位 IMU_ONLINE 亮")
    verdict("DRIVER_ON" in names, "状态位 DRIVER_ON 亮（TB6612 使能）")
    verdict(link.dec.crc_err == 0, f"CRC 错 {link.dec.crc_err} 次")
    verdict(link.dec.sync_err == 0, f"同步错 {link.dec.sync_err} 次")

    # ---------------- [2] 急停与恢复 ----------------
    print("[2] 急停与恢复")
    link.tx(build_estop(link.seq))
    time.sleep(0.05)
    frames, _ = pump(0.4)
    last = frames[-1] if frames else None
    if last:
        verdict("ESTOP" in last["status_names"], "急停后 ESTOP 位亮")
        verdict("DRIVER_ON" not in last["status_names"], "急停后 DRIVER_ON 已清（STBY 拉低）")
    else:
        verdict(False, "急停后没有反馈")

    frames, _ = pump(0.5, lambda s: build_speed(s, 0.0, 0.0, 0.0))
    last = frames[-1] if frames else None
    if last:
        verdict("DRIVER_ON" in last["status_names"], "发 EN=1 后 DRIVER_ON 恢复")
        verdict("ESTOP" not in last["status_names"], "ESTOP 锁存已解除")

    # ---------------- [3] 里程计清零 ----------------
    print("[3] 里程计清零")
    link.tx(build_clear(link.seq))
    time.sleep(0.05)
    frames, _ = pump(0.3)
    last = frames[-1] if frames else None
    if last:
        zero = abs(last["x_m"]) < 1e-6 and abs(last["y_m"]) < 1e-6 and abs(last["yaw_deg"]) < 0.01
        verdict(zero, f"清零后位姿 x={last['x_m']:+.3f} y={last['y_m']:+.3f} yaw={last['yaw_deg']:+.2f}°")

    # ---------------- [5] 看门狗（断流 -> 自动停车 -> 恢复）----------------
    print("[5] 看门狗：故意断流 1.2 s")
    frames, _ = pump(1.2)                      # 只收不发
    last = frames[-1] if frames else None
    if last:
        names = set(last["status_names"])
        verdict("WATCHDOG" in names, "断流后 WATCHDOG 位亮（F407 自己判超时）")
        verdict("LINK_OK" not in names, "断流 >500 ms 后 LINK_OK 熄灭")
        verdict(abs(last["vx"]) < 0.05 and abs(last["wz"]) < 0.05,
                f"断流后速度已归零 v=({last['vx']:+.3f},{last['vy']:+.3f})m/s "
                f"wz={last['wz']:+.2f}")
        verdict("DRIVER_ON" in names, "断流后 STBY 仍保持（不失能、不松轴）")
    frames, _ = pump(0.5, lambda s: build_speed(s, 0.0, 0.0, 0.0))
    last = frames[-1] if frames else None
    if last:
        names = set(last["status_names"])
        verdict("WATCHDOG" not in names, "恢复发帧后 WATCHDOG 清除")
        verdict("LINK_OK" in names, "恢复发帧后 LINK_OK 重新亮")

    # ---------------- [4] 三轴方向验证（可选，需架空）----------------
    if args.move:
        v = args.move / 1000.0          # m/s
        w = 1.0                          # rad/s（逆时针）
        print(f"[4] 三轴方向（车必须架空）：vx=+{args.move:.0f}mm/s  vy=+{args.move:.0f}mm/s  wz=+{w:.2f}rad/s")

        # 4a 前进
        frames, _ = pump(1.2, lambda s: build_speed(s, v, 0.0, 0.0))
        last = frames[-1] if frames else None
        if last:
            m = last["vx"] * 1000.0
            verdict(m > 0, f"vx 指令 +{args.move:.0f} -> 正解 {m:+.0f} mm/s（符号一致）")
            verdict(abs(m - args.move) < args.move * 0.6,
                    f"vx 幅值 {m:+.0f} mm/s（容差 60%）")
        pump(0.4, lambda s: build_speed(s, 0.0, 0.0, 0.0))

        # 4b 左移（+y）
        frames, _ = pump(1.2, lambda s: build_speed(s, 0.0, v, 0.0))
        last = frames[-1] if frames else None
        if last:
            m = last["vy"] * 1000.0
            verdict(m > 0, f"vy 指令 +{args.move:.0f} -> 正解 {m:+.0f} mm/s（符号一致）")
            verdict(abs(m - args.move) < args.move * 0.6,
                    f"vy 幅值 {m:+.0f} mm/s（容差 60%）")
        pump(0.4, lambda s: build_speed(s, 0.0, 0.0, 0.0))

        # 4c 逆时针自旋（+wz）
        frames, _ = pump(1.2, lambda s: build_speed(s, 0.0, 0.0, w))
        last = frames[-1] if frames else None
        if last:
            m = last["wz"]
            verdict(m > 0, f"wz 指令 +{w:.2f} -> 正解 {m:+.2f} rad/s（符号一致）")
            verdict(0.3 < m < 5.0, f"wz 幅值 {m:+.2f} rad/s（陀螺外环会补偿，容许偏大）")
        pump(0.4, lambda s: build_speed(s, 0.0, 0.0, 0.0))
        print("      说明：架空时轮子空转，正解速度应≈指令速度；符号不对就是电机/编码器方向问题")
    else:
        print("[4] 三轴方向：跳过（加 --move 100 才做，需架空）")

    # ---------------- 收尾 ----------------
    pump(0.3, lambda s: build_speed(s, 0.0, 0.0, 0.0, enable=False))
    print()
    print("结论：", "链路全部项目通过 ✅" if ok_all else "存在失败项 ❌")
    return 0 if ok_all else 1


# ============================ 麦轮四轮方向检查 ============================
#
# 架空后依次给 +vx / +vy / +wz，打印四个轮子的实际转速（RPM），
# 并与麦轮逆解的期望符号对比。符号全对 = 运动学与电机/编码器方向一致。
#
# 期望符号（CHASSIS_ROLLER_SIGN = +1 时）：
#           FL   FR   BL   BR
#   vx>0    +    +    +    +        前进：四轮同向
#   vy>0    +    -    -    +        左移：左前/右后向前，右前/左后向后
#   wz>0    -    -    +    +        逆时针：左侧后退，右侧前进

WHEEL_EXPECT = {
    "vx": {"desc": "前进 vx>0", "signs": (+1, +1, +1, +1)},
    "vy": {"desc": "左移 vy>0", "signs": (+1, -1, -1, +1)},
    "wz": {"desc": "逆时针 wz>0", "signs": (-1, -1, +1, +1)},
}


def run_wheels(args) -> int:
    link = Link(args.port, args.baud, args.hz, verbose=False)
    print("四轮方向检查（车必须架空）")
    print(f"  {'工况':<12} {'FL':>8} {'FR':>8} {'BL':>8} {'BR':>8}   判定")
    ok_all = True

    def pump_last(seconds, builder):
        frames = []
        t_end = time.time() + seconds
        while time.time() < t_end:
            now = time.time()
            if now - link.last_tx >= link.period:
                link.last_tx = now
                link.seq = (link.seq + 1) & 0xFF
                link.tx(builder(link.seq))
            frames.extend(link.rx())
            time.sleep(0.001)
        return frames[-1] if frames else None

    for axis, spec in WHEEL_EXPECT.items():
        if axis == "vx":
            fb = pump_last(1.0, lambda s: build_speed(s, 0.15, 0.0, 0.0))
        elif axis == "vy":
            fb = pump_last(1.0, lambda s: build_speed(s, 0.0, 0.15, 0.0))
        else:
            fb = pump_last(1.0, lambda s: build_speed(s, 0.0, 0.0, 2.0))
        pump_last(0.4, lambda s: build_speed(s, 0.0, 0.0, 0.0))
        if fb is None:
            print(f"  {spec['desc']:<12} 没有反馈 ❌")
            ok_all = False
            continue
        w = fb["wheel"]
        vals = [w["FL"], w["FR"], w["BL"], w["BR"]]
        signs_ok = all(v * e >= 0 and abs(v) > 2.0 for v, e in zip(vals, spec["signs"]))
        ok_all = ok_all and signs_ok
        print(f"  {spec['desc']:<12} " +
              " ".join(f"{v:+8.1f}" for v in vals) +
              f"   {'✅' if signs_ok else '❌ 期望符号 ' + str(spec['signs'])}")

    pump_last(0.3, lambda s: build_speed(s, 0.0, 0.0, 0.0, enable=False))
    print()
    if ok_all:
        print("四轮符号全部正确 ✅（若实车方向仍不对，就是辊筒朝向/机械装配问题，见文档）")
    else:
        print("存在符号不一致 ❌ —— 逐轮对比上面的期望值，反的那一轮查 ENC_DIR / 电机线序")
    return 0 if ok_all else 1


# ============================ 长时间稳定性（soak） ============================# ============================ 长时间稳定性（soak） ============================

def run_soak(args) -> int:
    """N 秒持续 100 Hz 收发，统计丢帧/乱序/卡顿。全程速度 0，车不动。"""
    link = Link(args.port, args.baud, args.hz, verbose=False)
    print(f"[soak] {args.secs:.0f} s，100 Hz 收发，速度全 0（车不会动）")

    sent = 0
    recv = 0
    gaps = 0          # 反馈帧序号跳变次数
    max_gap_ms = 0.0
    last_fb_seq = None
    last_rx_t = None
    prev_cmd = None
    t0 = time.time()
    while time.time() - t0 < args.secs:
        now = time.time()
        if now - link.last_tx >= link.period:
            link.last_tx = now
            link.seq = (link.seq + 1) & 0xFF
            link.tx(build_speed(link.seq, 0.0, 0.0, 0.0))
            sent += 1
        for fb in link.rx():
            recv += 1
            if last_fb_seq is not None:
                d = (fb["fb_seq"] - last_fb_seq) & 0xFFFF
                if d > 1:
                    gaps += d - 1
            last_fb_seq = fb["fb_seq"]
            if last_rx_t is not None:
                max_gap_ms = max(max_gap_ms, (now - last_rx_t) * 1000.0)
            last_rx_t = now
            if prev_cmd is None:
                prev_cmd = fb["cmd_cnt"]
            prev_cmd = fb["cmd_cnt"]
    dur = time.time() - t0

    last = link.last_fb
    print(f"  时长            {dur:.1f} s")
    print(f"  发送命令帧      {sent}  ({sent / dur:.1f} Hz)")
    print(f"  收到反馈帧      {recv}  ({recv / dur:.1f} Hz)")
    print(f"  反馈丢帧        {gaps} 帧")
    print(f"  最大接收间隔    {max_gap_ms:.1f} ms")
    print(f"  CRC 错 / 同步错 {link.dec.crc_err} / {link.dec.sync_err}")
    if last:
        print(f"  F407 收到命令   {last['cmd_cnt']} 帧  回显 seq={last['seq']}")
        print(f"  状态位          {'|'.join(last['status_names']) or '-'}")
    ok = (abs(recv / dur - 100) < 12 and gaps <= max(3, recv // 500)
          and link.dec.crc_err == 0 and max_gap_ms < 100)
    print()
    print("结论：", "长时间稳定 ✅" if ok else "存在异常 ❌")
    return 0 if ok else 1


# ============================ CLI ============================

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="USART2 底盘链路测试工具")
    ap.add_argument("mode", choices=["listen", "drive", "spin", "estop", "clear",
                                     "selfcheck", "vectors", "check", "soak",
                                     "wheels"])
    ap.add_argument("-p", "--port", default="/dev/ttyUSB0")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("--hz", type=float, default=100.0)
    ap.add_argument("--vx", type=float, default=0.0, help="m/s")
    ap.add_argument("--vy", type=float, default=0.0, help="m/s")
    ap.add_argument("--wz", type=float, default=0.0, help="rad/s")
    ap.add_argument("--rate", type=float, default=3.5, help="自旋 rad/s（spin 模式）")
    ap.add_argument("--secs", type=float, default=3.0)
    ap.add_argument("--move", type=float, default=0.0,
                    help="check 模式下的低速行驶测试 vx (mm/s)，默认不做")
    ap.add_argument("-q", "--quiet", action="store_true")
    args = ap.parse_args(argv)

    if args.mode == "selfcheck":
        return selfcheck()

    if args.mode == "vectors":
        print(emit_vectors())
        return 0

    if args.mode == "check":
        return run_check(args)

    if args.mode == "soak":
        return run_soak(args)

    if args.mode == "wheels":
        return run_wheels(args)

    if args.mode == "listen":
        link = Link(args.port, args.baud, args.hz, verbose=not args.quiet)
        print(f"监听 {args.port} … Ctrl-C 结束")
        print(f"期望：约 100 Hz 一行反馈；收到 WATCHDOG 位说明上位机没在发控制帧（正常）")
        try:
            while True:
                link.rx()
                time.sleep(0.001)
        except KeyboardInterrupt:
            print(f"\n收：合法帧丢失 {link.dec.crc_err} / 同步错 {link.dec.sync_err}")
        return 0

    link = Link(args.port, args.baud, args.hz, verbose=not args.quiet)

    if args.mode == "drive":
        print(f"前进/平移 {args.secs}s：vx={args.vx} vy={args.vy} wz={args.wz}")
        link.tick(lambda s: build_speed(s, args.vx, args.vy, args.wz))
    elif args.mode == "spin":
        print(f"小陀螺 {args.secs}s：{args.rate} rad/s")
        link.tick(lambda s: build_spin(s, args.rate))
    elif args.mode == "estop":
        print("急停：STBY 拉低")
        link.tx(build_estop(link.seq))
        time.sleep(0.2)
        link.rx()
        return 0
    elif args.mode == "clear":
        print("里程计 + 陀螺 yaw 清零")
        link.tx(build_clear(link.seq))
        time.sleep(0.2)
        link.rx()
        return 0

    # 收尾：明确发 EN=0 的停止帧（比直接拔线更礼貌）
    link.tx(build_speed(link.seq, 0.0, 0.0, 0.0, enable=False))
    time.sleep(0.2)
    link.rx()
    print("已停止")
    return 0


if __name__ == "__main__":
    sys.exit(main())
