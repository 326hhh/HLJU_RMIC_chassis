# Chassis —— 麦克纳姆轮底盘基础库

> 本文与代码同步于 2026-10-18（git `889bae7`）。改完参数请顺手更新下面的「当前参数」表。
> 接线（引脚/驱动板/电机线）以 [`../../README.md`](../../README.md) 为准；
> 原理、标定过程、整定数据以 [`../../docs/CHASSIS_LIBRARY.md`](../../docs/CHASSIS_LIBRARY.md) 为准。

## 1. 定位

把它当成一个**"车体速度指令"的黑箱**：

- 输入：`vx`（前）`vy`（左）`wz`（逆时针），单位 m/s、m/s、rad/s
- 输出：四个轮子的 TB6612 占空比，外加一份里程计（`x, y, yaw`）
- 内部自己解决：麦轮逆解、轮速斜坡、速度环 PID、编码器测速、正解积分

不带 RTOS、不依赖裁判系统，裸机 1 kHz。现在**没有任何指令入口**（遥控/CAN/串口解析都还没做），
调用方负责调 `chassis_set_velocity()`。

## 2. 快速接入

```c
#include "chassis.h"

/* main() 的 USER CODE BEGIN 2 —— 必须在 MX_*_Init() 之后 */
chassis_init();

while (1) {
    chassis_poll();                       /* 主循环每次必调，内部自己掐 1ms 节拍 */
    chassis_set_velocity(0.5f, 0.0f, 0.0f); /* 前进 0.5 m/s */
}
```

> ⚠️ 主循环里**不能有 `HAL_Delay()` 之类的长阻塞**。`chassis_poll()` 靠 HAL tick 判定节拍，
> 阻塞多久就掉多少拍（`dt` 会被限幅到 5 ms 兜底，但闭环质量会明显变差）。

## 3. 文件清单

| 文件 | 行数 | 职责 | 详见 |
|---|---|---|---|
| `chassis_config.h` | 151 | **所有可调参数**：物理尺寸、限幅、PID、方向、开关 | §7 |
| `chassis.c` / `.h` | 296 / 90 | 主模块：硬件绑定表 `s_bind[]`、1 kHz 循环、里程计、模式切换 | §6.5 §8 |
| `motor.c` / `.h` | 65 / 43 | TB6612 单路：占空比 ±1000 → IN1/IN2 + CCR | §6.1 |
| `encoder.c` / `.h` | 77 / 44 | 编码器：输入数字滤波、16→32 位累加、10 ms 滑窗测速 | §6.2 |
| `pid.c` / `.h` | 91 / 46 | 位置式 PI(D)：速度前馈、加速度前馈、条件积分抗饱和 | §6.3 |
| `mecanum.c` / `.h` | 53 / 37 | 麦轮正解/逆解（互为逆运算） | §6.4 |
| `chassis_math.h` | 40 | clamp / wrap_pi / ramp，纯 inline | §3 |
| `chassis_debug.c` / `.h` | 110 / 16 | USART1 调试输出（RawText / JustFloat 双协议） | §6.6 |
| `chassis_demo.c` / `.h` | 133 / 15 | 演示状态机（前进→左移→左转→后退→右移→右转），**可整文件删除** | §12 |
| `chassis_tune.c` / `.h` | 517 / 36 | PID 整定实验（4 个实验），`CHASSIS_TUNE_ENABLE=0` 时整个模块编译为空 | §6.8 |

## 4. 数据流

```
chassis_set_velocity(vx, vy, wz)          ← 指令入口，内部限幅
        │
        ▼
  mecanum_inverse()   车体速度 → 四轮 RPM
        │  轮速限幅 ±250 RPM · 斜坡 ±1500 RPM/s
        ▼
  pid_calc()          duty = Kff·ref + Kacc·dref/dt + Kp·e + Ki·∫e
        │   （条件积分抗饱和；实测 Kd 恒为 0）
        ▼
  motor_set_duty()    ±1000 → IN1/IN2 + PWM CCR
        │
        ▼
  TB6612 ──► 电机 ──► 编码器 ──► encoder_update()  滑窗测速 + ENC_DIR 归一
                                      │
                                      ├──► pid_calc() 的反馈
                                      └──► mecanum_forward() ──► 里程计 x/y/yaw
```

整条链 **1 kHz 执行一次**，全在 `chassis_update(dt)` 里，没有中断参与。

## 5. 接口速查

### 5.1 正式接口 —— 跑车只用这 5 个

| 函数 | 作用 | 备注 |
|---|---|---|
| `chassis_init(void)` | 绑定电机/编码器、启 PWM、置高 STBY | 一次 |
| `chassis_poll(void)` | 1 kHz 控制循环 | 主循环每次必调；未到节拍立即返回 |
| `chassis_set_velocity(vx,vy,wz)` | 车体速度指令 | 内部自动限幅，不会拒绝 |
| `chassis_stop(void)` | 目标速度清零 | 走斜坡减速，不是急停 |
| `chassis_state(void)` | 读测量值/里程计 | 返回 `const` 指针，只读 |

坐标系：`+x` 前进、`+y` 左移、`+wz` 逆时针（RM 约定，俯视）→ §4。

轮序固定 **FL(0) / FR(1) / BL(2) / BR(3)** = 左前 / 右前 / 左后 / 右后，
所有数组、所有 PID 参数都按这个顺序。

### 5.2 整定/调试接口 —— 正式固件里不要用

| 函数 | 唯一用途 |
|---|---|
| `chassis_set_mode()` | 切 `NORMAL` / `OPEN_DUTY` / `WHEEL_REF`，**绕过运动学与斜坡** |
| `chassis_set_open_duty()` | 四轮同占空比，开环标定 |
| `chassis_set_wheel_ref()` | 四轮直给 RPM，阶跃/斜坡实验 |
| `chassis_set_wheel_gain()` | 在线改某轮 Kp/Ki/Kacc（会清积分） |
| `chassis_driver_enable()` | `0` = STBY 拉低，电机高阻自由（手转测编码器用） |

> 这些接口存在的唯一理由是 `chassis_tune.c` 要用。模式一切换就清 PID 积分和加速度前馈状态，
> 正常跑车时 `s_mode` 恒为 `CHASSIS_MODE_NORMAL`。→ §6.8

## 6. 编译开关

全部集中在 `chassis_config.h`，改完重新编译即可。

| 宏 | 默认 | 什么时候改 |
|---|---|---|
| `CHASSIS_DEBUG_ENABLE` | `1` | 正式跑车改 `0` —— JustFloat 每帧 56 B @115200 约阻塞 4.9 ms，会吃掉控制节拍 |
| `CHASSIS_DEBUG_PROTOCOL` | JustFloat | VOFA+ 画波形用 JustFloat；想终端直接读文本用 RawText |
| `CHASSIS_TUNE_ENABLE` | `0` | 调 PID 时改 `1`，`main.c` 会改调 `chassis_tune_poll()` 并停用调试输出 |
| `CHASSIS_TUNE_EXPERIMENT` | `2` | 1 静态标定 / 2 斜升扫参 / 3 转向目检 / 4 惯性滑行（仅 TUNE 打开时有效） |
| `CHASSIS_DEMO_ENABLE` | `0` | 架空自检时改 `1`（自动跑六段演示） |
| `CHASSIS_DRIVER_DISABLE` | `0` | 只在两种情况下改 `1`：手转轮子测编码器 / 怀疑电机乱转。**跑车必须为 0** |

## 7. 首次上电必做（重新布线后要重做 2、3 两步）

1. **CPR 实测**：`CHASSIS_DRIVER_DISABLE=1`，手转某轮整一圈，看 `cnt` 变化量是否 ≈ 1760
   → 不对就改 `CHASSIS_ENCODER_CPR`；§10.1
2. **编码器方向**：跑实验 1，看 `duty=+1000` 那行，**读数与占空比反号**的轮子把 `CHASSIS_ENC_DIR_x` 取反；§10.2
3. **电机方向**：跑实验 3 人眼目检，四轮反了的去 `chassis.c` 的 `s_bind[]` 对调 `fwd_in1/fwd_in2`，
   **同时**把该轮 `ENC_DIR` 再取反一次；§10.2
4. **运动学符号**：`CHASSIS_DEMO_ENABLE=1` 跑演示，前进直但左移变右移 / 旋转反向
   → 翻转 `CHASSIS_ROLLER_SIGN`；§10.3

## 8. 当前参数与实测状态

**物理参数**：轮径 80 mm、轴距 200.20 mm、轮距 240.00 mm、CPR 1760（理论值，待实测校准）

**限幅**：轮速 ±250 RPM、斜坡 1500 RPM/s、车体 ±1.0 m/s / ±4.0 rad/s

**PID（逐轮独立，改任一轮只影响该轮）**：

| 轮 | Kp | Ki | Kd | I_MAX | Kff | Kacc |
|---|---|---|---|---|---|---|
| FL 左前 | 3.0 | 40 | 0 | 20 | 3.85 | 0.25 |
| FR 右前 | 3.0 | 40 | 0 | 20 | 3.85 | 0.25 |
| BL 左后 | 3.0 | 40 | 0 | 20 | 3.95 | 0.25 |
| BR 右后 | 3.0 | 40 | 0 | 20 | 3.50 | 0.25 |

**标定结论**：`ENC_DIR` = +1/−1/+1/−1（右两轮 AB 线接反）；`ROLLER_SIGN` = +1

**实测指标**（2026-10-18，四轮架空、稳压供电）：

| 项 | 结果 |
|---|---|
| 稳态误差 | 120 RPM 时 <1 RPM，25 RPM 低速仍稳在 ±1 RPM |
| 斜坡超调 | 5~11% |
| 测速噪声 | 1.5~2.0 RPM（std） |
| 满占空比转速 | 260~296 RPM（四轮不同，BR 最快）→ 上限只能取 250 |
| 启动死区 | 正向 80~100‰、反向约 50‰（由积分自动补偿） |
| 里程计 | 演示前进段 yaw 漂移 0.1° |

> ⚠️ **以上全是架空无载整定结果**。落地后负载增大，建议低速（0.2~0.3 m/s）复测 yaw 漂移，
> 必要时把 `Kff` 整体上调 5~10% 让积分少干活。→ §11.5

## 9. 五个常见坑

| 症状 | 原因 | 对策 |
|---|---|---|
| 转速随机尖刺、占空比抖几百‰ | 编码器幽灵计数（AB 线噪声/接触） | `ENC_INPUT_FILTER` 已在 `encoder.c` 里硬写为 8，**不要靠 CubeMX 配**（会被覆盖）；仍抖就查线 |
| 某轮实测为负+占空比顶 ±1000 飞转 | 电机方向和编码器方向只反了一个 | 回 §7 第 2、3 步，两个方向**分开**标 |
| 到不了目标转速、差一大截 | 前馈偏小 / 积分太慢 | 加大该轮 Kff；或加大 Ki |
| 占空比短时冲到近 1000 再回落 | 缺加速度前馈，积分在斜坡里"抢"力矩 | 加大 `Kacc`（0.05 一档试） |
| 加了 Kd 反而更差 | 测速是 1 ms 采样 + 10 ms 滑窗，微分会把量化噪声放大成尖刺 | **速度环不要用 Kd**，默认就是 0 |

## 10. 详细文档索引

| 想看什么 | 去哪 |
|---|---|
| 坐标系与符号约定、麦轮公式推导 | §4、§14 |
| 各模块实现细节 | §6.1 ~ §6.8 |
| 参数逐项含义与调法 | §7、§11 |
| 硬件绑定表 `s_bind[]` | §8 |
| 上电校准全流程 | §10 |
| PID 整定方法 + 实测数据 + 现象对照表 | §11 |
| 已知限制与后续计划 | §13 |
| CAN 通信验证 | §15（另见 `UserLib/Comm/README.md`） |
| 上位机 Jetson `~/gimbal` | §16 |
| IMU（LSM6DSV16X）与陀螺偏航闭环 / 小陀螺 | §17；驱动本身见 `../IMU/README.md` |
| 版本记录 | §18 |
| 整定数据解析脚本 | `../../tools/step_analysis.py` |
