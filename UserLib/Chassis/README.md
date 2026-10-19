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

## 3. 目录结构与文件详解

### 3.1 目录结构

```
UserLib/Chassis/
├── chassis_config.h      # 所有可调参数（物理尺寸/限幅/PID/方向/开关）
├── chassis.h             # 主模块对外接口 + chassis_state_t 状态快照
├── chassis.c             # 主模块实现：硬件绑定表、1kHz 控制循环、里程计、模式切换
├── chassis_math.h        # 通用小工具（clamp / wrap_pi / ramp / RPM↔rad/s），纯 inline
├── chassis_gyro.h        # 陀螺偏航闭环接口（模式枚举 / 状态 / 配置入口）
├── chassis_gyro.c        # 陀螺 PI 外环实现（RATE / SPIN / YAW_TARGET 三种模式）
├── mecanum.h             # 麦轮运动学：轮序枚举 + 正逆解原型
├── mecanum.c             # 麦轮正/逆解实现
├── motor.h               # TB6612 单路抽象（占空比 ±1000 → 方向引脚 + CCR）
├── motor.c               # TB6612 单路实现
├── encoder.h             # 编码器抽象：滑窗测速 + 32 位累计
├── encoder.c             # 编码器实现（输入滤波 / 溢出拼接 / 10ms 滑窗）
├── pid.h                 # 位置式 PI(D) 数据结构与控制律说明
├── pid.c                 # 位置式 PI(D) 实现（前馈 / 抗饱和 / 微分低通）
├── chassis_debug.h       # 调试输出接口
├── chassis_debug.c       # USART1 调试输出（RawText / JustFloat 双协议）
├── chassis_demo.h        # 演示状态机接口
├── chassis_demo.c        # 演示状态机实现（前进/左移/左转/后退/右移/右转/小陀螺）
├── chassis_tune.h        # PID 整定实验接口
├── chassis_tune.c        # 五种整定实验实现（CHASSIS_TUNE_ENABLE=0 时整体为空）
└── README.md
```

### 3.2 逐个文件详解

#### `chassis_config.h` —— 参数总表（改完重新编译即可）

全部可调参数集中在这一个头文件，正文只有 `#define`，不含任何代码：

- **物理参数**：轮径（半径 40 mm）、半轴距 100.1 mm、半轮距 120 mm、编码器 CPR 1760。
- **电机/驱动**：轮速上限 250 RPM、轮速斜坡 1500 RPM/s。
- **车体限幅**：`±1.0 m/s`（vx/vy）、`±4.0 rad/s`（wz）。
- **测速/控制周期**：控制周期 1 ms、测速滑动窗口 10 ms。
- **四轮独立 PID**：每轮一套 `KP/KI/KD/I_MAX/KFF/KACC`（轮序 FL FR BL BR）。
- **编码器方向**：每轮 `CHASSIS_ENC_DIR_x`（+1 / -1），只归一测量符号。
- **陀螺外环**：`CHASSIS_GYRO_ENABLE`、默认模式、`Kp/Ki/I_MAX/I_BAND/CORR_MAX/WZ_LIMIT`、
  角度环增益、静止判定阈值、小陀螺默认速率。
- **调试与整定开关**：`CHASSIS_DEBUG_ENABLE`、`CHASSIS_DEBUG_PROTOCOL`、
  `CHASSIS_TUNE_ENABLE` / `CHASSIS_TUNE_EXPERIMENT`、`CHASSIS_DEMO_ENABLE`、`CHASSIS_DRIVER_DISABLE`。

#### `chassis.h` —— 主模块对外接口

- 定义 `chassis_state_t`：指令速度（`cmd_vx/vy/wz`）、正解测量速度（`meas_*`）、
  四轮目标/实测轮速（`wheel_rpm_ref/meas`）、占空比（`wheel_duty`）、编码器累计（`wheel_count`）、
  里程计（`x/y/yaw`）、驱动状态（`driver_on`）、IMU 镜像（`imu_wz/imu_yaw/wz_eff/imu_online`）。
- 定义工作模式枚举：`NORMAL`（正常）、`OPEN_DUTY`（开环占空比）、`WHEEL_REF`（直给轮速）。
- 声明全部公开函数（见 §5 接口速查）。

#### `chassis.c` —— 主模块实现

- **硬件绑定表 `s_bind[]`**：每轮一组 `{PWM 定时器/通道, IN1/IN2 端口与引脚, 正转电平, 编码器定时器}`，
  轮序固定 FL/FR/BL/BR。
- **模块实例**：`s_motor[4]`、`s_enc[4]`、`s_pid[4]`、目标轮速/上拍目标/加速度前馈系数、`chassis_state_t`。
- **`chassis_init()`**：按 `CHASSIS_DRIVER_DISABLE` 置 STBY；逐轮初始化电机（启 PWM）、编码器（编码器模式）、PID；
  清状态；调用 `chassis_gyro_init()`（内含 `imu_init()`）。
- **`chassis_poll()`**：读 HAL tick，未到 1 ms 直接返回；`dt` 超过 5 ms 时限幅；调用 `chassis_update(dt)`。
- **`chassis_update(dt)`（1 kHz 控制核心）**：
  1. `imu_poll()` 采样 IMU；
  2. `chassis_gyro_update()` 算最终 `wz_eff`（陀螺外环）；
  3. `mecanum_inverse()` 逆解 + 轮速限幅；
  4. 逐轮：`encoder_update()` 测速 + 方向归一 → 目标轮速走斜坡（`chassis_rampf`）→
     `pid_calc()` + 加速度前馈 → 限幅 → `motor_set_duty()`；
     `OPEN_DUTY`/`WHEEL_REF` 调试模式在此分流（绕过运动学与斜坡）；
  5. `mecanum_forward()` 正解 → 里程计积分（`x/y/yaw`）；
  6. 镜像 IMU 状态到 `chassis_state_t`；整定模式下回调 `chassis_tune_on_tick()`。
- **模式切换**：`chassis_set_mode()` 会清 PID 积分与斜坡状态，防止切换瞬间的冲击。

#### `chassis_math.h` —— 通用小工具

纯头文件 inline 实现：`chassis_clampf()`（限幅）、`chassis_wrap_pi()`（角度归一到 ±π）、
`chassis_rampf()`（按斜率逼近目标）、`CHASSIS_RPM_TO_RAD_S` / `CHASSIS_RAD_S_TO_RPM` 换算宏、π 常量。

#### `chassis_gyro.h` / `chassis_gyro.c` —— 陀螺偏航闭环（外环）

- 头文件：模式枚举 `OFF / RATE / SPIN / YAW_TARGET`、状态结构体 `chassis_gyro_state_t`、
  接口声明（`init/update/state/set_mode/set_spin_rate/set_yaw_target/reset/reset_correction`）。
- 实现（控制律 `wz_out = clamp(wz_ref + Kp·e + Ki·∫e, ±WZ_LIMIT)`，`e = wz_ref − wz_gyro`）：
  - 三种参考：用户 wz（RATE）/ 自旋速率（SPIN）/ 角度环 `Kp_angle·yaw_err`（YAW_TARGET）；
  - **斜坡段冻结积分与修正**：轮速还在爬坡时误差是物理上追不上的，不参与积分；
  - **I_BAND 带外冻结**：误差大于能力带（默认 2.0 rad/s）时沿用上一拍修正值；
  - **条件积分抗饱和**、输出限幅、零指令时积分按 τ=0.3 s 缓慢回零；
  - **静止零偏跟踪**：车确实静止（轮子停 + 三轴角速度小于阈值 + 用户 wz≈0）时调 `imu_track_bias()`；
  - IMU 掉线/非正常模式时彻底退回纯运动学（返回用户指令原值）。

#### `mecanum.h` / `mecanum.c` —— 麦轮运动学

- 头文件：轮序枚举 `FL/FR/BL/BR` 与轮数 4；`mecanum_inverse()` / `mecanum_forward()` 原型。
- 实现：
  - 逆解：`omega_i = (vx + S·ky_i·vy + S·kw_i·wz·L)/r`，其中 `L = 半轴距 + 半轮距`、
    `S = CHASSIS_ROLLER_SIGN`、`ky = {+1,-1,-1,+1}`、`kw = {-1,-1,+1,+1}`；
  - 正解：四轮 rad/s → 车体 `vx/vy/wz`，与逆解互为逆运算（里程计用）。

#### `motor.h` / `motor.c` —— TB6612 单路抽象

- 头文件：`motor_t`（PWM 定时器/通道、IN1/IN2 引脚、“正转”电平、当前占空比）与接口。
- 实现：`motor_init()` 启动 PWM 并清零；`motor_set_duty()` 把 `[-1000, +1000]` 千分比映射为
  方向引脚电平（正转用 `fwd_in1/fwd_in2`，反转按位取反）与 PWM 比较值（`duty/1000 · (Period+1)`）；
  `|duty| < 0.5` 视为 0 → IN1=IN2=0 滑行停止。正负号语义：正 = 沿车体 +x 向前滚。

#### `encoder.h` / `encoder.c` —— 编码器测速

- 头文件：`encoder_t`（定时器、上次计数、32 位累计、10 样本环形缓冲）。
- 实现：`encoder_start()` 启动编码器模式并**硬编码输入数字滤波**（`CCMR1` 的 IC1F/IC2F 写 8 =
  f_DTS/256 采样、连续 8 个一致，抑制 AB 线幽灵计数）；`encoder_update()` 每 1 ms 读 16 位计数器，
  无符号差截断为有符号增量（天然处理回绕）、累加进 32 位 `total`、维护滑动窗口；
  `encoder_speed_rpm()` 用窗口增量 / 窗口时长算平均转速。

#### `pid.h` / `pid.c` —— 位置式 PI(D)

- 头文件：`pid_t` 结构与控制律说明：
  `ff = kff·ref`、`e = ref − meas`、`I += e·dt`（条件积分）、`d = LPF(d(meas)/dt)`、
  `out = kp·e + ki·I − kd·d + ff`，再限幅。
- 实现：速度前馈、积分限幅、输出限幅、**条件积分抗饱和**（输出顶限幅且误差同向时不积分）、
  微分作用在**测量值**上并做一阶低通（`d_lpf_tau`，实测速度环 Kd 恒为 0）、首次计算从当前测量值起步。

#### `chassis_debug.h` / `chassis_debug.c` —— USART1 调试输出

- 头文件：只声明 `chassis_debug_poll()`。
- 实现：`CHASSIS_DEBUG_ENABLE=0` 时整个文件编译为空函数。开启后每 250 ms 发一帧：
  - **JustFloat 协议**：17 通道小端 float + 帧尾 `00 00 80 7F`，给 VOFA+ 画波形
    （ch0~3 目标轮速、ch4~7 实测轮速、ch8~11 占空比、ch12 里程计 yaw、ch13 陀螺 wz、
    ch14 陀螺积分 yaw、ch15 下发 wz、ch16 IMU 在线）；
  - **RawText 协议**：人眼可读的文本行，浮点用定点 `±d.ddd` 手工格式化（nano printf 没开 `%f`）。

#### `chassis_demo.h` / `chassis_demo.c` —— 演示状态机

- 头文件：`chassis_demo_init()` / `chassis_demo_poll()`。
- 实现：非阻塞状态机，上电 1.5 s 后自动循环：
  前进 0.5 m/s → 左移 0.4 m/s → 用里程计 yaw 闭环原地左转 90° → 后退 0.5 m/s →
  右移 0.4 m/s → 原地右转 90° → 小陀螺 3 s（`chassis_spin()`）→ 静止 3 s 后重来。
  只用于架空自检，`CHASSIS_DEMO_ENABLE=0` 时 `main.c` 不会调用。

#### `chassis_tune.h` / `chassis_tune.c` —— PID 整定实验

- 头文件：`chassis_tune_init()` / `chassis_tune_on_tick()` / `chassis_tune_poll()` / `chassis_tune_busy()`。
- 实现：`CHASSIS_TUNE_ENABLE=0` 时整个模块编译为空。开启后上电约 1 s 自动跑实验：
  1. **开环静态标定**：四轮同占空比扫档位，输出每档稳态转速表 → 得方向、启动死区、占空比-转速增益（定 KFF）；
  2. **闭环斜升扫参**：按跑车斜坡做 `0→+A→0→−A` 跟踪，1 kHz 采样存 RAM 后二进制回传（配 `tools/step_analysis.py`）；
  3. **开环转向目检**：+300‰/��300‰ 循环，人眼确认四轮转向；
  4. **惯性滑行自证**：驱动后断开 STBY/PWM，看编码器随惯性平滑衰减；
  5. **陀螺自旋实测**：静置→+自旋→停→−自旋四段，1 kHz 采样陀螺/里程计/轮速（配 `tools/spin_analysis.py`）。
- `chassis_on_tick` 在 `chassis_update()` 末尾每拍回调，保证采样与控制严格同频；
  实验结束后由主循环调 `chassis_tune_poll()` 把数据从 USART1 吐出去。

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

整条链 **1 kHz 执行一次**，全在 `chassis_update(dt)` 里；`imu_poll()` 在每拍开头按 2 ms 节拍采样，
陀螺外环 `chassis_gyro_update()` 输出的 `wz_eff` 参与逆解。

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

### 5.2 陀螺 / 小陀螺接口

| 函数 | 作用 |
|---|---|
| `chassis_spin(rad_s)` | 小陀螺：定速自旋（逆时针为正）；传 0 退出自旋回 RATE 模式 |
| `chassis_imu_online(void)` | IMU 是否在线（`imu_state()->healthy`） |
| `chassis_gyro_set_mode()` | `OFF / RATE / SPIN / YAW_TARGET`（见 `chassis_gyro.h`） |
| `chassis_gyro_set_spin_rate()` | SPIN 模式目标自旋速率 (rad/s) |
| `chassis_gyro_set_yaw_target()` | YAW_TARGET 模式目标角 (rad)，云台跟随预留 |
| `chassis_gyro_reset_correction()` | 只清外环修正量（停车/急停/看门狗时用） |

### 5.3 整定/调试接口 —— 正式固件里不要用

| 函数 | 唯一用途 |
|---|---|
| `chassis_set_mode()` | 切 `NORMAL` / `OPEN_DUTY` / `WHEEL_REF`，**绕过运动学与斜坡** |
| `chassis_set_open_duty()` | 四轮同占空比，开环标定 |
| `chassis_set_wheel_ref()` | 四轮直给 RPM，阶跃/斜坡实验 |
| `chassis_set_wheel_gain()` | 在线改某轮 Kp/Ki/Kacc（会清积分） |
| `chassis_driver_enable()` | `0` = STBY 拉低，电机高阻自由（手转测编码器用） |
| `chassis_reset_odometry()` | 里程计 x/y/yaw 归零（不动电机、不清 PID） |

> 这些接口存在的唯一理由是 `chassis_tune.c` 要用。模式一切换就清 PID 积分和加速度前馈状态，
> 正常跑车时 `s_mode` 恒为 `CHASSIS_MODE_NORMAL`。→ §6.8

## 6. 编译开关

全部集中在 `chassis_config.h`，改完重新编译即可。

| 宏 | 默认 | 什么时候改 |
|---|---|---|
| `CHASSIS_DEBUG_ENABLE` | `1` | 正式跑车改 `0` —— JustFloat 每帧 56 B @115200 约阻塞 4.9 ms，会吃掉控制节拍 |
| `CHASSIS_DEBUG_PROTOCOL` | `RAWTEXT` | VOFA+ 画波形用 JustFloat；想终端直接读文本用 RawText |
| `CHASSIS_TUNE_ENABLE` | `0` | 调 PID 时改 `1`，`main.c` 会改调 `chassis_tune_poll()` 并停用调试输出 |
| `CHASSIS_TUNE_EXPERIMENT` | `2` | 1 静态标定 / 2 斜升扫参 / 3 转向目检 / 4 惯性滑行 / 5 陀螺自旋（仅 TUNE 打开时有效） |
| `CHASSIS_DEMO_ENABLE` | `0` | 架空自检时改 `1`（自动跑六段演示 + 小陀螺） |
| `CHASSIS_DRIVER_DISABLE` | `0` | 只在两种情况下改 `1`：手转轮子测编码器 / 怀疑电机乱转。**跑车必须为 0** |
| `CHASSIS_GYRO_ENABLE` | `1` | IMU 没插也能编译；运行时自动退回纯运动学 |

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

**陀螺外环**：`Kp = 0.60`、`Ki = 4.00`、`I_MAX = 0.40`、`I_BAND = 2.0 rad/s`、
`CORR_MAX = 1.5 rad/s`、`WZ_LIMIT = 4.6 rad/s`；默认 `RATE` 模式；小陀螺默认 2.0 rad/s（落地实测抓地极限约 2.0~2.5）。

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
| 坐标系与符号约定、麦轮公式推导 | §4、§14（`docs/CHASSIS_LIBRARY.md`） |
| 各模块实现细节 | §6.1 ~ §6.8 |
| 参数逐项含义与调法 | §7、§11 |
| 硬件绑定表 `s_bind[]` | §8 |
| 上电校准全流程 | §10 |
| PID 整定方法 + 实测数据 + 现象对照表 | §11 |
| 已知限制与后续计划 | §13 |
| CAN 通信验证 | §15（另见 `UserLib/Comm/README.md`） |
| 上位机 Jetson `~/gimbal` | §16 |
| IMU（LSM6DSV16X）与陀螺偏航闭环 / 小陀螺 | §17；驱动本身见 `../IMU/README.md` |
| USART2 上位机控制链路（uart_link 协议） | §18；实现细节见 `../Comm/README.md` |
| 版本记录 | §19 |
| 整定数据解析脚本 | `../../tools/README.md` |
