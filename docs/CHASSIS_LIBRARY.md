# 底盘基础库设计文档（Chassis Library）

> 本文档是 `UserLib/Chassis/` 底盘基础库的**唯一权威说明**，
> 并同时收录 **CAN 通信验证**（§15）、**上位机正式工程 Jetson `~/gimbal`**（§16）、
> **IMU 与小陀螺**（§17）和 **USART2 控制链路协议**（§18）。
> 接线信息仍以项目根目录 `README.md` 为准，本文档不重复接线表，只引用；
> 各模块自己的用法速查在 `UserLib/*/README.md`（Chassis / Comm / IMU）。

- 适用平台：STM32F407VGTx + TB6612 ×2 + 亚博智能 L 型 520 编码器减速电机 ×4（麦克纳姆轮底盘）
- 电机参数：减速比 1:40，磁环 11 线，12V 供电，减速后空载约 300 RPM（±5%）
- 控制方式：四轮速度闭环（速度前馈 + 参考加速度前馈 + PI）+ 里程计，裸机 1 kHz 周期调度
- 整定状态：2026-10-18 完成四轮速度环实机整定（架空、满占空比），参数与实测指标见 §7 / §11
- IMU 链路：2026-10-19 打通并定型（硬件 I2C 400 kHz、SA0 焊死 GND=0x6A、20 分钟 soak 零错误），见 §17
- 上位机：Jetson Nano `~/gimbal`（C++17），控制云台 ZDT X42S ×2（UART），预留底盘链路（CAN / UART）

---

## 1. 概述

本库把底盘控制拆成「测量 / 控制 / 运动学」三层，目标是与 RM（RoboMaster）开源工程同等的工程化水平：

- **测量层**：TIM 编码器模式采样（输入带数字滤波），16 位计数器溢出拼接为 32 位累计，滑动窗口测速；
- **控制层**：车体速度指令 → 麦克纳姆逆解 → 每轮斜坡限幅 → 速度环（速度前馈 + 参考加速度前馈 + PI）→ TB6612 占空比；
- **运动学层**：逆解（指令分配）与正解（里程计）互为逆运算，符号统一。

## 2. 参考与借鉴

结构设计参考了 RM 开源底盘代码的成熟模式（measure/control 分离、轮速斜坡、速度环 PID、运动学独立模块）：

- RoboMaster 开发板 C 型示例工程：<https://github.com/RoboMaster/Development-Board-C-Example>
- rm-controls 框架：<https://github.com/rm-controls/rm_control>

本库为独立实现：不依赖 RTOS、不依赖裁判系统，坐标符号与 RM 约定保持一致，便于后续对接 RM 生态代码。

## 3. 目录结构

```
UserLib/Chassis/
├── chassis_config.h    # 全部可调参数（物理参数/限幅/PID/方向），唯一需要常改的文件
├── chassis_math.h      # 限幅、角度归一、斜坡等内联工具
├── motor.h/.c          # TB6612 单路电机抽象（占空比 ±1000）
├── encoder.h/.c        # 编码器抽象（输入滤波 + 累计计数 + 窗口测速）
├── pid.h/.c            # 位置式 PI(D)：速度前馈、条件积分抗饱和、微分低通
├── mecanum.h/.c        # 麦克纳姆运动学正解/逆解
├── chassis.h/.c        # 主模块：硬件绑定、1 kHz 控制循环、里程计
├── chassis_gyro.h/.c   # 陀螺偏航闭环：小陀螺自旋 / 云台跟随（见 §17）
├── chassis_demo.h/.c   # 演示状态机（可整文件删除）
├── chassis_debug.h/.c  # USART1 调试打印（可用 CHASSIS_DEBUG_ENABLE 关闭）
└── chassis_tune.h/.c   # PID 整定实验模块（默认关闭，见 §6.8）
UserLib/IMU/
├── README.md           # 驱动本身怎么用（接口速查/配置表/上电时序/坑）
├── imu_config.h        # IMU 配置表（I2C 地址/量程/采样率/轴映射）
├── imu.h/.c            # LSM6DSV16X 驱动：I2C 层 + 零偏标定 + 采样 + yaw 积分
├── lsm6dsv16x_reg.h/.c # ST 官方驱动（原样拷贝，不要手改）
tools/
├── step_analysis.py    # 整定数据解析：SET/STEP 二进制块 → 指标 + 曲线
└── spin_analysis.py    # 陀螺自旋实验（实验 5）解析：稳态误差/纹波/超调/漂移
test_data/
├── tune_runs/          # 2026-10-18 整定实测原始抓取（.log）与中间结果
└── spin_runs/          # 陀螺噪声/手转/自旋实验的原始串口抓取
docs/
└── CHASSIS_LIBRARY.md  # 本文档
```

## 4. 坐标系与符号约定（RM 约定）

```
            +x（车体前进方向）
             ^
             |
      FL(0)  |  FR(1)         轮子编号顺序：
             |                 0 = FL 左前, 1 = FR 右前
   +y <------+------->         2 = BL 左后, 3 = BR 右后
  （车体左侧）
             |
      BL(2)  |  BR(3)
```

| 量 | 约定 |
|---|---|
| `vx` | +x = 车体前进（m/s） |
| `vy` | +y = 车体左移（m/s） |
| `wz` | 俯视**逆时针**为正（rad/s） |
| 轮速 | 正 = 该轮沿 +x 向前滚动（RPM） |
| 里程计 yaw | 与 wz 同符号，逆时针为正，初始为 0 |

## 5. 架构与数据流

```
chassis_set_velocity(vx, vy, wz)
        │
        ▼
┌──────────────────────┐   ┌──────────────────────┐
│  mecanum_inverse()   │   │   encoder_update()   │
│  车体速度 → 四轮RPM   │   │  16bit→32bit + 窗口测速│
└──────────┬───────────┘   └──────────┬───────────┘
           │ 轮速限幅 ±250RPM         │ 方向归一(ENC_DIR)
           │ 斜坡 ±1500RPM/s          ▼
           ▼                   wheel_rpm_meas[4]
      wheel_rpm_ref[4]                │
           │                          │
           ▼                          ▼
      pid_calc()            mecanum_forward()
      duty = Kff·ref        实测轮速 → vx/vy/wz
           + Kacc·dref/dt          │
           + Kp·e + Ki·∫e           ▼
           （条件积分抗饱和）    里程计积分 (x, y, yaw)
           │
           ▼
      motor_set_duty()
      （±1000 → TB6612 IN1/IN2 + PWM CCR）
```

整条链以 1 kHz 执行一次：`chassis_poll()` 内部检测 `HAL_GetTick()` 的 1 ms 节拍，到点跑一拍。**主循环里不能再用 `HAL_Delay()` 长时间阻塞**，否则控制会掉拍。

> 整定期间（`CHASSIS_TUNE_ENABLE=1`）控制循环的末尾会多回调一次 `chassis_tune_on_tick()`，
> 采样与本拍严格同频；实验本身只改 `s_ref_override[]` / `s_open_duty`，不改控制律。

## 6. 模块说明

### 6.1 motor — TB6612 电机抽象

- `motor_init()`：绑定定时器/通道与 IN1/IN2 引脚，启动 PWM 并清零输出；
- `motor_set_duty(motor, duty)`：`duty ∈ [-1000, +1000]`（千分比）。
  - `duty > 0`：IN 电平 = `fwd_in1/fwd_in2`；
  - `duty < 0`：两电平互换；
  - `duty == 0`：IN1=IN2=0（**滑行停止**，非刹车）。

### 6.2 encoder — 编码器

- `encoder_start()`：以 TIM 编码器模式启动（x4 计数，1760 counts/轮圈），随后**强制写入输入数字滤波**；
- `encoder_update()`：每控制周期采样一次：
  - 增量 = 无符号差值截断为 int16，天然处理 16 位溢出回绕（前提：单周期增量 < 32767，本配置 1 kHz 下余量极大）；
  - 累计到 32 位 `total`，同时送入滑动窗口；
- `encoder_speed_rpm()`：窗口内增量求和 → RPM。窗口默认 10 ms，低转速分辨率约 3.4 RPM（60/1760/0.01）；窗口越大越平滑、延迟越大（改 `CHASSIS_SPEED_WINDOW_MS`）。

**输入数字滤波（重要）**

`encoder.c` 里 `ENC_INPUT_FILTER = 8`，`encoder_apply_filter()` 在启动后直接改 `CCMR1` 的
`IC1F/IC2F`（bits 7:4 / 15:12）：采样率 `f_DTS/256`、需连续多个一致才认边沿，约十几微秒的尖峰抑制。

- CubeMX 生成的 `tim.c` 里 `IC1Filter/IC2Filter` 默认是 **0（完全不滤）**，而电机 PWM 与换相噪声会灌进
  A/B 线，产生**幽灵计数**。2026-10-18 实测：FR 一路在 8000 个 1 ms 采样里出现 **146 次 >20 RPM/ms 的
  物理不可能跳变**（典型：占空比 −598‰ 时实测瞬间 +65 RPM），把速度环直接带偏、表现为"反向超调"；
- 把滤波改成 8 之后同条件跳变数 **归零**；最高转速下边沿间隔约 122 µs，远慢于滤波窗口，不影响测速精度；
- 之所以写在 `encoder.c` 而不是 `tim.c`：**CubeMX 重新生成代码不会覆盖**。（若你要用 CubeMX 图形界面改
  TIM 参数，也可以在 `.ioc` 里同步设好 `IC1Filter`，两者不冲突。）

### 6.3 pid — 位置式 PI(D)（前馈 + 抗饱和）

控制律（每个 1 ms 控制周期）：

```
ff   = Kff · ref                      速度前馈：维持该转速需要的占空比
dref = (ref - ref_prev) / dt          目标轮速的导数
I   += e · dt     （条件积分）         e = ref - meas
d    = LPF( d(meas)/dt , τ_d )        一阶低通后的测量微分
out  = Kff·ref + Kacc·dref + Kp·e + Ki·I − Kd·d
out  = clamp(out, ±OUT_MAX)
```

- **速度前馈 `Kff`**：承担"给多少力"，闭环只补偿负载、电压与个体差异。数值来自 §11 的开环静态标定表
  （`Kff ≈ 稳态占空比 / 稳态转速`，本车 3.50~3.95 ‰/RPM）；
- **参考加速度前馈 `Kacc`**：`Kacc·dref/dt` 只在斜坡段非零。斜坡加速时电机本来就吃满力矩，若让积分去
  "抢"这段力矩，斜坡结束时积分已经积了一大坨，必须靠超调慢慢吐出来。实测加入后：
  斜坡末端超调 17% → 5~8%，500 ms 稳态误差 +4.9 → +0.6 RPM；
  本车 `Kacc = 0.25 ‰/(RPM/s)`（斜坡 1500 RPM/s 时约等于额外给 375‰ 占空比）；
- **条件积分抗饱和**：输出已顶限幅、且误差还在往同方向推时，本次不吸收积分（替代原来的硬限幅，
  避免退饱和时积分要"慢慢吐"）；
- **微分**作用在测量值上并做一阶低通（`CHASSIS_PID_D_LPF_TAU_S`）。**本车 Kd 默认 0**：1 ms 采样 +
  10 ms 窗口的测速量化噪声约 ±2 RPM，Kd=0.08 会把它放大成数百‰ 的占空比尖刺，速度环基本不需要 D；
- 输出单位：占空比千分比；`i_max` 语义是**积分项**的限幅（RPM·s），不是占空比。

### 6.4 mecanum — 运动学

逆解（公式推导见附录 A）：

```
omega_i = ( vx + S·ky_i·vy + S·kw_i·wz·(lx+ly) ) / r
ky = {+1,-1,-1,+1}   kw = {-1,-1,+1,+1}   （FL FR BL BR）
S = CHASSIS_ROLLER_SIGN
```

正解为逆解的反推，两者共享同一套符号系数，里程计与指令不会互相矛盾。

### 6.5 chassis — 主模块

- 硬件绑定表 `s_bind[]`（见 §8），`fwd_in1/fwd_in2` 是该轮电机转向的**唯一**来源；
- `chassis_set_velocity()`：写入并限幅车体速度指令；
- `chassis_poll()`：1 kHz 控制循环（测量 → 逆解 → 斜坡 → PID → 输出 → 正解 → 里程计）；
- `chassis_state()`：返回只读状态（实测轮速、占空比、编码器累计、位姿），调试/上位机可读；
- 方向归一系数 `s_enc_dir[]` **只作用于测量值**（修编码器符号），电机转向只由 `s_bind[]` 的 fwd 电平决定，两者互不干扰。

除正常接口外，还提供一组**调试/整定专用**入口（正式跑车保持 `CHASSIS_MODE_NORMAL` 即可，对功能无影响）：

| 接口 | 作用 |
|---|---|
| `chassis_set_mode(mode)` | `NORMAL` 正常；`OPEN_DUTY` 四轮直接给同一占空比（绕过 PID）；`WHEEL_REF` 四轮直接给轮速目标（无斜坡）。切模式会清 PID 积分 |
| `chassis_set_open_duty(d)` | 模式 `OPEN_DUTY` 下的占空比，立即生效 |
| `chassis_set_wheel_ref(rpm[4])` | 模式 `WHEEL_REF` 下的四轮目标（RPM），不做斜坡 |
| `chassis_set_wheel_gain(i, kp, ki, kacc)` | 在线改某一轮增益（清积分），整定扫参用 |
| `chassis_driver_enable(0/1)` | 0 = 两块 TB6612 的 STBY 拉低、输出高阻、电机自由（手转测编码器/惯性滑行实验用） |

### 6.6 chassis_debug — 调试输出（RawData / JustFloat 双协议）

`CHASSIS_DEBUG_ENABLE=1` 时，通过 USART1（PA9/PA10，115200-8-N-1）每 250 ms 输出一帧。协议由 `CHASSIS_DEBUG_PROTOCOL` 选择：

**RawData 文本**（`CHASSIS_DEBUG_PROTO_RAWTEXT`，终端人眼直读）：

```
t=12345 | rpm FL  119 FR  119 BL  119 BR  119 | cnt 1760 3520 1760 3520 | yaw=12deg
```

含义：`t` 上电毫秒数；`rpm` 后是四轮**实测轮速 (RPM，已方向归一)**；`cnt` 后是四轮**编码器累计计数
（原始方向，不做归一）**——手转轮子可直接看它变不变；`yaw` 里程计偏航角。

> 整行约 110 字节，115200 下约 9.5 ms。`chassis_debug.c` 的发送超时已放到 50 ms，
> 若改回小值会被截断成半行。

**JustFloat 二进制**（`CHASSIS_DEBUG_PROTO_JUSTFLOAT`，VOFA+ 画波形用）：
帧格式 = 17 通道小端 float32 + 帧尾 `00 00 80 7F`（官方协议）。注意二进制模式下终端文本区看不到内容，属正常。

通道表（CH_COUNT = 17）：

| 通道 | 含义 | 通道 | 含义 |
|---|---|---|---|
| ch0~3 | 目标轮速 ref：FL FR BL BR (RPM) | ch8~11 | 输出占空比 duty：FL FR BL BR (‰) |
| ch4~7 | 实测轮速 meas：FL FR BL BR (RPM) | ch12 | 里程计 yaw (deg) |
| ch13 | 陀螺实测 wz (rad/s) | ch15 | 实际下发 wz_eff (rad/s) |
| ch14 | 陀螺积分 yaw (deg) | ch16 | IMU 在线 (0/1) |

VOFA+ 用法：新建数据引擎选 **JustFloat**（1.4.5 已内置插件，无需手动安装），
拖入 JustFloat 控件，把曲线勾成 `ch0`（FL 目标）vs `ch4`（FL 实测）这类对比即可整定 PID。

### 6.7 chassis_demo — 演示

上电 1.5 s 后自动循环执行：前进 → 左移 → 原地逆时针转 90°（里程计闭环）→ 后退 → 右移 → 原地顺时针转回 90° → **小陀螺自旋 3 s（陀螺闭环）** → 静止 3 s → 重复。
正式比赛代码中删除本文件及其调用即可；现在另有编译开关 `CHASSIS_DEMO_ENABLE`（默认 **0**，
`main.c` 里控制是否初始化/轮询），架空自检时置 1。

2026-10-18 用本演示做了整机验证（四轮架空的满占空比已整定版本）：前进段四轮 118.5~120.2 RPM
（目标 119.4）、整段 yaw 只漂 0.1°；平移段四轮精确到 ±95.5 RPM。

### 6.8 chassis_tune — PID 整定实验模块

只在整定期间使用，由 `CHASSIS_TUNE_ENABLE`（默认 **0**）整体开关；为 0 时本模块**不参与编译**，
对底盘功能零影响。打开后 `main.c` 不再调 `chassis_debug_poll()`，改调 `chassis_tune_poll()`。

| 开关 | 说明 |
|---|---|
| `CHASSIS_TUNE_ENABLE` | 1 = 整定模式，0 = 正常模式（默认） |
| `CHASSIS_TUNE_EXPERIMENT` | 1 静态标定 / 2 斜升扫参 / 3 转向目检 / 4 惯性滑行自证 / 5 陀螺自旋实测 |
| `CHASSIS_TUNE_STEP_RPM` | 实验 2 的目标转速幅值（默认 120） |
| `CHASSIS_DRIVER_DISABLE` | 1 = STBY 拉低、电机自由（手转测编码器用），正常必须 0 |

五个实验（上电静置 1 s 后自动开始，数据全部从 USART1 出）：

| 实验 | 内容 | 输出 |
|---|---|---|
| 1 静态标定 | 四轮同占空比按 50→1000‰ 逐档上升、再逐档负向，每档 400 ms，取最后 100 ms 均值 | 文本 `MAP,duty,rpm10_FL,FR,BL,BR` |
| 2 斜升扫参 | 按实际跑车斜坡（1500 RPM/s）做 0→+A→0→−A→0 跟踪；`s_gain_sets[]` 里一次排多组 Kp/Ki/Kacc 连续跑完 | 二进制 `SET …` + `STEP_BEGIN n=2000 ch=12` + 2000×24 B |
| 3 转向目检 | +300‰ / −300‰ 循环播放（每轮 5 s），人眼确认四轮是否同向 | 文本标记 `DIR +300 开始` |
| 4 滑行自证 | 驱动 500‰ 转 1.5 s → 然后 PWM 与 STBY 全断，看编码器是否随惯性平滑衰减 | 二进制同实验 2 |
| 5 陀螺自旋 | 静置 → +自旋速率 → 停 → −自旋速率，各 2 s（走正常模式，陀螺外环真参与） | 二进制 `STEP_BEGIN n=2000 ch=8` + 2000×16 B |

数据块格式（实验 2/4，小端 int16 × 12 通道 × n 样本）：

```
SET idx=0 kp=300 ki=4000 kacc=250 rpm=120     ← 本组参数（kp/ki/kacc 为 ×100/×1000 的定点整数）
STEP_BEGIN n=2000 ch=12 dt_us=1000
<n × 24 字节>                                  ← ch0~3=目标RPM ch4~7=实测RPM ch8~11=占空比‰
STEP_END cksum=<字节累加校验>
```

编排说明（都是踩过的坑）：

- 采样在 `chassis_update()` 末尾回调，**与本拍严格同频**，不存在采样抖动；
- 大块回传必须分片（`HAL_UART_Transmit` 的 Size 是 `uint16_t`，72000 字节会截断成 6464）；
- 实验跑完立刻回传，占空比保持 0，回传期间主循环阻塞、控制停摆但电机已静止，安全；
- 分析脚本 `tools/step_analysis.py <log>`：解析出每个 SET 块的稳态误差/超调/上升时间/噪声/占空比，
  支持一次抓取里多组参数对比。

## 7. 参数配置（`chassis_config.h`）

| 宏 | 当前值 | 说明 |
|---|---|---|
| `CHASSIS_WHEEL_RADIUS_M` | 0.040 | 轮径 80mm 实测 |
| `CHASSIS_HALF_WHEELBASE_M` | 0.1001 | 轴距 200.20mm / 2 |
| `CHASSIS_HALF_TRACK_M` | 0.120 | 轮距 240.00mm / 2 |
| `CHASSIS_ENCODER_CPR` | 1760 | 11线×4倍频×40减速比（标称值，可用 §10.1 手转法实测校正） |
| `CHASSIS_MAX_WHEEL_RPM` | **250** | 实测满占空比只有 260~296 RPM，指令 280 会顶限幅、闭环没余量。250 对应约 1.05 m/s |
| `CHASSIS_WHEEL_ACCEL_RPM_S` | 1500 | 轮速斜坡限幅（≈6.3 m/s²，架空时够用，落地后按抓地力再收） |
| `CHASSIS_MAX_VX_M_S` / `_VY_` | 1.0 | 车体线速度限幅。1.0 m/s = 238 RPM，已贴近 250 上限，**建议实战压到 0.8~0.9** |
| `CHASSIS_MAX_WZ_RAD_S` | 4.0 | 车体角速度限幅 |
| `CHASSIS_ROLLER_SIGN` | +1 | 辊筒朝向（见 §10.3） |
| `CHASSIS_CONTROL_PERIOD_S` | 0.001 | 控制周期 1 kHz |
| `CHASSIS_SPEED_WINDOW_MS` | 10 | 测速滑动窗口（分辨率 ≈3.4 RPM） |
| `CHASSIS_PID_OUT_MAX` | 1000 | 输出限幅（占空比满幅，四轮共用） |
| `CHASSIS_PID_D_LPF_TAU_S` | 0.005 | 测量微分的一阶低通时间常数，仅 Kd>0 时起作用 |
| `CHASSIS_ENC_DIR_FL/FR/BL/BR` | **+1 / −1 / +1 / −1** | 每轮编码器测量方向归一，2026-10-18 实测标定（见 §10.2） |
| `CHASSIS_DEBUG_ENABLE` | 1 | USART1 调试输出总开关，正式使用改 0 |
| `CHASSIS_DEBUG_PROTOCOL` | JustFloat | 调试协议：RawData 文本 / JustFloat 二进制（VOFA+ 数据引擎要对应） |
| `CHASSIS_DEMO_ENABLE` | 0 | 架空自检时置 1，上电自动跑 §12 的演示 |
| `CHASSIS_TUNE_ENABLE` / `_EXPERIMENT` | 0 / 2 | PID 整定实验开关与实验号（见 §6.8） |
| `CHASSIS_DRIVER_DISABLE` | 0 | 1 = STBY 拉低、电机自由，仅手转测编码器时用，正常必须 0 |

**速度环增益（2026-10-18 整定，实测数据见 §11）**

控制律 `duty = Kff·ref + Kacc·dref/dt + Kp·e + Ki·∫e − Kd·d(meas)/dt`

| 轮 | Kp | Ki | Kd | I_MAX | Kff (‰/RPM) | Kacc (‰/(RPM/s)) |
|---|---|---|---|---|---|---|
| FL 左前 | 3.0 | 40.0 | 0 | 20.0 | 3.85 | 0.25 |
| FR 右前 | 3.0 | 40.0 | 0 | 20.0 | 3.85 | 0.25 |
| BL 左后 | 3.0 | 40.0 | 0 | 20.0 | 3.95 | 0.25 |
| BR 右后 | 3.0 | 40.0 | 0 | 20.0 | 3.50 | 0.25 |

Kff 逐轮不同是**真实的机械差异**（BR 轮阻力最小：滑行衰减最慢、同样转速所需占空比最低），
不是标定误差，不要图省事改成同一个数。

## 8. 硬件绑定表（`chassis.c` 中 `s_bind[]`）

依据 2025-10-06 重新布线后的 README 接线（A=前轮、B=后轮）：

| 轮 | PWM | 方向引脚 | 编码器 |
|---|---|---|---|
| FL 左前 | TIM5_CH2 (PA1, LF_PWMA) | L_AIN1(PC4) / L_AIN2(PC5) | TIM3 (PA6/PA7) |
| FR 右前 | TIM5_CH4 (PA3, RF_PWMA) | R_AIN1(PE8) / R_AIN2(PE10) | TIM1 (PE9/PE11) |
| BL 左后 | TIM5_CH3 (PA2, LB_PWMB) | L_BIN1(PB0) / L_BIN2(PB1) | TIM4 (PD12/PD13) |
| BR 右后 | TIM9_CH1 (PE5, RB_PWMB) | R_BIN1(PE12) / R_BIN2(PE13) | TIM8 (PC6/PC7) |

其余引脚：L_STBY=PA4、R_STBY=PE7；调试串口 USART1(PA9/PA10)；编码器全部为
TIM_ENCODERMODE_TI12（4 倍频，CPR=1760 不变）。PWM 频率：TIM5≈11.7kHz、TIM9=10kHz。

`fwd_in1/fwd_in2` 为该轮「正转」时 TB6612 IN1/IN2 电平。**新线束沿用了旧标定惯例
（左轮 (0,1)、右轮 (1,0)）作为默认值，首次上电必须按 §10.2 重新标定**
——某轮实际转向反了，把该轮两个电平对调即可。

## 9. 集成方式

- `CMakeLists.txt`：库源文件已加入 `target_sources`，头文件路径已加入 `target_include_directories`（新增用户文件时照此追加，`chassis_tune.c` 已加）；
- `main.c`：USER CODE 区内两三处——
  ```c
  chassis_init();                     /* 底盘 + IMU（imu_init 在里面，约 1.8s） */
  uart_link_init();                   /* USART2 上位机链路（依赖 NVIC 勾选 USART2 中断） */
  #if CHASSIS_TUNE_ENABLE
      chassis_tune_init();            /* 整定模式 */
  #elif CHASSIS_DEMO_ENABLE
      chassis_demo_init();            /* 架空自检 */
  #endif
  ...
  while (1) {
      chassis_poll();                 /* 1 kHz 控制循环（内部调 imu_poll） */
      uart_link_poll();               /* 收命令帧 + 看门狗 + 100Hz 反馈帧 */
  #if CHASSIS_TUNE_ENABLE
      chassis_tune_poll();
  #else
      chassis_debug_poll();           /* 非阻塞：HAL_UART_Transmit_IT */
      #if CHASSIS_DEMO_ENABLE
          chassis_demo_poll();
      #endif
  #endif
  }
  ```
- **控制环调度**用 `HAL_GetTick()`（SysTick 1 kHz），不占中断；
  但**通信**占两个中断：`USART2_IRQHandler`（上位机链路逐字节收帧）与 `USART1_IRQHandler`
  （调试帧非阻塞发送）。CubeMX 里 **USART2 global interrupt 必须勾选**，否则收不到命令（见 §18.8）；
  USART1 的 NVIC 是在 `usart.c` 的 USER CODE 区手工打开的（CubeMX 没勾，重新生成时要保留）。
- 全部集成点都在 USER CODE 区内，CubeMX 重新生成代码不会破坏。

> 当前仓库状态下 `can_test_init()` / `can_test_poll()` 是**注释掉的**：`can_test` 的自诊断会往 USART1
> 打日志（1 s 一行），与底盘的调试帧抢同一个串口。要用 CAN 时把这两行取消注释即可（见 §15）。

## 10. 上电校准（务必按顺序做）

> 2026-10-18 已按本节完整跑过一遍，结论见 §10.5。**重新布线/换线后必须重做 §10.2**。

### 10.1 编码器 CPR 实测

1. 让驱动板不使能（`CHASSIS_DRIVER_DISABLE=1`，或直接断开电机电源），把某轮**手转整一圈**；
2. 看 §6.6 RawData 的 `cnt` 字段，或读 `chassis_state()->wheel_count[轮]` 的变化量；
3. 应为 1760 左右，把实测值写回 `CHASSIS_ENCODER_CPR`。

> 手转是可靠的：2026-10-18 用 §6.8 实验 4（惯性滑行）验证过编码器测的是真转——断开 STBY 与 PWM 后，
> 实测转速从 133 RPM 平滑衰减到 0（τ≈0.35 s），而不是瞬间归零。

### 10.2 每轮方向标定（电机方向 与 编码器方向 分开修）

1. **先跑 §6.8 实验 1（开环静态标定）**：四轮同占空比扫一遍，输出 `MAP,duty,rpm10_...`。
   看 `duty = +1000` 那一行：**读数符号与占空比同号**的轮子不用动；**反号**的把对应
   `CHASSIS_ENC_DIR_x` 取反。这一步完全自动，不用人眼；
2. **再跑 §6.8 实验 3（转向目检）**：+300‰ / −300‰ 循环播放，人眼确认四轮**物理转向**一致
   （正占空比都朝车头滚）。某轮反了 → 去 `chassis.c` 的 `s_bind[]` 对调该轮 `fwd_in1/fwd_in2`，
   **同时**把该轮 `ENC_DIR` 再取反一次（保持"测量符号 = 占空比符号"这个不变量）；
3. 正常状态：前进段四轮转速接近、占空比不饱和、yaw 基本不变。

> 为什么必须分开修：电机方向（TB6612 接线）和编码器方向（编码器 AB 线）是两套独立线缆，
> 可能只反一个。若只把电机线反了而编码器没反，闭环会把该轮推到全速——就是「一侧明显更快」现象的来源。
> **编码器反号时的典型症状是：实测为负 + 该轮占空比顶在 ±1000 飞转。**

### 10.3 运动学符号（`CHASSIS_ROLLER_SIGN`）

用 §12 的演示（`CHASSIS_DEMO_ENABLE=1`）分别验证前进 / 左移 / 旋转三段：

- 前进跑偏（车身斜着走）→ 先回 10.2 查方向；
- 前进直、**左移变成右移**（或旋转方向反了）→ 把 `CHASSIS_ROLLER_SIGN` 取反，重测；
- 旋转方向反 → 同上（该宏同时翻转 vy 与 wz 两项符号）。

### 10.4 死区与最低占空比

TB6612 + 减速电机的启动死区实测（2026-10-18，架空）：

- **正向死区约 80~100‰**：+50‰ 四轮都不转，+100‰ 才勉强起步（8~12 RPM）；
- **反向死区更小**，约 50‰ 就有 6 RPM（正反向摩擦不对称）；
- 死区由速度环积分自动补偿，实测指令低到 **25 RPM 仍能稳在 ±1 RPM**（占空比约 100‰，噪声 1.5 RPM），
  不需要额外的死区补偿；再往低（<15 RPM）会掉进死区里，可能出现走走停停的极限环——**这一段尚未实测**，
  只是从死区位置推的结论。

### 10.5 2026-10-18 标定结论（重新布线后的首次完整标定）

| 项 | 结论 |
|---|---|
| 编码器方向 | FL **+1** / FR **−1** / BL **+1** / BR **−1**（右两轮 AB 线接反，只修测量符号） |
| 电机方向（`s_bind[]` fwd 电平） | **(0,1)/(1,0)/(0,1)/(1,0)** 保持不变，人眼确认四轮正向一致 |
| 编码器输入滤波 | 必须开（`ENC_INPUT_FILTER=8`），否则 FR 一路大量幽灵计数（见 §6.2） |
| 占空比-转速增益 | 满占空比 260~296 RPM，低档斜率约 0.28 RPM/‰，**逐轮差异明显**（BR 最快） |
| 启动死区 | 正向约 80~100‰、反向约 50‰ |
| 机械阻力 | BR 最小（滑行 τ≈0.7 s）、FL 最大（τ≈0.35 s），是 Kff 逐轮不同的物理来源 |

## 11. PID 整定

> 当前参数见 §7 的增益表；本节记录**方法**与**2026-10-18 的实测结论**。
> 四轮参数**相互独立**，先四轮同参整定，再按 Kff 逐轮微调。

### 11.1 控制律与各参数职责

`duty = Kff·ref + Kacc·dref/dt + Kp·e + Ki·∫e − Kd·d(meas)/dt`

| 参数 | 管什么 | 怎么定 |
|---|---|---|
| `Kff` | 承担全部"稳态出力"，闭环只做补偿 | 开环静态标定表直接算：`Kff = 稳态占空比 / 稳态转速` |
| `Kacc` | 斜坡段的加速力矩 | 扫参定：给太小则斜坡末端超调大，给太大则反向冲过 |
| `Kp` | 带宽/相位裕度 | 有前馈后 Kp 只要能纠正残余误差即可，过大反而制造占空比尖刺 |
| `Ki` | 稳态误差收敛速度 | 决定误差在多久内被吃掉（本车 ≈0.2~0.3 s），过大会在斜坡末段顶出超调 |
| `Kd` | — | **速度环不要用**：测速量化噪声 ±2 RPM，微分会把它放大成几百‰ 的尖刺 |

### 11.2 整定流程（可复现）

1. **静态标定**（实验 1）：得到逐轮占空比-转速曲线 → 算 `Kff`（取工作点附近，本车 120 RPM 处）；
   顺便确认方向（§10.2）与死区；
2. **定 Kacc**（实验 2）：把 Kp/Ki 固定，`s_gain_sets[]` 里扫 `{0, 0.08, 0.16, 0.25}` 一次烧录跑完，
   看"斜坡结束后的超调"与"500 ms 稳态误差"，取两者都小的；
3. **定 Kp/Ki**（实验 2）：固定 Kacc，扫 `Kp×Ki` 组合，目标是稳态误差 <1 RPM、超调 <10%；
4. **全速域验证**：把目标幅值改成 25 / 60 / 120 / 240 RPM 各跑一轮，确认低速（死区附近）
   和高速（接近占空比上限）都不发散；
5. 数据用 `tools/step_analysis.py` 解析，多组参数并排比较。

### 11.3 2026-10-18 实测结果（四轮架空、满占空比、稳压供电）

| 目标 | 四轮稳态误差 | 超调 | 测速噪声 (std) |
|---|---|---|---|
| 25 RPM | −1.1 ~ +0.0 | 8% | 1.5 |
| 60 RPM | −0.3 ~ +0.1 | 8~13% | 1.5 |
| 120 RPM | −0.6 ~ +0.9 | 5~11% | 1.7~2.0 |
| 240 RPM | −5.4 ~ +3.2 | 4~8% | 1.5~2.8 |

- 参考轨迹是**实际跑车的斜坡**（1500 RPM/s，即 0→120 RPM 用 80 ms），不是理想阶跃——
  理想阶跃会把 Kp 的冲击放大到不真实的程度；
- 240 RPM 那一档的 2% 误差是因为占空比已到 780~940‰，闭环余量变小；
- **280 RPM 达不到**：占空比顶在 1000‰ 仍只有 252~297 RPM，属物理极限，所以 `CHASSIS_MAX_WHEEL_RPM` 改成 250。

### 11.4 现象 → 对策

| 现象 | 原因 | 对策 |
|---|---|---|
| 稳态差一截、占空比没饱和 | 前馈偏小 / 积分太慢 | 加大该轮 `KFF`；或加大 `KI` |
| 占空比短时冲到近 1000 再回落 | 缺加速度前馈，积分在坡度里"抢"力矩 | 加大 `KACC`（0.05 一档试） |
| 斜坡结束超调明显、回落慢 | `KI` 偏大或 `KACC` 偏小 | 先加 `KACC`，再考虑降 `KI` |
| 转速随机尖刺、占空比抖 | 编码器幽灵计数（滤波没开/线接触不良） | 先查 §6.2 的 `ENC_INPUT_FILTER`，再查接线 |
| 某轮反向时冲得特别猛 | 正反向摩擦不对称（反向死区更小） | 该轮 `KFF` 略降，让积分慢慢补 |
| 占空比顶 1000 但转速上不去 | 物理极限（负载/供电/堵转） | 调参无用，查供电与机械 |

### 11.5 整定前提

- 用**满电电池或稳压电源**：实测长时间运行占空比会随电池电压漂移（电池掉压 → 同样转速要更大占空比）；
- 架空整定得到的是**无载**参数，落地后负载更大、死区更明显；建议落地后低速（0.2~0.3 m/s）复测 yaw 漂移，
  必要时把 `KFF` 整体上调 5~10% 让积分少干活。

## 12. 演示程序说明

- 由 `CHASSIS_DEMO_ENABLE` 控制（默认 0）；置 1 后上电 1.5 s 开始（留出把车放到地上的时间）；
- 两个旋转段用**里程计 yaw** 闭环，转满 90° 即进入下一段——同时验证了运动学正解与编码器方向；
- 若某段行为与预期相反，按 §10 顺序排查，不要改演示逻辑本身。

## 13. 已知限制与后续计划

- **角度仍是"各算各的"**：里程计 yaw 来自轮速积分（打滑会累积），陀螺 yaw 来自 gz 积分（会漂移），
  两者没有融合。陀螺外环只用 wz（速率），不用角度。要长时角度保持（云台跟随），
  需要加"不打滑时用里程计校正陀螺 yaw"的互补滤波——取舍分析见 §17.6；
- **自旋速率受抓地限制**：实测地砖上约 2.0~2.5 rad/s，`CHASSIS_SPIN_RATE_RAD_S` 已按此设为 2.0（§17.6）；
- 未做电流环（TB6612 无电流采样），堵转保护靠限幅 + 后续可加堵转检测；
- 电机制动未用短路刹车（停止为滑行），如需刹车可扩展 `motor_brake()`；
- **落地闭环指标未测**：小陀螺的稳态误差/纹波/超调还缺一次干净的落地实测（首轮被线拽/车卡住，见 §17.6）。
- **满速上限 250 RPM**（≈1.05 m/s）：实测占空比 1000‰ 只有 260~296 RPM，`CHASSIS_MAX_VX` 给 1.0 已贴近上限；
- **四轮参数不能通用**：Kff 3.50~3.95、机械阻力差异明显（BR 滑行衰减最慢），逐轮差异是物理事实；
- **正反向不对称**：反向死区（≈50‰）明显小于正向（≈80~100‰），反向低速时积分补得多一些，属正常；
- **当前参数是架空整定的**：落地后负载增大，Kp/Ki 可能偏软、Kff 可能偏小，建议按 §11.5 复测；
- **编码器 A/B 线仍建议复查**：滤波已把 FR 那路的幽灵计数压到 0，但 100+ 次/8000 采样的原始跳变
  说明该路存在噪声/接触问题，滤波是软件兜底而非修好了硬件。

## 14. 附录 A：运动学公式推导

设轮 i 位于 (x_i, y_i)（车体坐标系，前+x、左+y），车体速度 (vx, vy, wz)，轮半径 r：

轮毂速度：`u_i = (vx - wz·y_i, vy + wz·x_i)`

FL 轮辊筒轴方向 ê=(1,1)/√2（FR/BL 为 (1,-1)/√2，标准安装），由「接触点沿辊筒轴无滑动」得：

```
u_i·ê_i = s_i·(ex·ê_i)   =>   s_i = √2·(u_i·ê_i)   （s_i = r·omega_i）
```

代入四轮坐标 (±ly, ±lx) 并整理：

| 轮 | omega_i·r |
|---|---|
| FL | vx + vy − wz·(lx+ly) |
| FR | vx − vy − wz·(lx+ly) |
| BL | vx − vy + wz·(lx+ly) |
| BR | vx + vy + wz·(lx+ly) |

正解由线性组合反推：

```
vx = r/4·(ωfl + ωfr + ωbl + ωbr)
vy = r/4·(ωfl − ωfr − ωbl + ωbr)
wz = r/(4(lx+ly))·(−ωfl − ωfr + ωbl + ωbr)
```

## 15. CAN 通信与点灯验证（`UserLib/Comm/can_test`）

> 帧格式、滤波器、位时序等**实现细节**以 `docs/CAN_TEST.md` 为唯一权威说明；
> 本节只记录**模块定位、配置开关、实测结论与排障方法**，不重复前者的内容。

### 15.1 模块定位

`can_test` 是**最小可验证**的 CAN 收发固件，回答一个问题：**F407 和上位机（Jetson）之间到底通不通**。

| 能力 | 说明 |
|---|---|
| ① 心跳 | 每 500 ms 发一帧 `0x7F0`，数据 `[序号, 0xCA, 0xFE, 已收帧数低8位]` |
| ② 回显 | 收到任何帧原样回一帧，标准帧 ID + 1；扩展帧统一回 `0x123` |
| ③ 点灯 | 收到 `0x200` 控制帧 → 操作 PB2 板载 LED → 回 `0x201` 应答 |
| ④ 自诊断 | 每 1 s 经 USART1（115200 8N1）打印速率、收发计数、TEC/REC/LEC |
| ⑤ 自检 | silent-loopback 模式下自发自收，**不依赖任何适配器**即可验证协议链路 |

### 15.2 帧格式约定

标准帧（11 位）用于 Jetson ↔ F407；扩展帧（29 位）留给云台 ZDT 步进，两者天然隔离不冲突。

| ID | 方向 | 内容 | 数据 |
|---|---|---|---|
| `0x7F0` | F407 → 上位机 | 心跳 | `[seq, 0xCA, 0xFE, rx_count & 0xFF]` |
| `0x200` | 上位机 → F407 | LED 控制 | `[cmd]`，cmd：`0`=灭 `1`=亮 `2`=翻转（无数据按翻转处理） |
| `0x201` | F407 → 上位机 | LED 应答 | `[cmd, 当前状态, rx_count & 0xFF, 0xA5]` |
| 收到的 ID + 1 | F407 → 上位机 | 回显 | 原样转发 |

未知 `cmd` 不改灯但仍回 `0x201`，方便上位机确认链路活着；`0xA5` 是应答帧专属魔数。

### 15.3 配置开关（`can_test_config.h`）

**这是全模块唯一需要动的文件。** 按用途分成三档，切换时只改这几个宏：

| 用途 | `LOOPBACK_SELFTEST` | `HEARTBEAT_ENABLE` | `AUTO_RETRANSMIT` | `ECHO_ENABLE` |
|---|---|---|---|---|
| **正常联调**（接真适配器） | `0` | `1` | `1` | `1` |
| **诊断**（对面不响应，看错误计数器） | `0` | `0` | `0` | `0` |
| **自检**（无适配器，验证协议逻辑） | `1` | `0` | `0` | `0` |

⚠️ **自检档必须关回显**：回环模式下 `0x201` 会被回显成 `0x202`、再回显成 `0x203`……无限套娃把三个发送邮箱塞爆，`err` 会变成 `0x00200000`（`HAL_CAN_ERROR_PARAM` = 邮箱满）。

`CANTEST_LED_ACTIVE_HIGH`：PB2 的极性，`1` = 写高电平灯亮。实测本板为 `1`（自检时灯正常闪）。

### 15.4 位时序

CAN1 挂 APB1 = 42 MHz。固定 **14 TQ**（`1 + 11 + 2`）时：

```
位速率 = 42 MHz / 14 / Prescaler = 3 Mbps / Prescaler
```

所以换速率**只改 `CANTEST_PRESCALER` 一个数**，采样点恒为 85.714%：

| 速率 | Prescaler | 速率 | Prescaler |
|---|---|---|---|
| **1 Mbps** | **3** | 125 kbps | 24 |
| 500 kbps | 6 | 100 kbps | 30 |
| 250 kbps | 12 | 50 kbps | 60 |

例外：`400 kbps` 需要 15 TQ（Prescaler 7 / BS1 12 / BS2 2，采样点 86.7%）；
`800 kbps` 在 42 MHz 下**无法精确生成**（需 `Prescaler × TQ = 52.5`），建议改用 750 k 或 1 M。

**`.ioc` 也必须配**：CubeMX 的 `CAN1` 面板里要填 `Prescaler=3` / `BS1=11 Times` / `BS2=2 Times` / `SJW=1`。
本工程早期只有 `CalculateBaudRate=875000` 这类自动算出来的键，没有真正生效的 `Prescaler/BS1/BS2`，
重新生成会退回 **875 kbps / 一位仅 3 个 TQ / 采样点 66.7%** 的废配置（物理上不可能通信）。
现已补进 `.ioc`，`IPParameters` 中含 `BS1,Prescaler,BS2`，判断是否修好的唯一标准就是这一项。

### 15.5 四个踩过的坑

| # | 坑 | 现象 | 解法 |
|---|---|---|---|
| 1 | **滤波器不配** | 一帧都收不到（HAL 不会自动给默认滤波器） | 掩码全 0 的全通过滤波器，占第 0 组 |
| 2 | **位时序被 CubeMX 覆盖** | 能编译能下载，但物理上通不了信 | 运行时强制重配 + 把真值写进 `.ioc` |
| 3 | **自己发的帧收不到** | `bus.recv()` 返回 `None`，但 TX 计数在涨 | SocketCAN 的 `CAN_RAW_RECV_OWN_MSGS` 默认关闭；python-can 需 `receive_own_messages=True`，或另开 `candump` 观察 |
| 4 | **`err=0x00200000`** | `tx` 卡住不涨 | 这是 `HAL_CAN_ERROR_PARAM`，在 `AddTxMessage` 里的含义是**三个发送邮箱全满** → 对面没 ACK，`AutoRetransmission` 让帧无限重传 |

### 15.6 排障方法：用 TEC / REC / LEC 定位故障层

HAL 的 `ErrorCode` 只反映**软件调用层**的错误，看不到总线层的错误计数器。
`can_test` 直接从 `CAN_ESR` 寄存器反读，串口打印：

```
CAN 1000000bps | tx=45 rx=44 err=0x00000000 bof=0 | TEC=45 REC=0 LEC=none
```

**这张表把"通不通"这件事拆成了可判定的三档**，是整个排障过程最有价值的部分：

| 读数 | 结论 | 下一步 |
|---|---|---|
| `REC = 0` 且 `LEC = none` | **完全听不到总线**（RX 引脚一直是隐性） | 查物理层：收发器供电、CAN_H/CAN_L、终端电阻 |
| `REC > 0` 或 `LEC = STUFF/FORM/CRC` | **听到了波形但解不出来** | 位速率 / 采样点不一致 |
| `LEC = ACK` 且 `TEC` 上涨 | 发得出去但**没人应答 ACK** | 对面没上电、线断、或对面收发器没工作 |
| `TEC = 0` + `tx` 不涨 + 总线上量不到电平变化 | **设备侧压根没往外发** | 适配器/驱动的问题，与 MCU 无关 |

### 15.7 实测结论

**F407 侧全链路已验证合格**（silent-loopback 自检，`tx`/`rx` 每秒同步 +2、`TEC=0 REC=0 LEC=none`）：

滤波器配置 → RX FIFO 轮询 → `can_test_handle_rx()` 分发 → LED 写入 → `0x201` 应答发送，全部正确。

物理层实测（万用表）：

| 测点 | 实测值 | 判定 |
|---|---|---|
| TJA1050 `VCC` (pin 3) | 5 V | ✅ 收发器有电 |
| `CANH` / `CANL`（发帧时） | 3.10 V / 1.704 V | ✅ 差分显性，TJA1050 输出正常 |
| `CANH ↔ CANL`（断电，接上适配器） | 118.9 Ω | ✅ 板子 120Ω 终端在，导线通 |

原理图核对：`CAN1_RX → PB8`、`CAN1_TX → PB9`、收发器 `TJA1050T`、网络名 `CANH/CANL`，与固件配置一致，
且与 I2C1（PB6/PB7）无冲突。

**唯一卡点是上位机侧的 CAN 适配器**，与底盘代码无关。选型教训见 §15.8。

### 15.8 适配器选型（重要，避免重复踩坑）

| 型号 | Linux 支持 | 结论 |
|---|---|---|
| **CANable 2.0 / candleLight**（STM32F072/G431） | 内核自带 `gs_usb`（`CONFIG_CAN_GS_USB=m`）或 `slcan`（`CONFIG_CAN_SLCAN=m`），插上即 `can0` | ✅ **推荐**，无需任何厂商驱动 |
| KCANBus-U-I（鲲弘）`0483:000D`（KCAN 原生模式） | 厂商驱动表只有 `0c72:xxxx`，只能 `new_id` 强绑；强绑后 USB 初始化命令全部超时（`err -110`），netdev 能注册但**一帧都发不出去**（`TEC` 恒 0，总线无电平变化） | ❌ 不可用 |
| 同上，切 `AD50:60C4`（SLCAN 模式） | 设备枚举为 CDC+HID 复合设备，但**不响应标准 LAWICEL 指令**（`V`/`C`/`S8`/`O` 均零字节返回），`slcand` 无法驱动 | ❌ 不可用 |

**Jetson 侧现状**：内核 4.9.253-tegra 已含 `CAN_GS_USB` / `CAN_SLCAN` / `CAN_PEAK_USB` / `CAN_8DEV_USB` 等模块；
但 **mttcan 只支持 Tegra194**（Xavier/Orin），Jetson Nano 是 t210，设备树无 CAN 节点，**无法使用板载 CAN 控制器**，
必须外接 USB 适配器。

⚠️ **遗留隐患**：`99-kcan-usbpro.rules` + `kcan-usbpro.service` 会对任意 `0483:000d` 设备执行 `new_id` 强绑并重启服务。
该规则只在用鲲弘设备时才需要，长期建议 `systemctl disable --now kcan-usbpro.service`，避免误抓设备。

> 注：CAN FD 在 Jetson 的内核 4.9 上用不了（`gs_usb` 是 2016 年的老版本，只支持经典 CAN）。
> 本项目只需要 1 Mbps 经典帧，不受影响，选型时不必为 FD 多花钱。

> **状态**：受适配器问题影响，CAN 通信方向已**整体暂停**；
> 上位机工程的定位、对接点与恢复计划见 §16。

## 16. 上位机正式工程：Jetson `~/gimbal`

> 本节确立本项目的**上位机正式工程**——跑在 Jetson Nano 上的 `~/gimbal`，并说明它与本底盘库的对接关系。
> 云台本体、通信、遥控全部由它负责；底盘只预留接口，见 §16.9。
> **CAN 通信方向当前暂停**（适配器在 Linux 下不可用，见 §15.8），恢复计划见 §16.12。

### 16.1 定位

| 项 | 值 |
|---|---|
| 位置 | Jetson Nano：`~/gimbal`；笔记本同源副本：`~/gimbal/`（**两边 21 个源文件 md5 一致**） |
| 平台 | L4T R32.6.1 / Ubuntu 20.04 / kernel 4.9.253-tegra / aarch64 |
| 语言 / 构建 | C++17（g++ 8.4.0）+ CMake 3.16.3 |
| 形态 | 单进程守护式，无 RTOS、无 ROS 依赖 |
| 控制对象 | 云台 ZDT X42S 闭环步进 ×2（**yaw = 2 / pitch = 1**），UART 115200 8N1 |
| 控制入口 | UDP 40001（PC 鼠标客户端）+ stdin（本地调试） |
| 底盘部分 | **仅预留接口**，协议未落地（§16.9 / §16.10） |
| 外部依赖 | **零第三方库** —— 仅 C++ 标准库 + Linux 系统调用（termios / SocketCAN / BSD socket） |

**为什么不用 Python / ROS**：

- §9 的 `test_server.py` 已证明链路可行，但 100 Hz 控制循环 + ZDT 12 ms 命令间隔在 Python 下时序抖动大；
- Jetson 上虽已装 ROS 2 Foxy（§8），但单进程 CMake 工程部署更简单、启动更快、无 DDS 开销；
- 板端零第三方依赖，换硬件平台时只需交叉编译，不牵扯库版本。

### 16.2 工程规模与目录结构

**21 个源文件（12 头 + 9 源），3186 行**；外加 `CMakeLists.txt`(48 行) 与 `README.md`(266 行)，共 23 个文件。

```
~/gimbal/
├── CMakeLists.txt
├── README.md
├── include/gimbal/
│   ├── protocol/     zdt_codes.hpp        zdt_frame.hpp
│   ├── transport/    itransport.hpp       uart_transport.hpp
│   │                 can_transport.hpp    fake_transport.hpp
│   ├── device/       zdt_motor.hpp
│   ├── gimbal/       axis.hpp             gimbal.hpp
│   └── link/         control_link.hpp     udp_control_link.hpp
│                     chassis_link.hpp
└── src/
    ├── main.cpp
    ├── protocol/     zdt_frame.cpp
    ├── transport/    itransport.cpp       uart_transport.cpp
    │                 can_transport.cpp
    ├── device/       zdt_motor.cpp
    ├── gimbal/       axis.cpp             gimbal.cpp
    └── link/         udp_control_link.cpp
```

行数分布（判断改动影响面用）：

| 文件 | 行数 | 文件 | 行数 |
|---|---|---|---|
| `src/main.cpp` | 575 | `include/.../device/zdt_motor.hpp` | 121 |
| `include/.../link/control_link.hpp` | 224 | `src/gimbal/gimbal.cpp` | 111 |
| `src/device/zdt_motor.cpp` | 221 | `include/.../protocol/zdt_codes.hpp` | 104 |
| `include/.../transport/fake_transport.hpp` | 204 | `include/.../protocol/zdt_frame.hpp` | 97 |
| `src/protocol/zdt_frame.cpp` | 200 | `include/.../gimbal/axis.hpp` | 94 |
| `src/link/udp_control_link.cpp` | 175 | `include/.../transport/itransport.hpp` | 90 |
| `src/transport/can_transport.cpp` | 172 | `include/.../gimbal/gimbal.hpp` | 89 |
| `src/transport/uart_transport.cpp` | 167 | `src/transport/itransport.cpp` | 77 |
| `src/gimbal/axis.cpp` | 148 | `include/.../link/udp_control_link.hpp` | 75 |
| `include/.../link/chassis_link.hpp` | 130 | `include/.../transport/can_transport.hpp` | 61 |
| | | `include/.../transport/uart_transport.hpp` | 51 |

### 16.3 分层架构与数据流

```
┌──────────────────────────────────────────────────────────────┐
│ main.cpp      100 Hz 控制循环 · stdin 交互 · UDP 收包/发遥测   │
├──────────────────────────────────────────────────────────────┤
│ Gimbal        双轴协调 · 两级看门狗 · 传输层 poll 去重         │
├──────────────────────────────────────────────────────────────┤
│ Axis          角速度斜坡 · 死区 · 软限位 · 回中 · 角度估计     │
├──────────────────────────────────────────────────────────────┤
│ ZdtMotor      物理量 ↔ ZDT 帧 · 遥测轮询 · 状态解析 · 单位换算 │
├──────────────────────────────────────────────────────────────┤
│ ITransport    发送节流队列 · 流式解析 · 按地址路由的收帧队列   │
│   ├── UartTransport   当前使用（CH340 / ttyTHS1），热插拔重连  │
│   ├── CanTransport    扩展帧拆包重组，**未上机验证**           │
│   └── FakeTransport   纯软件模拟电机，--dry / --selftest 用    │
└──────────────────────────────────────────────────────────────┘
```

**换总线只换一个实现类，上面四层一行不改** —— 这是分层最主要的收益。

单拍数据流（`Gimbal::step(dt)`，dt 默认 10 ms）：

```
操作端 UDP 包 ─┐
               ├─► Gimbal::setRate(yaw_dps, pitch_dps)
本地 stdin ────┘         │
                         ▼
                   Axis::setRate()  ── 限幅到 max_rate_dps，重置 since_cmd_s_
                         │
                         ▼
                   Axis::update(dt) ── ① 看门狗判断
                                        ② 斜坡逼近（rate_accel_dps2）
                                        ③ 死区吸收（rate_deadband_dps）
                                        ④ 软限位钳制 + 角度积分
                         │
                         ▼
        输出 °/s ──换算──► 电机 RPM（× gear ÷ 6）
                         │
                         ▼
                   ZdtMotor::setSpeedRpm() ──► ITransport::send()
                                                    │  12 ms 节流队列
                                                    ▼
                                              UART / CAN ──► 驱动板

收方向：UART 字节 ─► zdt::Parser 流式切帧 ─► 按 Addr 路由
        ─► ZdtMotor 解析（速度/角度/状态/电压/温度）─► Axis / Gimbal 查询
        ─► main 组装 ControlTelemetry ─► UDP 下行 20 Hz
```

`Gimbal` 只对**去重后**的传输层调 `poll()`（两轴共一根总线时只有一个），避免同一串口被 poll 两次。

### 16.4 ZDT 电机协议要点（Emm5.0）

依据《ZDT_X42S 第二代闭环步进电机用户手册 V1.0.5_260527》。

**UART 帧格式**

```
主机 → 电机:  Addr | 功能码 | 命令数据 | 校验码
电机 → 主机:  Addr | 功能码 | 返回数据 | 校验码
校验码出厂固定 0x6B（可改为 XOR / CRC-8 / Modbus / DMX512）
```

**用到的功能码**（`protocol/zdt_codes.hpp`）

| 码 | 名称 | 作用 | 码 | 名称 | 作用 |
|---|---|---|---|---|---|
| `F3` | Enable | 使能 / 失能 | `36` | GetPos | 实时位置回读 |
| `F6` | Speed | 速度模式 | `3A` | GetStatus | 状态标志位 |
| `FD` | Position | 位置模式 | `39` | GetTemp | 驱动温度 |
| `FE 98` | EStop | 立即停止 | `24` | GetVbus | 母线电压 mV |
| `FF 66` | SyncMove | 多机同步触发 | `42` | GetConfig | 驱动参数（33 B） |
| `35` | GetSpeed | 实时转速回读 | `0A` | ZeroPos | 当前位置清零 |

**`0x3A` 状态位**：`bit0` 使能 · `bit1` 到位 · `bit2` 堵转 · `bit3` 堵转保护 · `bit4` 左限位 · `bit5` 右限位 · `bit7` 掉电。

**⚠️ Emm 与 X 两套固件的单位不同**，混用必然错位：

| | **Emm5.0**（本工程） | X |
|---|---|---|
| 位置命令 4 字节 | **脉冲数** | 0.1° 角度 |
| 速度单位 | RPM | 0.1 RPM |
| 位置回读 `0x36` | **0–65535 = 一圈 0–360°**（单圈，拿不到多圈绝对值） | 角度 / 10 |
| 位置模式帧长 | 13 字节 | 不同 |

本工程默认走**速度模式**做鼠标控制，天然绕开位置换算；位置模式只实现了 Emm 分支。

**⚠️ 角度估计的诚实边界**：因为 `0x36` 只有单圈值，`Axis::estAngleDeg()` 是**转速积分出来的软件估计**，会缓慢漂移。
真正的绝对保护要靠驱动板的限位开关输入（状态位 `bit4/bit5`），或定期 `zeroHere()` 在机械零点重新对齐。

### 16.5 六条关键设计决策（每条都来自踩坑）

#### 16.5.1 命令节流 12 ms —— 防 ZDT 粘包

手册 §3.5.2 明确要求「每条命令发送完后要延时几毫秒，防止命令之间粘包失效」。
实测：**两条命令间隔 0 ms 时都回复的成功率仅 1/5；≥1 ms 即 5/5**。
本工程 `UartTransport::Config::tx_gap_ms = 12`（CAN 侧因无粘包问题取 2 ms）。

#### 16.5.2 `urgent` 是插队，**不是清空队列** ⭐

最初把 `urgent` 实现成 `txq_.clear()`。结果 `Gimbal::estop()` 里 `yaw_.estop()` 排入的「失能」命令，
被紧跟着的 `pitch_.estop()` 的 clear **直接清掉**，**yaw 永远没被失能**（自检里表现成 `yaw.en=1 pitch.en=0`）。
**一根总线挂多台电机时这是致命的。** 正确语义是 `push_front()` 插队，绝不动别人的命令。

#### 16.5.3 `Gimbal::enable()` 必须幂等 ⭐

操作端**每个控制包**都带 `Enable` 标志（语义是「保持使能」）。
若 `enable(true)` 每次都把 `enable_age_s_` 清零，`enable_delay_s`（0.3 s 上电空档）就永远走不完，
两轴会被永久压在 `hold()` 上 —— **表现是「发什么都是 0，但状态全绿」**，极难排查
（最后靠独立 Python 实现交叉验证才排除协议问题）。修法：`if (on == enabled_) return;`。
**协议里「每包都带的保持位」必须做成幂等操作。**

#### 16.5.4 看门狗：只归零、**不失能**

| 级别 | 触发 | 动作 | 参数 |
|---|---|---|---|
| 手感层 | `rate_hold_s` 无新指令 | 角速度指数衰减到 0 | 0.12 s，τ = 0.05 s |
| 安全层 | `command_timeout_s` 无指令 | 强制 hold，**保持使能** | 0.5 s |

**为什么安全层不失能**：云台一旦失能就松轴，受重力会垂下来 —— 比继续转更危险。
断流 0.9 s 实测状态字 `0x0F`（含 `Watchdog`），速度归零而 `Enabled` 位仍在，符合设计。

#### 16.5.5 遥测轮询每项独立计时

不能用轮转指针 `next = (next + 1) % N`（同 `AI-COMPUTER-LOG.md` §9.8 的教训：轮转指针会让状态冻结再跳变）。
`ZdtMotor` 里每个遥测项各持一个 `next_ms`，初始值错开 5 ms 避免同拍成簇。

#### 16.5.6 显式 `Disable` 标志

最初只有 `Enable`，操作端**没法显式失能**（「不发送」≠「失能」）。
补上 `flags` 位 `0x40 Disable`，与 `Enable` 互斥且优先。

### 16.6 UDP 操作端线协议

**两端必须逐字节一致。所有字段小端。用显式字节偏移写编解码，不依赖 C 结构体布局 / padding**
（跨编译器、跨架构不出错的关键；`control_link.hpp` 里 `putU16` / `getU32` 全是手写字节操作）。

**上行：PC → Jetson，16 字节**

| off | size | type | 字段 | 说明 |
|---|---|---|---|---|
| 0 | 4 | u32 | `magic` | `0x4D454341` |
| 4 | 4 | u32 | `seq` | 递增序号，两端用来算丢包 |
| 8 | 2 | i16 | `yaw_cdps` | yaw 角速度，单位 **0.01 °/s** |
| 10 | 2 | i16 | `pitch_cdps` | pitch 角速度，单位 0.01 °/s |
| 12 | 1 | u8 | `flags` | 见下表 |
| 13 | 1 | u8 | `chk` | 前 13 字节**逐字节 XOR** |
| 14 | 2 | u16 | `pad` | 保留，填 0 |

**下行：Jetson → PC，32 字节**

| off | size | type | 字段 | 说明 |
|---|---|---|---|---|
| 0 | 4 | u32 | `magic` | 同上 |
| 4 | 4 | u32 | `echo_seq` | 回显最后收到的 `seq`（客户端算 RTT / 丢包） |
| 8 | 2 | i16 | `yaw_cdps` | **实际输出**角速度 0.01 °/s |
| 10 | 2 | i16 | `pitch_cdps` | 同上 |
| 12 | 2 | i16 | `yaw_rpm` | 电机实测转速 **0.01 RPM** |
| 14 | 2 | i16 | `pitch_rpm` | 同上 |
| 16 | 4 | i32 | `yaw_cdeg` | 输出角估计 **0.01°** |
| 20 | 4 | i32 | `pitch_cdeg` | 同上 |
| 24 | 2 | u16 | `vbus_mv` | 母线电压 mV |
| 26 | 1 | u8 | `temp_c` | 驱动温度（解码按 `int8_t`） |
| 27 | 1 | u8 | `status` | 见下表 |
| 28 | 2 | u16 | `rx_hz` | 最近一秒接收频率 |
| 30 | 2 | u16 | `chk16` | 前 30 字节按 u16 **累加和** |

**`flags`（上行）**

| 位 | 名称 | 含义 | 位 | 名称 | 含义 |
|---|---|---|---|---|---|
| `0x01` | Enable | 保持使能 | `0x10` | EStop | 急停 |
| `0x02` | Hold | 停手（抱死） | `0x20` | ClearStall | 解除堵转保护 |
| `0x04` | Home | 回中 | `0x40` | Disable | 显式失能 |
| `0x08` | Zero | 角度清零 | | | |

**`status`（下行）**

| 位 | 名称 | 位 | 名称 |
|---|---|---|---|
| `0x01` | Enabled | `0x10` | YawLim（yaw 撞软限位） |
| `0x02` | YawOk | `0x20` | PitchLim |
| `0x04` | PitchOk | `0x40` | Stall |
| `0x08` | Watchdog | `0x80` | EStop |

**为什么用 UDP 而不是 TCP / WebSocket**：控制包极小（16 B @ 100 Hz ≈ 1.6 KB/s）；
**无队头阻塞**（丢包直接丢，等重传反而更糟）；**无连接状态**（客户端崩溃 / 重连不需握手）。
丢包容忍与时效判断交给两端看门狗（安全层在 `Gimbal` 里）。

### 16.7 传动比 2:1 的换算（最容易算错）

`--gear` 的语义是 **电机转数 / 输出轴转数**（2:1 减速填 `2.0`），影响三处：

```
输出 °/s  →  电机 RPM :   rpm = dps × gear / 6          （2:1 时 90 °/s → 30 RPM）
电机 RPM  →  输出 °/s :   dps = rpm × 6 / gear
脉冲 / 输出圈         :   pulses_per_motor_rev × gear   （3200 × 2 = 6400）
```

`pulses_per_motor_rev = 200 × 细分`，细分 16 → **3200 脉冲/电机圈**。自检里用断言锁死：
`90 °/s @ gear=2.0 → 电机 30.0 RPM`。

**机械约束现状**：yaw 有导电滑环，**360° 无机械限位**（软限位默认关闭，可用 `--yaw-limit` 打开）；
pitch 软限位默认 `-45 ~ +45`，**这两个数是估的，待机械实测确认**。

### 16.8 构建、运行与调试

```bash
# 板端构建
cd ~/gimbal && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j4

# 三种运行模式
./build/gimbal --selftest            # 纯软件自检，不需要任何硬件，跑完退出
./build/gimbal --dry                 # FakeTransport 模拟电机，跑交互
./build/gimbal                       # 真机（默认 /dev/ttyUSB0）
```

**命令行参数**（默认值即当前实机配置）

| 参数 | 默认 | 说明 | 参数 | 默认 | 说明 |
|---|---|---|---|---|---|
| `--device` | `/dev/ttyUSB0` | 串口设备 | `--acc` | 200 | 加速度档位 0–255 |
| `--baud` | 115200 | 波特率 | `--ctrl-port` | 40001 | UDP 控制口 |
| `--yaw-addr` | **2** | yaw 电机地址 | `--no-udp` | off | 不开 UDP，只用 stdin |
| `--pitch-addr` | **1** | pitch 电机地址 | `--hz` | 100 | 控制频率 |
| `--gear` | **2.0** | 传动比 | `--dry` | off | 纯软件模拟 |
| `--pulses` | 3200 | 脉冲 / 电机圈 | `--selftest` | off | 自检后退出 |
| `--yaw-limit a b` | **关闭** | 给 yaw 启软限位 | `--pitch-limit a b` | `-45 45` | pitch 软限位 |
| `--yaw-rate` | 180 | yaw 最大 °/s | `--pitch-rate` | 120 | pitch 最大 °/s |

**本地 stdin 命令**：`y <dps>` / `p <dps>` / `r <yaw> <pitch>` 设角速度 · `h` 停手 · `home` 回中 · `zero` 归零 · `en 0|1` 使能 · `x` 急停 · `s` 状态 · `q` 退出

**配套 PC 客户端 `~/gimbal_ctl`**（950 行 / 3 文件，原生 X11 + libjpeg，协议头直接复用不复制）：

```bash
cd ~/gimbal_ctl && cmake -S . -B build && cmake --build build -j4
./build/gimbal_ctl --host 192.168.2.2      # 鼠标控制 + MJPEG 视频
./build/gimbal_ctl --mode warp             # 若 XGrabPointer 手感不对
```

鼠标映射采用**角速度**（鼠标速度 → 云台角速度），`sens = 0.09 °/px`，`τ = 50 ms` 平滑。
按下时 `XGrabPointer` 锁光标，`Esc` 退出。客户端按键：`空格` 停手 · `e` 使能 · `x` 急停 · `h` 回中 · `z` 归零 · `r` 清速率 · `v` 视频开关。

### 16.9 与底盘库的对接点（已预留，尚未实现）

| 文件 | 角色 | 状态 |
|---|---|---|
| `include/gimbal/link/chassis_link.hpp` | `IChassisLink` 抽象 + `UartChassisLink` / `CanChassisLink` 两个实现 | 接口与数据结构完整；两个实现的 `poll()` 均为 `TODO` 占位 |
| `include/gimbal/transport/can_transport.hpp` | ZDT 电机改走 CAN 的传输层（29-bit 扩展帧、拆包重组） | 帧封装已按手册实现，注释明写**无硬件验证** |

`CanChassisLink` 是**全项目唯一的底盘接入点**，设计意图是「换实现不改上层调用点」——
将来底盘协议落地时，只需填 `poll()` / `setCommand()`，UDP 控制链路与云台逻辑都不用动。

⚠️ **当前占位实现故意不占用串口 / socket**：`UartChassisLink::open()` 只打印一行提示就返回 `true`，
不去 `open("/dev/ttyTHS1")`，避免与云台的 CH340 抢资源，也避免和本库调试串口 USART1 冲突。

### 16.10 预留底盘协议（与下位机对齐）

| ID | 帧类型 | 方向 | 内容 |
|---|---|---|---|
| `0x020` | 11-bit 标准帧 | Jetson → F407 | 底盘速度 `vx(i16 mm/s) \| vy(i16 mm/s) \| wz(i16 0.01rad/s) \| flags \| seq` |
| `0x000` | 11-bit 标准帧 | 广播 | 急停 |
| `0x110` | 11-bit 标准帧 | F407 → Jetson | 里程计 / IMU 反馈 |
| `(Addr << 8) \| Packet` | **29-bit 扩展帧** | 双向 | 云台 ZDT 步进电机 |

标准帧（11 位）与扩展帧（29 位）在 CAN 协议层天然隔离，**同一条总线上混跑不冲突** ——
这正是「云台和底盘共用一根 CAN，靠 ID 类型区分」这个设计的依据。

#### ⚠️ 待修正：坐标约定两侧不一致

| | `ChassisCommand`（上位机 `chassis_link.hpp`） | 本底盘库（§4） |
|---|---|---|
| `vx` | **右为正** | **+x = 前进** |
| `vy` | **前为正** | **+y = 左移** |
| `wz` | 逆时针为正 | 逆时针为正 ✅ |

**这两套约定是「交换 + 一项反向」的关系**，不是笔误级别的差异。
落地 `CanChassisLink` 时必须先统一，否则「前进」会变成「右移」。
建议**改上位机侧与本库 §4 对齐**（`+x 前进 / +y 左移`），因为本库的符号已完整贯穿运动学、里程计与 PID 三段，改下位机代价更大。

### 16.11 待决事项：总线波特率

| 方案 | F407 侧 | Jetson 侧 | 取舍 |
|---|---|---|---|
| **A（倾向）** | `CANTEST_PRESCALER = 6` → **500 kbps** | 不改 | ZDT 电机 CAN 默认 500 kbps，三者共总线时无需再改 |
| B | 保持 `3` → **1 Mbps** | 改 `CanChassisLink::Config.bitrate` | 总线余量大、延迟低 |

⚠️ **当前两侧不一致**：`chassis_link.hpp`（`bitrate = 500000`）与 `can_transport.hpp` 的默认值都是 500 kbps，
而 F407 实测跑在 **1 Mbps**（`CANTEST_PRESCALER = 3`）。**落地前必须先统一，否则总线必然不通。**

### 16.12 当前状态与下一步

#### 已验证 ✅

- 双平台编译通过（x86_64 笔记本 + aarch64 Jetson），**0 error / 0 warning**
- `--selftest` 全部通过（纯软件，无需硬件）
- 源码两边同步（md5 一致）；Jetson 侧已有 aarch64 可执行文件 `build/gimbal`（2.7 MB）
- **UDP 链路实机跑过**：`run.log` 记录显示板端监听 `0.0.0.0:40001` 后收到来自 `192.168.2.1`（笔记本）的连续控制包，
  以及优雅退出（速度归零 + 保持抱死）。**注意该次运行是 `--dry`（FakeTransport），未接真实电机。**
- UDP 协议用**独立 Python 实现交叉验证**，不依赖自身 C++ 编解码：

  | 用例 | 结果 |
  |---|---|
  | 发 `+90 °/s` | `yaw=+90.0`，电机 `+30.0 RPM`（90×2÷6 ✓） |
  | 发 `-120 °/s` | `yaw=-120.0`，电机 `-40.0 RPM` ✓ |
  | pitch 撞软限位 | `st=0x27`（含 `PitchLim`）✓ |
  | 断流 0.9 s | `st=0x0F`（含 `Watchdog`），速度归零，**使能位保持** ✓ |
  | 显式 `Disable` | `st=0x06`，`Enabled` 位清除 ✓ |
  | 急停 | 速度 0 + 已失能 ✓ |

#### 未验证 ❌（下一步的核心工作）

| # | 待办 | 依赖 |
|---|---|---|
| 1 | **真机 ZDT 串口通信**（`~/gimbal` 去掉 `--dry` 直连驱动板） | CH340 插上使 `/dev/ttyUSB0` 出现 |
| 2 | **方向极性标定**（两轴 `invert_dir`，命令行开关尚未暴露） | 同上 |
| 3 | **pitch 实际机械行程**实测（现 `-45~+45` 是估的） | 同上 |
| 4 | **两电机地址确认**（`yaw=2 / pitch=1`，须在驱动板菜单设，代码改不了） | 同上 |
| 5 | 双轴同步运动（ZDT 多机同步标志 + 广播触发，手册 §3.5.2） | 做绝对定位时 |
| 6 | 位置模式（`mkPositionEmm()` 已实现）上机验证 | 同上 |
| 7 | 统一总线波特率（§16.11） | CAN 硬件就绪 |
| 8 | 实现 `CanChassisLink`（SocketCAN 收发 + `0x020`/`0x110` 编解码） | **不依赖硬件，可先写** |
| 9 | F407 侧实现 `0x020` 解析 → 接入 `chassis_set_velocity()` | 需先核对 `chassis.h` 接口签名 |

#### CAN 方向暂停说明

上位机侧 CAN 适配器（KCANBus-U-I）在 Linux 下无法工作，选型结论见 §15.8。
**在与厂商确认 / 换用 CANable 2.0 之前，CAN 方向整体暂停。**

**不受影响、可独立推进的部分**：F407 的 silent-loopback 自检（§15.3 自检档）已验证协议链路完好；
底盘的电机闭环、运动学、PID 等全部逻辑与 CAN 无关；云台的 UART 链路与 `--dry` 自测也不依赖 CAN。

⚠️ **资源冲突提醒**：`/dev/ttyUSB0` **同一次只能被一个进程占用**。§9 的 `test_server.py` 与 `~/gimbal`
都想要它，调试前必须先杀掉另一个（`pkill -f test_server.py`）。

### 16.13 开发中踩过的坑（按价值排）

| # | 坑 | 现象 | 教训 |
|---|---|---|---|
| 1 | `Gimbal::enable()` 不幂等 | 发什么都是 0，但状态全绿 | 「每包都带的保持位」必须幂等 |
| 2 | `urgent` 实现成 `txq_.clear()` | yaw 永远不使能 | 多机共总线时清空队列会误杀别人的命令 |
| 3 | 渲染定时器 `t_render += 33ms` 跑飞 | 循环 10 ms 却每轮加 33 ms → `if` 永不成立，渲染块从不执行 | 定频要用「距上次 ≥ 周期」而非累加 |
| 4 | `XGrabPointer` 时序 | 返回 `rc=3 GrabNotViewable` | `XMapWindow` 后要等 `map_state == IsViewable` 再 grab |
| 5 | `CHECK` 宏把条件求值两次 | `CHECK(p.pop(b) && ...)` → `pop()` 跑两遍，吃掉两帧 | 宏里必须 `const bool _ok_ = (cond);` |
| 6 | `ZdtMotor::update()` 从没被调用 | 帧构造 / 解析全对，但遥测根本不下发 | 写完构造函数要确认谁在调它 |
| 7 | `CAN_MAX_DLC` 与 `<linux/can.h>` 宏撞名 | 编译冲突 | 改名 `kCanMaxDlc` |
| 8 | 方向约定不一致 | 正负号到处对不上 | 统一为：协议 `ccw=1` == 逻辑正方向 == 正转速 |
| 9 | 首帧判定 `shown_seq <= 1` | 视频明明在画，提示永不打印 | 渲染首跑时流里已有几十帧 |

### 16.14 验证边界（诚实声明）

本节所有 `~/gimbal` 数据来自：`--selftest` 输出、独立 Python 交叉验证脚本、`run.log` 实机记录、源码静态核对。

**没有一项来自「真实 ZDT 电机 + 真实云台机械」的运行** —— 该验证尚未进行（见 §16.12）。
因此以下内容属于**设计意图而非实测结论**：方向极性、pitch 机械行程、传动比在真实负载下的表现、
软限位对实际机械的保护效果、堵转保护联动的实际行为。

## 17. IMU（LSM6DSV16X）与陀螺偏航闭环（小陀螺）

### 17.1 硬件与接线

| 项 | 内容 |
|---|---|
| 模块 | 正点原子 ATK-MS6DSV（ST LSM6DSV16X 六轴，模块自带 4.7K 上拉） |
| 位置 | 底盘扩展板 IIC 座（H5），模块平放：丝印 **X 指车头、Y 指车体左侧、Z 向上** |
| 总线 | 3V3 供电（扩展板自带 R1/R3 两个 4.7K 上拉，与模块并联约 2.35K） |
| 接线 | 出厂时扩展板 H5 的 SDA / SCL 与模块引脚是**交叉**的（见 §17.7 坑 6），2026-10-18 已飞线改正 → 现在用**硬件 I2C1**（SCL=PB6 / SDA=PB7，400 kHz，`IMU_USE_SOFT_I2C=0`）。软件 I2C 后端保留着（`IMU_USE_SOFT_I2C=1`，SCL=PB7/SDA=PB6），换板子遇到同样的交叉可以直接切 |
| INT | **不用**（纯轮询，不接中断；芯片 INT 是推挽输出，空载无害）。线头剪短/包起来别蹭到别的网络 |
| SA0 | 2026-10-19 已**直连 GND** → 地址固定 **0x6A**。之前是弱上拉（板载 100k），会飘 → 见 §17.7 坑 11。注意**不能用 100k 下拉**：会和板载 100k 上拉分压成 1.65V，等于没接 |
| I2C 速率 | 建议 **400 kHz Fast Mode**（CubeMX：I2C1 → Speed Mode = Fast Mode）。仍为 100 kHz 时代码自动把陀螺采样周期从 2 ms 放宽到 3 ms |

轴映射写在 `UserLib/IMU/imu_config.h` 的 `IMU_MAP_X/Y/Z`（编码 = 符号 × (芯片轴号+1)，当前恒等映射）。
**验证方法**：手转车体逆时针（俯视），串口 `gz` 应变正；若为负，把 `IMU_MAP_Z` 取反即可，不用改接线。

### 17.2 数据链（`UserLib/IMU/imu.c`）

```
imu_poll()（跟着 1kHz 控制环调用，内部按 2ms 节拍限速）
   ├─ 读 OUTX_L_G（6 字节）→ 按 ±1000dps 换算 rad/s
   ├─ 减零偏 → 轴映射到车体系 → 40Hz 一阶低通（gyro_lpf，控制用这个）
   ├─ yaw += gyro_lpf[2] * dt（±PI 归一）
   └─ 每 10 个样本读一次加速度（只做静止/倾角参考）

静态方法：
   imu_calibrate_bias()  上电静止标定 300 点（约 0.6s），标准差 >0.05rad/s 判为「车在动」→ 标记未标定
   imu_track_bias()      车静止时按 τ=4s 慢速跟踪残余零偏（车在动时完全不动）
```

- 采样率：硬件 I2C 400 kHz 下 500 Hz（`IMU_GYRO_PERIOD_MS=2`）；100 kHz 硬件 I2C 自动降到 333 Hz；
  实测一次 6 字节读：**硬件 400 kHz = 213 µs**，软件 I2C（兜底方案）= 351 µs；
- 采样耗时 `imu_state()->read_us` 是 DWT 周期计数量的真实值，串口 RawText 的 `us=` 字段直接看得到；
- 陀螺量程 ±1000 dps：分辨率 0.03 dps ≈ 5.3e-4 rad/s，而自旋上限 4.76 rad/s = 273 dps，留 3.6 倍余量；
- 阻塞代价：一次读 213 µs、每 2 ms 一次 ≈ **10.6% CPU**（软件 I2C 那套是 351 µs ≈ 17%）。
  想上 1 kHz 采样或者降到零阻塞，得改成 I2C + DMA，本库暂不做。

**掉线与恢复**（2026-10-18 实测踩到过一次运行中掉线，复位后恢复）：

| 机制 | 参数 | 行为 |
|---|---|---|
| 总线恢复 | `IMU_ERR_RECOVER_COUNT=5`、限流 `IMU_RECOVER_MIN_MS=100` | 连续 5 次读失败 → 硬件模式 `DeInit` **+ GPIO 打 9 个时钟 + STOP + Init**；软件模式打 9 个时钟 |
| 判掉线 | `IMU_ERR_STALE_COUNT=10`（20 ms） | `imu_state()->healthy=false`，陀螺外环**自动退回纯运动学**，不再拿陈旧值去修正 |
| 地址热切换 | — | 读/写失败时自动换另一个地址（0x6A ↔ 0x6B）再试一次，成功就锁定新地址并计入 `addr_switches`（SA0 不稳时靠它兜住） |
| 自动恢复 | — | 只要重新读通，`healthy` 立刻回 true，外环无缝接回 |
| 诊断字段 | 串口 `eps= ad= sw= rec=` | 每秒新增错误数 / 当前地址 / 换地址次数 / 总线解锁次数，一行判断总线状态 |

缺了 9 个时钟那一步只 DeInit/Init 是解不开的：从机把 SDA 拉死时，必须给它时钟把状态机顶出来。

**20 分钟空闲 soak 对比**（`test_data/spin_runs/idle_20min.log` / `soak_after_reflow_20min.log`）：

| 版本 | 20 分钟错误数 | 形态 | 结果 |
|---|---|---|---|
| 重飞线前 | **144 次** | 集中在 t≈1100 s 的一次 0.7 s 爆发（`us` 213→27→213、`hz` 499→262→499） | 自动恢复，没进 STALE |
| 重飞线后 | 0 次 | `us` 恒为 213、`hz` 恒 499、`e` 全程 0 | 一次都没掉（后来又复发了，见下） |
| **SA0 焊死 GND 后** | **0 次** | `ad` 恒为 `6A`、`sw=0`、`rec=0`、`eps` 恒 0、`us` 恒 214 | 4801 帧全绿，彻底解决 |

结论：那次卡死是**飞线电气质量**问题（长线 + 400 kHz 边沿余量），重做线之后 20 分钟零错误；
软件的 9 时钟解锁 + `healthy` 回退仍然保留作兜底。

### 17.3 陀螺外环控制律（`UserLib/Chassis/chassis_gyro.c`）

```
wz_ref = 用户指令 / 自旋速率 / 角度环输出
e      = wz_ref − wz_gyro
wz_out = clamp( wz_ref + Kp·e + Ki·∫e , ±CHASSIS_GYRO_WZ_LIMIT )
```

四条设计约束（每条都来自踩坑，别随手删）：

1. **只修正不替代**：前馈仍是 `wz_ref`，陀螺只补打滑/擦地的差额，所以 IMU 掉线会退回纯运动学而不是失控；
2. **斜坡段冻结修正**：`chassis.c` 用「上一拍实际轮速目标 vs 逆解目标」判断斜坡还在爬，此时外环沿用上一拍的修正量。
   不冻结的话起步瞬间 `Kp·e`（3.5×0.6=2.1）会直接把 wz 顶到限幅——仿真里无打滑时自旋峰值冲到 4.29 rad/s（超调 23%）；
3. **能力带 `CHASSIS_GYRO_I_BAND`**：`|e|` 超过它也冻结（大误差 = 物理上还没追上）。
   必须大于最大预期打滑误差：30% 打滑 @3.5 rad/s ≈ 1.05 rad/s，所以给 2.0；
4. **条件积分抗饱和**：输出顶 `CHASSIS_GYRO_WZ_LIMIT` 且误差还在往同方向推时，本拍不积分。

外环输出限幅 `CHASSIS_GYRO_WZ_LIMIT = 4.60` 比用户指令限幅 `CHASSIS_MAX_WZ_RAD_S = 4.0` 高：
因为自旋要补偿打滑，物理上限是 `250RPM × 0.10472 × 0.04 / 0.2201 ≈ 4.76 rad/s`。

### 17.4 三种模式与接口

| 模式 | 用途 | 入口 |
|---|---|---|
| `CHASSIS_GYRO_MODE_OFF` | 不参与，wz 直接用指令（排错退路） | `chassis_gyro_set_mode()` |
| `CHASSIS_GYRO_MODE_RATE` | 角速度闭环：用户 wz 当目标，陀螺做修正（**默认**） | `chassis_set_velocity()` 照常调用 |
| `CHASSIS_GYRO_MODE_SPIN` | 小陀螺：恒定自旋速率 | `chassis_spin(rad_s)`，`chassis_spin(0)` 退出 |
| `CHASSIS_GYRO_MODE_YAW_TARGET` | 偏航角闭环：底盘转到指定角 | `chassis_gyro_set_yaw_target(rad)` |

状态镜像在 `chassis_state()`：`imu_wz`（陀螺实测）、`imu_yaw`（陀螺积分）、`wz_eff`（实际下发）、`imu_online`。

### 17.5 云台跟随（预留，尚未接入）

真要接云台时只需要一句：

```
底盘目标角 = 云台世界角 = imu_yaw + 云台相对底盘角
chassis_gyro_set_mode(CHASSIS_GYRO_MODE_YAW_TARGET);
chassis_gyro_set_yaw_target(gimbal_world_yaw);
```

云台相对角从云台 yaw 电机编码器来（CAN/USART 都行，接在 `main` 或上层链路里），
底盘库不关心它从哪来。小陀螺本身（底盘恒速自旋 + 云台自己反向补偿）不需要这个接口。

### 17.6 验证现状

**已做的**：`chassis_gyro.c` 的闭环逻辑在 PC 上做过对象仿真（轮速斜坡 + 一阶电机滞后 + 打滑系数 + 陀螺噪声）：

| 场景 | 稳态误差 | 峰值 | 说明 |
|---|---|---|---|
| 自旋 3.5 无打滑 | <0.001 rad/s | 3.503 | 无超调 |
| 自旋 3.5 打滑 18% | −0.003 rad/s | 3.500 | 外环补 0.76 rad/s |
| 自旋 3.5 打滑 30% | −0.28 rad/s | 3.220 | 顶 4.6 限幅，属物理极限 |
| 自旋 1.5 / 0.5 | <0.001 rad/s | 目标值 | 低速也一样 |
| 自旋 −3.5（反向） | +0.003 rad/s | −3.500 | 符号正确 |

**2026-10-18 实机已确认**：

| 项 | 实测 | 备注 |
|---|---|---|
| 握手 | `imu ok`，WHO_AM_I=0x70，模块地址 **0x6B**（SA0 被板子拉高） | 两个地址都探测，接 GND 变 0x6A 也能用 |
| 采样（硬件 I2C 400 kHz，飞线后） | `hz=499`、`us=213`（连抓 12 s 一帧不变） | 无超时、无重试 |
| 采样（软件 I2C 兜底） | `hz=499`、`us=351` | 引脚交叉时用这套 |
| 零偏 | 上电静止标定后 `gz` 在 ±0.001 rad/s 内 | |
| 加速度 | 平放时 `ac=(0.33,0.43,9.89) m/s²` → Z≈9.9、X/Y≈0 | 「X 前 / Y 左 / Z 上」映射正确 |
| 噪声（静止 1 kHz 原始值） | `gz` std **0.028 dps = 0.49 mrad/s**（40Hz 低通后 0.34 mrad/s） | 见下方噪声实测 |

**2026-10-18 手转 + 架空实测**（`test_data/spin_runs/`）：

| 测试 | 结果 | 说明 |
|---|---|---|
| 手转逆时针 | `gz` 单调为正（峰值 +1.06 rad/s）、陀螺积分 yaw −97°→+78° | **轴映射与符号正确**，`IMU_MAP_Z` 不用取反 |
| 同一次手转里的外环 | 四轮自动反着给 ±78 RPM（占空比 ~±550‰）、积分 0.8 s 顶到 `I_MAX×KI=1.6` | 抗旋转扰动方向正确，闭环确实在工作 |
| 手转停止后 | 静止 4 s 内陀螺积分 yaw 完全不动 | 零偏跟踪有效，短时静止看不到漂移 |
| 实验 5 四轮架空 | 目标 ±3.5 rad/s → 四轮 **±184 RPM**（占空比 686‰ / 658‰），车体不转所以 `gz≈0` | 运动学反解 = 手算 183.9 RPM ✅；**电机 PWM 对 I2C 零干扰**（转动中陀螺噪声 std 0.0014 rad/s，静止 0.0004） |

#### 噪声实测（静止、1 kHz 原始值、±1000dps / ±4g，`test_data/spin_runs/imu_noise*.log`）

| 项 | 实测 | 备注 |
|---|---|---|
| `gz` 噪声 std | **0.028 dps = 0.49 mrad/s** | 经 40 Hz 软件低通后 **0.34 mrad/s** |
| `gx` / `gy` 噪声 std | 0.05~0.08 dps = 0.9~1.4 mrad/s | 比 z 轴略大，不影响（只闭环 wz） |
| 等效噪声密度 | gz ≈ **1.3 mdps/√Hz**，gx/gy ≈ 2.3~3.8 mdps/√Hz | 数据手册典型值 2.8 mdps/√Hz，正常水平 |
| 加速度噪声 std | 20~25 mm/s²（≈2~2.5 mg） | |
| 零偏（原始） | gz **+0.32 dps**、gx −0.62 dps、gy +0.11 dps | 逐块（4s/块）重复性 **0.003 dps = 0.05 mrad/s**，很稳 |
| **上电瞬态** | 上电后约 5 s 内 x/y 轴噪声明显变大（gx 峰峰 11 dps、acc_z std 33 mm/s²），之后恢复 | 所以零偏标定延后到 **1.2 s**（`IMU_STARTUP_SETTLE_MS`） |

#### 漂移实测（静止 130 s，`test_data/spin_runs/drift_120s.log`）

| 项 | 实测 |
|---|---|
| 陀螺积分 yaw | 130 s 内显示 `0°`，即 **< 1°**（串口按整度输出，分辨率有限） |
| 期间 `gz` 均值 | ±0.0000 rad/s（±0.03°/s 量级以内） |
| 按零偏块间重复性推算 | 约 **0.1~0.3 °/min**，10 分钟 1~3° |

> 对比一下量级：四轮测速的量化噪声约 ±2 RPM ≈ **±0.21 rad/s**，是陀螺噪声（0.0005 rad/s）的 **400 倍**。
> 也就是说陀螺给的是整个底盘库里最干净的速度反馈，外环用它是赚的。

#### 地面自旋实测（2026-10-19，首轮）

四轮落地、当时 `CHASSIS_SPIN_RATE_RAD_S=3.5`，四段（静置/+3.5/停/−3.5）数据在
`test_data/spin_runs/spin_ground_3v5.log`。**这一轮被外部干扰污染（车被线拽住/卡住），不能当闭环指标用**，
但两个现象是真实的：

| 现象 | 数据 | 结论 |
|---|---|---|
| +3.5 rad/s 段：满占空比也只到 2.0~2.6 rad/s | 四轮被外环推到 −211 / +242 RPM，`duty=1000‰` 顶满，`gyro` 峰值 2.64 后掉到 1.7 | **3.5 rad/s（200°/s）超出这台车在该地面的抓地极限**；麦轮自旋要横向擦地，损耗很大 |
| −3.5 rad/s 段：轮速精准到位但车体不转 | 四轮 = ±184 RPM（正好等于运动学解）、`duty=830‰` 未饱和、`gyro≈0.00` | 轮子在空转、车体没在地上 → 现场确认是**车被卡住**，数据作废 |

调参结论：**自旋速率要按实际抓地能力定**。这台车在地砖上大约 2.0~2.5 rad/s（120~145°/s）是稳态上限；
`CHASSIS_SPIN_RATE_RAD_S` 因此**从 3.5 下调到 2.0**（约 115°/s，四轮约 105RPM）。
等有开阔场地 + 不绕线的条件，再用扫速（1.0/2.0/3.0/4.0 rad/s 各 2 s）实测实际上限。

**还没做的（需要落地面）**：真实打滑率、自旋转速闭环稳态误差/纹波、yaw 长时间漂移。
用 §6.8 实验 5 抓数据（`CHASSIS_TUNE_ENABLE=1` + `CHASSIS_TUNE_EXPERIMENT=5`），
四段各 2 s，看 `ch1−ch0` 的稳态差与纹波、以及停机段 yaw 是否还在爬。
抓到的原始串口数据用 `tools/spin_analysis.py <log>` 一键算指标（需要 `/usr/bin/python3`，系统 python 带 numpy）。

### 17.7 踩坑记录

| # | 坑 | 现象 | 教训 |
|---|---|---|---|
| 1 | 浮点常量用在 `#if` | `error: floating constant in preprocessor expression` | 配置里的 `40.0f` 只能进运行期 `if` |
| 2 | 外环 P 项在起步瞬间顶满 | 自旋峰值 4.29 rad/s（目标 3.5） | 斜坡/大误差段冻结修正 |
| 3 | 冻结条件只挂 `ramping` | 稳态打滑 18% 永远补不回来（停在 3.35） | 冻结条件必须同时挂误差带 |
| 4 | 误差带取 1.0 太窄 | 30% 打滑（误差 1.05）掉在带外，永远不积分 | 带宽 > 最大打滑误差 |
| 5 | ST 官方驱动 I2C 读多字节 | 给寄存器地址置 0x80 会读到错误寄存器 | I2C 模式不置 0x80，靠 IF_INC 自动递增（并显式 `auto_increment_set`） |
| 6 | **H5 座上 SDA/SCL 交叉**（已飞线修复） | 硬件 I2C1 对 0x6A/0x6B 全部 NACK（`Err=0x04` AF），全总线扫描 0 个设备；但引脚空闲电平正常、外设寄存器配置全对（CR2 FREQ=42、CCR=0x20D、TRISE=0x2B） | 用**软件 I2C 两种引脚角色各扫一遍**：SCL=PB6/SDA=PB7 → 0 个；SCL=PB7/SDA=PB6 → **0x6B ACK、WHO_AM_I=0x70**。结论：模块是好的，板子把 SDA/SCL 接反了。软件 I2C 兜底可立刻跑；飞线把两根交叉回来后硬件 I2C 400 kHz 直接通（实测 213µs/读） |
| 7 | 软件 I2C 用 `HAL_GPIO_WritePin` | 一次读 6 字节要 **1285 µs**（-O0 下 HAL 调用太慢） | 改用 `BSRR` 直写 + DWT 周期计数做延时 → **351 µs** |
| 8 | 上电就标零偏 | 上电后 ~5 s 内陀螺 x/y 噪声偏大（gx 峰峰 11 dps、acc_z std 33 mm/s²），0.7 s 就标定会被这段瞬态污染 | 加 `IMU_STARTUP_SETTLE_MS=1200`，标定延后到 1.2 s；上电到能跑车约 1.8 s |
| 9 | 飞线后 SWD 提速到 1 MHz 会掉 | 飞线后 SWD 1 MHz 写 flash 报 `Failed to write memory`，降到 500 kHz 就成功 | 飞线后检查 SWD 那 4 根线（尤其 GND）；烧录用 `adapter speed 500` |
| 11 | **SA0 弱上拉 → 地址漂移** | 运行中大量 27µs 快速 NACK（地址字节被拒），错误计数暴涨，控制环掉拍；偶发一阵后自己恢复 → 复发 | 根因是 SA0 没被拉死。**直连 GND（0Ω 或 ≤1k）**，不要 100k（会与板载 100k 上拉分压成 1.65V）。焊死后 20 分钟 soak 零错误 |
| 10 | IMU 运行中突然掉线 | 串口变成 `imu ok` 但 `us=27 hz=0 ac=0,0,0`（读在地址阶段就被 NACK），复位能恢复 → HAL I2C 状态机/总线卡死，不是模块坏 | 恢复流程必须**先 DeInit 再用 GPIO 打 9 个时钟**把从机顶出来；另外连续失败要判 `healthy=false` 让外环退回，不能继续用陈旧数据 |

### 17.8 IMU 运行期地址应答不稳 → 底盘电机「一卡一卡」（2026-10-19 现场）

> 一次完整的故障报告，供 IMU 作者对照排查。**修复已烧录**，当前状态：
> `e=0`、`us=214µs`、`hz=499`、IMU `CAL=1`、F407 反馈 **100 Hz**。

#### 现象

- 无线遥控 W/S/A/D 驱动时，四轮直流减速电机**一卡一卡**（走走停停、顿挫）。
- Jetson 侧实测 F407 反馈帧率只有 **57 Hz**（正常 100 Hz），帧间隔 p50≈10 ms、p90≈51.7 ms、max≈67.7 ms。
- UART1 调试口（DAPLink 的 CDC）读到：`e=1496722` 且持续增长（≈280~520 次/秒）、
  `us=27`、`hz=499`、`imu ok`、`CAL=0`（零偏标定无效）。

#### 证据

| 观测 | 数值 | 结论 |
|---|---|---|
| 单次读耗时 `us=` | **27 µs** | 400 kHz 下地址字节 9 bit ≈ 22.5 µs —— **在地址阶段就被 NACK**，不是 3 ms 超时 |
| 错误计数 `e=` | 149 万，持续 +280~520/s | 约一半以上的读失败（`IMU_ERR_RECOVER_COUNT=5` 频繁触发恢复） |
| 采样节拍 `hz=` | 499 | 失败后仍按 2 ms 节拍重试，节拍本身正常 |
| 反馈帧率 | 57 Hz（p90 51.7 ms） | 1 kHz 控制环掉拍 ≈40 % |
| 修复后 | `e=0`、`us=214 µs`、`CAL=1`、反馈 **100 Hz** | 换地址重试后读成功率 100 % |

#### 根因

1. **地址应答不稳**：驱动启动时探测到的是 `0x6B`（SA0 被板子拉高），但运行期中约一半的
   I2C 事务在**地址字节**就被 NACK（`us≈27µs`）。这说明从机地址应答不稳定 ——
   嫌疑指向 **SA0 引脚悬空/接触不良导致地址在 0x6A/0x6B 之间漂移**
   （`IMU_PROBE_BOTH_ADDR=1` 探测时两个地址都能应答，正好掩盖了这一点）。
2. **阻塞式恢复雪上加霜**：`imu_note_result()` 连续失败 5 次 → `imu_bus_recover()` →
   `hw_bus_unstick()` 用 `HAL_Delay(1)` 连打 20 多下（DeInit + 9 个 SCL 脉冲 + STOP + 重 Init），
   **每次约 25 ms，且限流 100 ms 一次**；另有部分失败走满 `IMU_I2C_TIMEOUT_MS=3`。
   两者叠加，1 kHz 控制环每 100 ms 掉拍 30~40 % → 电机顿挫、反馈掉频。
3. 调试串口 `HAL_UART_Transmit` 每 250 ms 阻塞约 17 ms（200 字节），进一步加重掉拍。

#### 已做的修复

| 文件 | 改动 | 效果 |
|---|---|---|
| `UserLib/IMU/imu.c` | `plat_read/plat_write` 失败时**换 0x6A/0x6B 另一个地址重试一次**，成功则锁定新地址（`imu_alt_addr()`） | `e`：150 万 → **0** |
| `UserLib/IMU/imu.c` | 新增 `dwt_delay_us()`；`hw_bus_unstick()` 的 `HAL_Delay(1)` 全换 10 µs 忙等 | 恢复耗时 25 ms → **0.24 ms**（兜底，即使再触发也不拖控制环） |
| `UserLib/Chassis/chassis_debug.c/.h` | 调试发送改 `HAL_UART_Transmit_IT` + 缓冲 + 忙标志（发不完丢帧） | 每 250 ms 不再阻塞 17 ms |
| `UserLib/Comm/uart_link.c` | `HAL_UART_TxCpltCallback` 里转发 `chassis_debug_on_tx_cplt()` 清忙标志 | |
| `Core/Src/usart.c` | USART1_MspInit 补 `HAL_NVIC_EnableIRQ(USART1_IRQn)`（优先级 7） | `Transmit_IT` 能真正发出去 |
| `Core/Src/stm32f4xx_it.c` | 补 `USART1_IRQHandler` + `extern huart1` | |

#### 验证

- `e=0`、`us=214 µs`（正常 6 字节读耗时）、`hz=499`、`CAL=1`（零偏标定恢复）；
- UART1 调试行 `t=` 间隔精确 250 ms（之前偶发 +18 ms）；
- Jetson 侧反馈：201 帧中 **200 帧 CRC 全对**、2 s 内 200 帧 ≈ **100 Hz**。

#### 给 IMU 负责同学的建议

1. **先查 SA0 硬件**：SDA/SCL 飞线已确认没问题，但 **SA0 没查**。上电量一下
   H5 座/飞线上 SA0 的电平是否稳定在 VCC 或 GND；若悬空，硬件上焊死一个电平最干净
   （代码里定死地址），否则任何「双地址自动切换」都只是软件兜底。
2. **代码层面**：探测时记下两个地址中应答的那个；运行期检测 NACK 时自动切另一个
   （本次的兜底做法，`imu_alt_addr()`）；建议在 `imu_state` 里加「当前地址 /
   每秒新增错误数 / 恢复次数」，一眼看出总线状态。
3. **阻塞式恢复改非阻塞**：`hw_bus_unstick()` 放在 1 kHz 控制环里，`HAL_Delay(1)` 打
   20 多下 = 25 ms 级阻塞，务必改 µs 级忙等（DWT）或下放到底优先级任务。
4. **读超时**：`IMU_I2C_TIMEOUT_MS=3` 对 400 kHz 6 字节（~200 µs）足够；
   但若总线接触不良会出现整段 3 ms 阻塞，可考虑失败后冷却一拍再重试。
5. **调试输出**：`e=` 累计计数很好用；建议再补「每秒新增错误数」与「最近一次错误类型
   （AF/BERR/ARLO）」，定位 NACK 还是总线错误会快很多。

#### 复现与验证方法

- 连接：DAPLink 的 `SWD`（烧录）+ `UART1`（调试口，CDC 出现在主机 `/dev/ttyACM0`）。
- 读调试：`stty -F /dev/ttyACM0 115200 raw -echo` 后 `cat`，看 `e=` 是否增长、
  `us=` 是否为 27 µs（异常）或 ~214 µs（正常）。
- 反馈帧率：Jetson `/dev/ttyUSB0` 抓 2 秒原始字节，按 `5A A5 81`+40B+CRC+`0D` 切帧计数。
- 注意：USART1 的 NVIC 已写入 `.ioc`（`NVIC.USART1_IRQn=true:7:0`，等价于在 CubeMX
  里勾选 USART1 global interrupt），CubeMX 重新生成代码时会自动生成 `USART1_IRQHandler`
  与 MspInit 里的 NVIC 使能；`stm32f4xx_it.c` 里手工补的那段在重新生成后会被官方版本
  替换（内容等价，不影响功能）。

## 18. USART2 上位机控制链路（`UserLib/Comm/uart_link`）

> 这是**实际跑车用的控制入口**。§15 的 CAN 方案当前暂停（适配器 Linux 支持问题），
> §16 的 Jetson `~/gimbal` 是云台工程，两者都以这条 UART 链路为准。
> 模块自身的完整文档在 [`../UserLib/Comm/README.md`](../UserLib/Comm/README.md)。

### 18.1 拓扑与数据流

```
Jetson Nano / 笔记本（USB-TTL，115200-8-N-1）
        │  PD5(TX) -> F407 RX     PD6(RX) <- F407 TX
        ▼
USART2 RX 中断（逐字节）→ 256 B 环形缓冲 → uart_link_poll() 切帧
        │  CRC 校验 + 命令字分发
        ├─ SPEED  → chassis_spin(0) + chassis_set_velocity(vx, vy, wz)
        ├─ SPIN   → chassis_spin(wz)
        ├─ ESTOP  → chassis_spin(0) + chassis_stop() + chassis_driver_enable(0)
        └─ CLEAR  → chassis_reset_odometry() + chassis_gyro_reset()
        │
        └─ 反馈帧 40 B / 10 ms → USART1 之外的 USART2 发回 Jetson
```

### 18.2 命令帧（Jetson → F407，**12 B**，全部小端）

| off | 0 | 1 | 2 | 3 | 4..5 | 6..7 | 8..9 | 10 | 11 |
|---|---|---|---|---|---|---|---|---|---|
| 字段 | `0xA5` | `0x5A` | `cmd` | `seq` | `vx` i16 | `vy` i16 | `wz` i16 | `flags` | `crc8` |
| 单位 | — | — | 见下表 | 递增 | mm/s | mm/s | 0.01 rad/s | 位域 | — |

- `vx/vy`：**+x 前进 / +y 左移**（与 §4 的底盘约定一致）
- `wz`：0.01 rad/s，逆时针为正
- `crc8`：CRC-8/ATM（poly 0x07，init 0x00，不反射），覆盖 **offset 0~10**
- `flags`：bit0 `EN` = 允许驱动；`EN=0` 时命令帧只刷新看门狗，目标速度被清零

| cmd | 名称 | 作用 |
|---|---|---|
| `0x01` | SPEED | `vx/vy/wz` 直接下发；执行前先 `chassis_spin(0)` 退出自旋（SPIN 会无视用户 wz） |
| `0x02` | ESTOP | 退出自旋 → 清速度 → **拉低 STBY**（电机自由）；需要下一帧带 `EN` 的 SPEED/SPIN 才能恢复 |
| `0x03` | SPIN | `wz` 字段当自旋速率 → `chassis_spin(wz)`（小陀螺） |
| `0x04` | CLEAR | 里程计 (x,y,yaw) + 陀螺积分 yaw 一起归零 |

### 18.3 反馈帧（F407 → Jetson，**40 B**，10 ms 一帧 = 100 Hz）

| off | 字段 | 类型 | 单位 |
|---|---|---|---|
| 0..1 | 帧头 `0x5A 0xA5`（与命令帧方向相反，便于重同步） | u8×2 | — |
| 2 | 类型码 `0x81` | u8 | — |
| 3 | 最近一次合法命令的 `seq` 回显 | u8 | — |
| 4..7 / 8..11 | 里程计 `x` / `y` | i32 | mm |
| 12..13 | 里程计 `yaw` | i16 | 0.01° |
| 14..15 / 16..17 | 正解速度 `vx` / `vy` | i16 | mm/s |
| 18..19 | 正解角速度 `wz` | i16 | 0.01 rad/s |
| 20..21 / 22..23 | 陀螺积分 `yaw` / 陀螺实测 `wz` | i16 | 0.01° / 0.01 rad/s |
| 24..25 | `status`（见下表） | u16 | 位域 |
| 26..27 | `cmd_cnt` 累计合法命令帧数（低 16 位，上位机算丢包） | u16 | — |
| 28..29 | `fb_seq` 反馈帧自身序号（算接收率） | u16 | — |
| 30..37 | 四轮实测转速 FL/FR/BL/BR | i16×4 | 0.01 RPM |
| 38 | `crc8`（覆盖 offset 0~37） | u8 | — |
| 39 | 帧尾 `0x0D` | u8 | — |

`status` 位：

| 位 | 名称 | 含义 |
|---|---|---|
| 0x0001 | IMU_ONLINE | IMU 在线且配置成功 |
| 0x0002 | LINK_OK | 最近 100 ms 内收到过合法控制帧 |
| 0x0004 | WATCHDOG | 控制帧超时，目标速度已清零 |
| 0x0008 | ESTOP | 急停锁存中 |
| 0x0010 | DRIVER_ON | TB6612 STBY 高（输出使能） |
| 0x0020 | SPIN | 小陀螺模式 |
| 0x0040 | GYRO_ON | 陀螺外环非 OFF |
| 0x0080 | IMU_CAL | 陀螺零偏标定有效 |

### 18.4 看门狗与安全约定

| 层级 | 触发 | 动作 | 参数 |
|---|---|---|---|
| 控制层 | 100 ms 没有合法 SPEED/SPIN 帧 | 目标速度强制清零（经斜坡减速）+ 置 `WATCHDOG`；**不失能、不松 STBY** | `ULINK_CMD_TIMEOUT_MS=100` |
| 提示层 | 500 ms 连一帧合法命令都没有 | 置 `LINK_LOST`（只作状态显示，停车已在上一层完成） | `ULINK_CMD_LOST_MS=500` |
| 急停 | ESTOP 命令 | 退出自旋 + 清速度 + 拉低 STBY，锁存 | 需带 `EN` 的帧恢复 |

**为什么不超时就失能**：底盘/云台一旦松轴，受自重和惯性甩动比继续停着更危险（同 §16.5.4 的云台结论）。

### 18.5 接口与统计

```c
uart_link_init();      /* main() 里 chassis_init() 之后 */
while (1) { chassis_poll(); uart_link_poll(); }
const uart_link_state_t *s = uart_link_state();   /* 只读统计 */
```

| 统计字段 | 含义 | 什么时候该看 |
|---|---|---|
| `rx_frames` / `last_seq` | 累计合法命令帧 / 最近 seq | 上位机算丢包率 |
| `rx_crc_err` | CRC 失败次数 | 持续增长 = 干扰/波特率/线长问题 |
| `rx_sync_err` | 帧头对不上的字节数 | 增长 = 丢字节后重同步 |
| `rx_hw_err` | ORE/FE/NE 硬件错误 | 增长 = 波特率或电气问题 |
| `rx_overflow` | 环形缓冲溢出丢字节 | **只有主循环被卡住才会出现**，是"掉拍"的直接证据 |
| `tx_frames` / `tx_drop` | 已排入发送 / 因上一帧没发完丢掉 | `tx_drop` 增长 = 串口发不动 |
| `cmd_speed` / `cmd_unknown` | 速度帧 / 未知命令字计数 | 排查上位机发错命令 |
| `wdg_trips` | 看门狗触发次数 | 链路抖动的量化指标 |

### 18.6 与底盘库的对接（单位与约定）

| 链路量 | 底盘量 | 换算 |
|---|---|---|
| `vx/vy` (i16) | `chassis_set_velocity(vx, vy, wz)` (m/s) | ×0.001 |
| `wz` (i16) | wz (rad/s) / `chassis_spin(rad_s)` | ×0.01 |
| 反馈 `x/y` (i32 mm) | `chassis_state()->x / y` (m) | ×1000 |
| 反馈 `yaw/wz` | 里程计 / 陀螺 | 0.01° / 0.01 rad/s |

> ⚠️ **坐标约定必须统一**：本链路与底盘库 §4 一致（+x 前进 / +y 左移）。
> §16.10 里 Jetson 侧 `ChassisCommand` 那套是「vx 右为正 / vy 前为正」，两者是**交换 + 一项反向**的关系，
> 上位机发帧前要转过来，否则「前进」会变成「右移」。

### 18.7 PC 端验证

| 工具 | 用途 |
|---|---|
| `tools/uart_link_test.py` | 生成测试向量 / 独立 Python 实现交叉校验（不依赖 C 代码） |
| `tools/codec_test.c` | 在 PC 上用 gcc 编译 `uart_link_codec.h`，做编解码双向自检 |

实测（2026-10-19）：100 Hz 反馈稳定，`cmd_cnt`/`fb_seq` 连续，200/201 帧 CRC 全对；
IMU 地址应答问题修好后，此前"反馈掉到 57 Hz"的现象消失。

### 18.8 踩坑

| # | 坑 | 现象 | 教训 |
|---|---|---|---|
| 1 | CubeMX 没勾 USART2 全局中断 | 一个字节都收不到，`rx_bytes` 恒 0 | `stm32f4xx_it.c` 里必须有 `USART2_IRQHandler`；`.ioc` 的 `NVIC.USART2_IRQn` 要打勾，否则重新生成会丢 |
| 2 | 主循环里有长阻塞 | `rx_overflow` 增长、丢帧 | 256 B 环形缓冲在 100 Hz×12 B = 1.2 KB/s 下只够**约 200 ms**，控制环/串口任何一处卡住都会溢出 |
| 3 | 改了字段没同步两端 | 上位机读数乱跳 | CRC 覆盖范围（命令 0~10 / 反馈 0~37）、偏移、单位必须与 `uart_link_codec.h` 和 PC 侧逐字节对齐 |
| 4 | 帧头方向 | 反向帧头容易被误认 | 命令 `0xA5 0x5A`、反馈 `0x5A 0xA5` 故意相反，切帧时先判方向再校验 |
| 5 | 自旋模式忽略用户 wz | 发了 SPEED 车不按指令平移 | `uart_link` 里 SPEED 执行前先 `chassis_spin(0)`，否则一直停在自旋 |

## 19. 版本记录

| 版本 | 日期 | 说明 |
|---|---|---|
| 0.1 | 2025-10-03 | 初版：motor/encoder/pid/mecanum/chassis/demo，速度闭环 + 里程计，裸机 1 kHz |
| 0.2 | 2025-10-03 | 拆分电机方向与编码器方向归一；新增 chassis_debug 串口调试打印；右两轮 ENC_DIR 默认取反 |
| 0.3 | 2025-10-03 | 依据串口实测数据整定：Kp 5→3、Ki 0.5→0.6、Kd 0→0.08、I_MAX 300→900（解决到不了目标转速与右后轮振荡） |
| 0.4 | 2025-10-03 | chassis_debug 增加 JustFloat 二进制协议输出（13 通道：ref/meas/duty/yaw），供 VOFA+ 画波形 |
| 0.5 | 2025-10-03 | PID 参数改为每轮独立（Kp/Ki/Kd/I_MAX × FL/FR/BL/BR），OUT_MAX 保持四轮共用 |
| 0.6 | 2025-10-03 | 依据 test_data/20251003_pid_v05_6min.csv 整定：加速度前馈 Kff（解决稳态差一截），BR 单独钝化（Kp 2.5/Kd 0.15/Kff 3.0 抑制振荡） |
| 0.7 | 2025-10-06 | 适配重新布线：PWM 改为 TIM5_CH2/3/4 + TIM9_CH1，编码器改为 TIM1/3/4/8（全部 TI12 四倍频），方向引脚换新；ENC_DIR 重置为 +1 待标定；§8 绑定表与 §7 配置表同步更新 |
| 0.8 | 2026-10-09 | 仓库纳入硬件资料：新增 `hardware/` 目录（扩展板原理图 PDF + 嘉立创EDA 工程文件），根目录 `.gitignore` 增加 EDA 自动备份目录（`*_backup/`）与系统垃圾文件的忽略规则。**本库代码本次无改动** |
| 0.9 | 2026-10-18 | 新增 §15 CAN 通信与点灯验证：`can_test` 模块（心跳 `0x7F0` / 点灯 `0x200`·`0x201` / 回显 / silent-loopback 自检 / TEC·REC·LEC 诊断）；修复 `.ioc` 位时序（875 kbps × 3 TQ → 1 Mbps × 14 TQ）；`main.c` 调试期注释掉 `chassis_poll()` / `chassis_debug_poll()`。F407 侧全链路已验证合格，卡点在上位机 CAN 适配器的 Linux 支持（选型结论见 §15.8） |
| 0.10 | 2026-10-18 | 新增 §16：确立 Jetson `~/gimbal` 为**上位机正式工程**，记录系统分工、`IChassisLink` 对接点、预留协议（`0x020`/`0x000`/`0x110`）、总线波特率待决事项（500k vs 1M，两侧当前不一致）；**CAN 通信方向暂定暂停**，待适配器选型确定后按 §16.12 的待办恢复 |
| 0.11 | 2026-10-18 | **重写 §16 为上位机正式工程完整文档**：确立 Jetson `~/gimbal` 为上位机正式工程，补全工程规模（21 源文件 / 3186 行）、五层架构与单拍数据流、ZDT Emm5.0 协议要点（功能码 / 状态位 / Emm-vs-X 单位差异 / 单圈回读局限）、六条关键设计决策（12 ms 节流 · urgent 插队 · enable 幂等 · 看门狗不失能 · 独立计时 · Disable 标志）、UDP 线协议完整字段表（上行 16 B / 下行 32 B / flags / status）、传动比 2:1 换算、构建运行与全部命令行参数、9 条开发踩坑、验证边界声明；**新发现 `ChassisCommand` 的 vx/vy 方向与底盘库 §4 不一致**（右 / 前 vs 前进 / 左，属交换 + 一项反向）；原 §16.2~§16.5 顺延为 §16.9~§16.12 |
| 0.12 | 2026-10-18 | **四轮速度环实机整定**：①修复 FR/BR 编码器方向（ENC_DIR = +1/−1/+1/−1）；②编码器输入数字滤波（IC1F/IC2F=8，写在 `encoder.c` 里防 CubeGen 覆盖），FR 一路幽灵计数 146 次/8000 采样 → 0；③PID 重构：速度前馈 Kff 移入 `pid_calc`、新增参考加速度前馈 Kacc、条件积分抗饱和、测量微分一阶低通；④Kd 归零（噪声放大）、Ki 0.6→40（原来等于没积分）；⑤新增 §6.8 `chassis_tune` 整定模块（静态标定/斜升扫参/转向目检/惯性滑行自证 4 个实验，二进制 RAM 回传）；⑥新增 `tools/step_analysis.py`；⑦`CHASSIS_MAX_WHEEL_RPM` 280→250（实测满占空比仅 260~296RPM）、`I_MAX` 900→20；⑧`main.c` 恢复 `chassis_poll()`，演示/整定改为编译开关。实测：120 RPM 稳态误差 <1 RPM、斜坡超调 5~11%、25 RPM 低速仍稳；完整演示前进段 yaw 漂移 0.1° |
| 0.13 | 2026-10-18 | **新增 §17 IMU 与陀螺偏航闭环（小陀螺）**：①`UserLib/IMU/`（ST 官方 `lsm6dsv16x_reg` 驱动 + `imu.c/h`：I2C1 400k、地址双探测、±1000dps/960Hz、上电静止零偏标定 + 静止慢速跟踪、40Hz 软件低通、yaw 积分、总线卡死自动恢复）；②`chassis_gyro.c/h`：陀螺外环 PI（斜坡/大误差双冻结、能力带、条件积分），三种模式 RATE/SPIN/YAW_TARGET；③`chassis.c` 接入外环并新增 `chassis_state()` 的 `imu_wz/imu_yaw/wz_eff/imu_online`；④JustFloat 13→17 通道、RawText 加 imu 字段；⑤`chassis_tune` 新增实验 5（陀螺自旋实测，8 通道二进制回传）；⑥演示新增小陀螺段；⑦`chassis_spin()` / `chassis_imu_online()` 接口。闭环逻辑已用 PC 对象仿真验证（打滑 18% 稳态误差 0.003 rad/s、无超调），**实机数据待补** |
| 0.14 | 2026-10-18 | **文档补齐 + IMU 链路定型**：①新增 `UserLib/IMU/README.md`（定位/接口速查/配置表/上电时序/实测指标/六个坑）；②§17 整体重排：修正子标题编号（18.x → 17.x）、合并「软件 I2C 打通后」与「硬件 I2C」两张实机表、补 §17.6 噪声实测与上电瞬态、§17.7 坑表补到 9 条（飞线修复 SDA/SCL 交叉、软件 I2C 用 BSRR+DWT 提速、上电瞬态挡住标定、飞线后 SWD 降速）；③`IMU_STARTUP_SETTLE_MS=1200` 把零偏标定延后到启动瞬态之后；④I2C1 400 kHz 写进 `.ioc` + `i2c.c` |
| 0.15 | 2026-10-18 | **IMU 掉线健壮性 + 文档定稿**：①修复实测发现的「运行中掉线」（HAL I2C 状态机/总线卡死）：恢复流程改为 DeInit → GPIO 打 9 个时钟 → STOP → 重 Init，并限流 100 ms；②新增 `healthy` 判据（连续 10 次读失败 = 掉线），陀螺外环自动退回纯运动学、读通自动接回；③串口加 `e=` 错误计数与 `STALE` 状态；④`docs/CHASSIS_LIBRARY.md` §17 补「掉线与恢复」「漂移实测（130s，<1°）」、坑表补到 10 条；⑤新增 `UserLib/IMU/README.md`（含 §10「为什么没加互补滤波/卡尔曼」的量化取舍） |
| 0.16 | 2026-10-19 | **定位并修复「电机一卡一卡」**：IMU 运行期 I2C 地址应答不稳（约一半读在地址阶段被 NACK，`us≈27µs`）→ 频繁触发阻塞式总线恢复（25ms/100ms）+ 部分 3ms 超时 → 1kHz 控制环掉拍 30~40%、反馈 57Hz。修复：①`imu.c` 读失败换 0x6A/0x6B 地址重试并锁定（`e` 150万→0）；②`hw_bus_unstick` 的 `HAL_Delay(1)` 改 10µs DWT 忙等（25ms→0.24ms）；③`chassis_debug` 改非阻塞 `Transmit_IT` 并补 USART1 NVIC/IRQHandler；④`uart_link.c` TxCplt 转发。验证：反馈 100Hz（200/201 CRC 全对）、IMU `CAL=1`、`e=0`。详见 §17.8 |
| 0.17 | 2026-10-19 | **SA0 焊死 + 总线诊断字段 + soak 复验**：①定位根因 = **SA0 弱上拉**（板载 100k 把地址吊在临界值，运行中在 0x6A/0x6B 间漂移 → 地址字节被 NACK，`us≈27µs`），**直连 GND** 后地址固定 6A；②`imu_state` 增 `err_per_s`/`addr_switches`/`recover_count`，串口加 `eps= ad= sw= rec=`（一行判断总线状态）；③地址热切换成功后同步 `i2c_addr` 并计数，开机探测的切换不计入；④20 分钟 soak：4801 帧全绿（`ad=6A`、`sw=0`、`rec=0`、`eps=0`、`us` 恒 214），对比焊死前同条件 144 次错误/一次 0.7s 爆发 |
| 0.18 | 2026-10-19 | **新增 §18 USART2 上位机控制链路**：完整记录 `uart_link` 的 12 B 命令帧 / 40 B 反馈帧字段表、命令字（SPEED/ESTOP/SPIN/CLEAR）、`status` 位、看门狗三级约定（100 ms 清速度不失能 / 500 ms 置 LINK_LOST / ESTOP 才断 STBY）、统计字段含义、与底盘库的单位与坐标约定（⚠️ 与 §16.10 的 Jetson `ChassisCommand` 约定不同）、PC 端验证工具与 5 条踩坑；§19 顺延为版本记录 |
