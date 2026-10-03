# 底盘基础库设计文档（Chassis Library）

> 本文档是 `UserLib/Chassis/` 底盘基础库的**唯一权威说明**。
> 接线信息仍以项目根目录 `README.md` 为准，本文档不重复接线表，只引用。

- 适用平台：STM32F407VGTx + TB6612 ×2 + 亚博智能 L 型 520 编码器减速电机 ×4（麦克纳姆轮底盘）
- 电机参数：减速比 1:40，磁环 11 线，12V 供电，减速后空载约 300 RPM（±5%）
- 控制方式：四轮速度闭环 PID + 里程计，裸机 1 kHz 周期调度

---

## 1. 概述

本库把底盘控制拆成「测量 / 控制 / 运动学」三层，目标是与 RM（RoboMaster）开源工程同等的工程化水平：

- **测量层**：TIM 编码器模式采样，16 位计数器溢出拼接为 32 位累计，滑动窗口测速；
- **控制层**：车体速度指令 → 麦克纳姆逆解 → 每轮斜坡限幅 → 速度环 PID → TB6612 占空比；
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
├── encoder.h/.c        # 编码器抽象（累计计数 + 窗口测速）
├── pid.h/.c            # 位置式 PID（积分/输出限幅，微分作用于测量值）
├── mecanum.h/.c        # 麦克纳姆运动学正解/逆解
├── chassis.h/.c        # 主模块：硬件绑定、1 kHz 控制循环、里程计
├── chassis_demo.h/.c   # 演示状态机（可整文件删除）
└── chassis_debug.h/.c  # USART1 调试打印（可用 CHASSIS_DEBUG_ENABLE 关闭）
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
           │ 轮速限幅 ±280RPM         │ 方向归一(ENC_DIR)
           │ 斜坡 ±1500RPM/s          ▼
           ▼                   wheel_rpm_meas[4]
      wheel_rpm_ref[4]                │
           │                          │
           ▼                          ▼
      pid_calc()            mecanum_forward()
      （速度环，每轮一个）    实测轮速 → vx/vy/wz
           │                          │
           ▼                          ▼
      motor_set_duty()         里程计积分 (x, y, yaw)
      （±1000 → TB6612 IN1/IN2 + PWM CCR）
```

整条链以 1 kHz 执行一次：`chassis_poll()` 内部检测 `HAL_GetTick()` 的 1 ms 节拍，到点跑一拍。**主循环里不能再用 `HAL_Delay()` 长时间阻塞**，否则控制会掉拍。

## 6. 模块说明

### 6.1 motor — TB6612 电机抽象

- `motor_init()`：绑定定时器/通道与 IN1/IN2 引脚，启动 PWM 并清零输出；
- `motor_set_duty(motor, duty)`：`duty ∈ [-1000, +1000]`（千分比）。
  - `duty > 0`：IN 电平 = `fwd_in1/fwd_in2`；
  - `duty < 0`：两电平互换；
  - `duty == 0`：IN1=IN2=0（**滑行停止**，非刹车）。

### 6.2 encoder — 编码器

- `encoder_start()`：以 TIM 编码器模式启动（x4 计数，1760 counts/轮圈）；
- `encoder_update()`：每控制周期采样一次：
  - 增量 = 无符号差值截断为 int16，天然处理 16 位溢出回绕（前提：单周期增量 < 32767，本配置 1 kHz 下余量极大）；
  - 累计到 32 位 `total`，同时送入滑动窗口；
- `encoder_speed_rpm()`：窗口内增量求和 → RPM。窗口默认 10 ms，低转速分辨率约 5.7 RPM；窗口越大越平滑、延迟越大（改 `CHASSIS_SPEED_WINDOW_MS`）。

### 6.3 pid — 位置式 PID

- 积分限幅抗饱和，输出限幅；
- 微分作用于**测量值**（`-kd·d(meas)/dt`），给定值突变不会产生微分冲击；
- 输出单位：占空比千分比。

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

### 6.6 chassis_debug — 调试输出（RawData / JustFloat 双协议）

`CHASSIS_DEBUG_ENABLE=1` 时，通过 USART1（PA9/PA10，115200-8-N-1）每 250 ms 输出一帧。协议由 `CHASSIS_DEBUG_PROTOCOL` 选择：

**RawData 文本**（`CHASSIS_DEBUG_PROTO_RAWTEXT`，终端人眼直读）：

```
t=12345 | FL  119/ 350 FR  119/ 360 BL  119/ 340 BR  119/ 355 | yaw=12deg
```

含义：`t` 上电毫秒数；`FL/FR/BL/BR` 后两个数字分别是**实测轮速 (RPM) / 输出占空比 (千分比)**；`yaw` 里程计偏航角。

**JustFloat 二进制**（`CHASSIS_DEBUG_PROTO_JUSTFLOAT`，VOFA+ 画波形用）：
帧格式 = 13 通道小端 float32 + 帧尾 `00 00 80 7F`（官方协议）。注意二进制模式下终端文本区看不到内容，属正常。

通道表（CH_COUNT = 13）：

| 通道 | 含义 | 通道 | 含义 |
|---|---|---|---|
| ch0~3 | 目标轮速 ref：FL FR BL BR (RPM) | ch8~11 | 输出占空比 duty：FL FR BL BR (‰) |
| ch4~7 | 实测轮速 meas：FL FR BL BR (RPM) | ch12 | 里程计 yaw (deg) |

VOFA+ 用法：新建数据引擎选 **JustFloat**（1.4.5 已内置插件，无需手动安装），
拖入 JustFloat 控件，把曲线勾成 `ch0`（FL 目标）vs `ch4`（FL 实测）这类对比即可整定 PID。

### 6.7 chassis_demo — 演示

上电 1.5 s 后自动循环执行：前进 → 左移 → 原地逆时针转 90°（里程计闭环）→ 后退 → 右移 → 原地顺时针转回 90° → 静止 3 s → 重复。
正式比赛代码中删除本文件及其调用即可。

## 7. 参数配置（`chassis_config.h`）

| 宏 | 当前值 | 说明 |
|---|---|---|
| `CHASSIS_WHEEL_RADIUS_M` | 0.040 | 轮径 80mm 实测 |
| `CHASSIS_HALF_WHEELBASE_M` | 0.1001 | 轴距 200.20mm / 2 |
| `CHASSIS_HALF_TRACK_M` | 0.120 | 轮距 240.00mm / 2 |
| `CHASSIS_ENCODER_CPR` | 1760 | 11线×4倍频×40减速比，**待实测校准** |
| `CHASSIS_MAX_WHEEL_RPM` | 280 | 12V/1:40 空载约 300RPM，留余量 |
| `CHASSIS_WHEEL_ACCEL_RPM_S` | 1500 | 轮速斜坡限幅 |
| `CHASSIS_MAX_VX_M_S` / `_VY_` | 1.0 | 车体线速度限幅 |
| `CHASSIS_MAX_WZ_RAD_S` | 4.0 | 车体角速度限幅 |
| `CHASSIS_ROLLER_SIGN` | +1 | 辊筒朝向（见 §10.3） |
| `CHASSIS_CONTROL_PERIOD_S` | 0.001 | 控制周期 1 kHz |
| `CHASSIS_SPEED_WINDOW_MS` | 10 | 测速滑动窗口 |
| `CHASSIS_PID_{KP,KI,KD,I_MAX,KFF}_{FL,FR,BL,BR}` | 3/0.6/0.08/900/4.0 | 速度环增益，**每轮独立**，控制律 `duty = Kp·e + Ki·∫e − Kd·d(meas)/dt + Kff·ref`，需整定 |
| `CHASSIS_PID_OUT_MAX` | 1000 | 输出限幅（占空比满幅，四轮共用） |
| `CHASSIS_ENC_DIR_FL/FR/BL/BR` | +1/−1/+1/−1 | 每轮编码器测量方向归一（见 §10.2）。右两轮镜像安装已默认取反 |
| `CHASSIS_DEBUG_ENABLE` | 1 | USART1 调试输出总开关，正式使用改 0 |
| `CHASSIS_DEBUG_PROTOCOL` | JustFloat | 调试协议：RawData 文本 / JustFloat 二进制（VOFA+ 数据引擎要对应） |

## 8. 硬件绑定表（`chassis.c` 中 `s_bind[]`）

依据 README 接线与「A=后轮 / B=前轮」确认结果：

| 轮 | PWM | 方向引脚 | 编码器 |
|---|---|---|---|
| FL 左前 | TIM1_CH4 (PE14, L_PWMB) | L_BIN1(PE7) / L_BIN2(PE8) | TIM3 (PA6/PA7) |
| FR 右前 | TIM9_CH2 (PE6, R_PWMB) | R_BIN1(PD14) / R_BIN2(PD15) | TIM2 (PA0/PA1) |
| BL 左后 | TIM1_CH1 (PE9, L_PWMA) | L_AIN1(PE10) / L_AIN2(PE11) | TIM4 (PD12/PD13) |
| BR 右后 | TIM9_CH1 (PE5, R_PWMA) | R_AIN1(PD10) / R_AIN2(PD11) | TIM8 (PC6/PC7) |

`fwd_in1/fwd_in2` 为该轮「正转」时 TB6612 IN1/IN2 电平，源自原始方向标定（并已包含右后轮刹车 bug 的修正）。**某轮实际转向反了，把该轮两个电平对调即可。**

## 9. 集成方式

- `CMakeLists.txt`：库源文件已加入 `target_sources`，头文件路径已加入 `target_include_directories`（新增用户文件时照此追加）；
- `main.c`：仅两处——
  ```c
  chassis_init();
  chassis_demo_init();
  ...
  while (1) {
      chassis_poll();
      chassis_demo_poll();
  }
  ```
- 调度源为 `HAL_GetTick()`（SysTick 1 kHz），**不占用任何中断**，CubeMX 重新生成代码时不会破坏集成（所有改动都在 USER CODE 区内）。

## 10. 首次上电校准（务必按顺序做）

### 10.1 编码器 CPR 实测

1. 用手推动车体直线前进一整圈（任一轮），记录 `chassis_state()->wheel_count[轮]` 变化；
2. 应为 1760 左右，把实测值写回 `CHASSIS_ENCODER_CPR`。
   （调试时可用调试器读 `wheel_count[]`，或临时用串口打印。）

### 10.2 每轮方向标定（电机方向 与 编码器方向 分开修）

> **架空四轮再上电**，打开 §6.6 的调试打印（USB-TTL 接 PA9/PA10，115200）。

运行 DEMO 前进段，看打印行判断：

1. **某轮实测 RPM 为负、且该轮占空比顶在 ±1000 附近**（飞转）→ 该轮编码器符号反了：
   把 `chassis_config.h` 里对应 `CHASSIS_ENC_DIR_x` 取反；
2. **某轮转向与其余三轮相反**（肉眼看）→ 该轮电机方向反了：
   去 `chassis.c` 的 `s_bind[]` 对调该轮的 `fwd_in1/fwd_in2`；
3. 正常状态：四轮 RPM 数值接近（前进段都约等于目标轮速）、占空比不饱和、yaw 基本不变。

> 为什么必须分开修：电机方向（TB6612 接线）和编码器方向（编码器 AB 线）是两套独立线缆，
> 可能只反一个。若只把电机线反了而编码器没反，闭环会把该轮推到全速——就是「一侧明显更快」现象的来源。

### 10.3 运动学符号（`CHASSIS_ROLLER_SIGN`）

DEMO 前进/左移/旋转三段分别验证：

- 前进跑偏（车身斜着走）→ 先回 10.2 查方向；
- 前进直、**左移变成右移**（或旋转方向反了）→ 把 `CHASSIS_ROLLER_SIGN` 取反，重测；
- 旋转方向反 → 同上（该宏同时翻转 vy 与 wz 两项符号）。

### 10.4 死区与最低占空比

TB6612 + 减速电机存在启动死区，低速指令下车可能不转或微颤，属正常。速度环积分会补偿一部分；如低速性能要求高，可在 `motor_set_duty` 中加死区补偿（本文档暂不展开）。

## 11. PID 整定建议

> 四轮 PID **相互独立**（`CHASSIS_PID_*_{FL,FR,BL,BR}`），先四轮同参整定，个别轮单独微调。
> 控制律：`duty = Kp·e + Ki·∫e − Kd·d(meas)/dt + Kff·ref`。

1. **先整 Kff（前馈）**：Kff 决定"给多少占空比能到目标转速"。前进段实测与目标差一大截、
   占空比又没饱和 → 加大该轮 `KFF`（步进 0.5）；一给指令就冲过目标 → 减小 `KFF`；
2. 稳态仍有小误差 → 加大该轮 Ki（步进 0.2）或 `I_MAX`；
3. 某轮转速绕目标来回摆动（尖刺）→ 减小该轮 Kp（步进 0.5）、加大 Kd（步进 0.03）；
4. 差一大截且占空比顶 1000 → 物理极限（负载/供电/堵转），调参无用；
5. 整定后重新标定 `CHASSIS_MAX_*` 限幅，避免满占空比削顶。
6. 整定请用满电电池/稳压电源，实测长时间运行占空比会随电池电压漂移（见 test_data 记录）。

## 12. 演示程序说明

- 上电 1.5 s 后开始（留出把车放到地上的时间）；
- 两个旋转段用**里程计 yaw** 闭环，转满 90° 即进入下一段——同时验证了运动学正解与编码器方向；
- 若某段行为与预期相反，按 §10 顺序排查，不要改演示逻辑本身。

## 13. 已知限制与后续计划

- 里程计仅由轮速积分，打滑会累积漂移；后续可接 I2C 陀螺仪（本工程 I2C1 已就绪）做 IMU 融合；
- 未做电流环（TB6612 无电流采样），堵转保护靠限幅 + 后续可加堵转检测；
- 未做 UART 遥控指令接口，后续可仿 RM 加 `vx vy wz` 串口解析；
- 电机制动未用短路刹车（停止为滑行），如需刹车可扩展 `motor_brake()`。

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

## 15. 版本记录

| 版本 | 日期 | 说明 |
|---|---|---|
| 0.1 | 2025-10-03 | 初版：motor/encoder/pid/mecanum/chassis/demo，速度闭环 + 里程计，裸机 1 kHz |
| 0.2 | 2025-10-03 | 拆分电机方向与编码器方向归一；新增 chassis_debug 串口调试打印；右两轮 ENC_DIR 默认取反 |
| 0.3 | 2025-10-03 | 依据串口实测数据整定：Kp 5→3、Ki 0.5→0.6、Kd 0→0.08、I_MAX 300→900（解决到不了目标转速与右后轮振荡） |
| 0.4 | 2025-10-03 | chassis_debug 增加 JustFloat 二进制协议输出（13 通道：ref/meas/duty/yaw），供 VOFA+ 画波形 |
| 0.5 | 2025-10-03 | PID 参数改为每轮独立（Kp/Ki/Kd/I_MAX × FL/FR/BL/BR），OUT_MAX 保持四轮共用 |
| 0.6 | 2025-10-03 | 依据 test_data/20251003_pid_v05_6min.csv 整定：加速度前馈 Kff（解决稳态差一截），BR 单独钝化（Kp 2.5/Kd 0.15/Kff 3.0 抑制振荡） |
