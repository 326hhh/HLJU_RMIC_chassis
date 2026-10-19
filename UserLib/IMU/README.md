# IMU —— LSM6DSV16X 驱动与姿态数据

> 本文与代码同步于 2026-10-18。引脚/接线以 [`../../README.md`](../../README.md) 为准，
> 陀螺外环（小陀螺）的原理与实测数据以 [`../../docs/CHASSIS_LIBRARY.md`](../../docs/CHASSIS_LIBRARY.md) §17 为准。

## 1. 定位

把 IMU 包成**"车体角速度 + 姿态"的黑箱**：

- 输入：无（自己轮询 I2C）
- 输出：车体系三轴角速度（rad/s，已去零偏、已低通）、加速度（m/s²）、陀螺积分的 yaw 角
- 内部自己解决：I2C 事务、地址探测、软复位与量程配置、上电零偏标定、静止零偏跟踪、轴映射、yaw 积分、总线卡死恢复

**只输出数据，不做控制**。控制（陀螺外环 / 小陀螺 / 云台跟随）在 `../Chassis/chassis_gyro.c` 里，
两层之间只有一个 `imu_state()` 结构体。

## 2. 快速接入

正常不用单独调用——`chassis_init()` 里已经帮你初始化了。要单独用：

```c
#include "imu.h"

imu_init();                    /* 探测 + 配置 + 零偏标定，阻塞约 1.8s，只在初始化阶段调 */

while (1) {
    imu_poll(HAL_GetTick());   /* 主循环高频调用，内部按 2ms 节拍限速 */
    const imu_state_t *imu = imu_state();
    /* imu->gyro_lpf[2] 就是 wz；imu->yaw_rad 是积分角 */
}
```

> ⚠️ 主循环里不要有长阻塞。`imu_poll()` 到点才读一次（213 µs @400 kHz），
> 阻塞多久就丢多少样本；`dt` 会被限幅到 50 ms 兜底。

## 3. 目录结构与文件详解

### 3.1 目录结构

```
UserLib/IMU/
├── imu_config.h          # 所有可调参数：总线/地址/速率/量程/滤波/零偏/轴映射/软件 I2C 兜底
├── imu.h                 # 对外接口 + imu_state_t 状态快照
├── imu.c                 # 驱动实现：平台层（硬件/软件 I2C 双后端）、配置、采样、零偏、轴映射、yaw 积分
├── lsm6dsv16x_reg.h      # ST 官方驱动头文件：寄存器地址定义 + 全部配置函数原型
├── lsm6dsv16x_reg.c      # ST 官方驱动实现：寄存器读写、量程/ODR/滤波配置、FIFO 等（原样拷贝，不手改）
└── README.md
```

### 3.2 逐个文件详解

#### `imu_config.h` —— 参数总表（唯一常改的文件）

纯宏定义，分七组：

- **总线**：`IMU_I2C_HANDLE`（hi2c1）、SA0 的两种 7 位地址（0x6A/0x6B）与是否都探测、
  单次事务超时 3 ms、连续错误恢复/掉线阈值（5 / 10 次）、恢复最小间隔 100 ms、硬件 I2C 引脚（PB6/PB7）。
- **软件 I2C 兜底**：`IMU_USE_SOFT_I2C`（0=硬件 / 1=软件，SCL=PB7、SDA=PB6 角色可配）、
  半周期 1.3 µs、DWT 不可用时退化空转次数。
- **传感器**：陀螺 ODR 960 Hz、加速度 ODR 120 Hz、陀螺量程 ±1000 dps、加速度 ±4 g、片内 LPF1 档位。
- **软件采样**：陀螺 2 ms（500 Hz，100 kHz 硬件 I2C 时代码自动 +1）、每 10 个陀螺样本读一次加速度、
  软件一阶低通 40 Hz。
- **零偏标定**：上电静置 1200 ms、标定点数 300、标准差上限 0.05 rad/s（超了判"车在动"）、
  静止跟踪时间常数 4 s / 最大可跟踪角速度 0.2 rad/s。
- **轴映射**：`IMU_MAP_X/Y/Z`（编码 = 符号 × (芯片轴号+1)），模块装反了只改这里。

#### `imu.h` —— 对外接口

- `imu_state_t`：健康状态（`online/healthy/whoami/i2c_addr/bias_calibrated/sample_count/error_count/
  sample_hz/dt_s/read_us`）、角速度（`gyro_rps` 未滤波 / `gyro_lpf` 控制用 / `gyro_bias` 零偏估计）、
  加速度（`accel_mps2`）、积分 yaw（`yaw_rad`）。
- 接口：`imu_init()` / `imu_poll()` / `imu_state()` / `imu_is_online()` / `imu_calibrate_bias()` /
  `imu_track_bias()` / `imu_reset_yaw()` / `imu_bus_recover()` / `imu_body_to_chip()`。

#### `imu.c` —— 驱动实现

- **平台层（两种后端，二选一）**：
  - **硬件 I2C**：把 ST 官方驱动的 `read_reg/write_reg` 回调翻译成 `HAL_I2C_Mem_Read/Mem_Write`
    （7 位地址 + 寄存器号，注意 I2C 模式不给寄存器地址置 0x80 多位读标志，靠 IF_INC 自动递增）；
  - **软件 I2C**：GPIO 位翻转 + DWT 计时做延时（BSRR 直写，`HAL_GPIO_WritePin` 太慢），
    用于"扩展板 H5 SDA/SCL 交叉"的急救场景。
  - 两套后端都只暴露 `plat_write/plat_read`，上层完全无感。
- **总线恢复**：连续 5 次失败 → `imu_bus_recover()`：DeInit 外设 → 引脚当 GPIO 打 9 个时钟
  （从机把 SDA 拉死时解锁）→ 重新 Init；两次恢复之间限流 100 ms。连续 10 次失败 → `healthy=false`，
  外环自动退回纯运动学。
- **`imu_init()`**：探测 0x6A/0x6B（读 `WHO_AM_I`==0x70）→ `sw_reset` → 配置
  BDU/IF_INC/量程/ODR/片内 LPF1 → `HAL_Delay(IMU_STARTUP_SETTLE_MS)` 等启动瞬态 →
  `imu_calibrate_bias(300)` 静止标零偏（约 0.6 s）。
- **`imu_poll()`**：按 `IMU_GYRO_PERIOD_MS` 节拍读 `OUTX_L_G` 6 字节（DWT 记一次读耗时 `read_us`）→
  换算 rad/s → 减零偏 → 轴映射到车体系 → 40 Hz 一阶低通（`gyro_lpf`）→ yaw 积分（±π 归一）；
  每 `IMU_ACCEL_EVERY_N` 个样本读一次 `OUTX_L_XL` 更新加速度。
- **`imu_track_bias()`**：由 `chassis_gyro` 在"车确实静止"时调用，残余零偏按 τ=4 s 往 0 拉，
  |gyro| 超 0.2 rad/s 就不动。
- **辅助**：`imu_reset_yaw()` 设置积分角、`imu_body_to_chip()` 车体系 → 芯片轴序（调试打印用）。

#### `lsm6dsv16x_reg.h` / `lsm6dsv16x_reg.c` —— ST 官方驱动（原样拷贝）

- 头文件：LSM6DSV16X 全部寄存器地址宏、位域定义、`stmdev_ctx_t` 回调结构、
  量程/ODR/FIFO/中断等配置函数原型（约 8400 行）。
- 实现：官方驱动函数体（约 8800 行），读写寄存器、设置量程与 ODR、配置滤波/FIFO 等，
  全部通过 `stmdev_ctx_t` 的 `read_reg/write_reg` 回调访问总线。
- ⚠️ **不要手改**。`imu.c` 只通过 `platform_init()` 注入回调后调用官方 API；
  需要换量程/速率时改 `imu_config.h`，不要动这份文件。

## 4. 数据流

```
imu_init()
  ├─ 地址探测（0x6A / 0x6B 都试，读 WHO_AM_I == 0x70）
  ├─ sw_reset → BDU / IF_INC / ±1000dps / 960Hz / 片内 LPF1
  ├─ HAL_Delay(IMU_STARTUP_SETTLE_MS)      ← 等上电瞬态过去（1.2s）
  └─ imu_calibrate_bias(300)               ← 静止标定 0.6s

imu_poll()（跟着 1kHz 控制环调，内部每 2ms 真正读一次）
  ├─ 读 OUTX_L_G 6 字节（DWT 记时 → read_us）
  │    └─ 读失败 → 立刻换另一个地址（0x6A↔0x6B）再试一次，成功就锁定并计入 addr_switches
  ├─ 换算 rad/s → 减零偏 → 轴映射到车体系 → 40Hz 一阶低通（gyro_lpf）
  ├─ yaw_rad += gyro_lpf[2] * dt（±PI 归一）
  └─ 每 10 个样本读一次 OUTX_L_XL（加速度，只做参考）

imu_track_bias()（由 chassis_gyro 在「车确实静止」时调用）
  └─ 残余零偏按 τ=4s 往 0 拉，|gyro| 超 0.2rad/s 就不动
```

## 5. 接口速查

| 函数 | 作用 | 备注 |
|---|---|---|
| `bool imu_init(void)` | 探测 + 配置 + 零偏标定 | 阻塞约 1.8 s，只在初始化调 |
| `void imu_poll(uint32_t now_ms)` | 采样（内部限速） | 传 `HAL_GetTick()` |
| `const imu_state_t *imu_state(void)` | 只读状态 | 见下表 |
| `bool imu_is_online(void)` | 是否在线（未初始化） | |
| `imu_state()->healthy` | **数据是否可信**（最近 10 次读都通） | 掉线时陀螺外环自动退回纯运动学，读通了自动接回 |
| `bool imu_calibrate_bias(uint16_t n)` | 静止重标定 | 阻塞 `n×2ms`，车必须静止 |
| `void imu_track_bias(bool stationary, float dt)` | 静止慢速跟零偏 | 由 `chassis_gyro.c` 调，一般不用管 |
| `void imu_reset_yaw(float rad)` | 把积分 yaw 设成指定值 | 角度环对齐用 |
| `void imu_bus_recover(void)` | 总线恢复 | 连续 5 次读失败时自动调一次 |
| `void imu_body_to_chip(const float[3], float[3])` | 车体系 → 芯片轴序 | 调试打印用 |

`imu_state_t` 关键字段：

| 字段 | 含义 |
|---|---|
| `online` / `healthy` | `online` = 初始化时探测到；`healthy` = 最近在读得通（掉线判据见 §9） |
| `whoami` / `i2c_addr` | WHO_AM_I（应为 0x70）/ HAL 8 位地址（0xD6 = 0x6B<<1） |
| `gyro_rps[3]` | 车体系角速度 (rad/s)，已去零偏，**未滤波** |
| `gyro_lpf[3]` | 同上但过了 40 Hz 软件低通 —— **控制用这个**（`gyro_lpf[2]` 就是 wz） |
| `gyro_bias[3]` | 当前零偏估计 (rad/s) |
| `accel_mps2[3]` | 车体系加速度 (m/s²)，平放时 z ≈ 9.81 |
| `yaw_rad` | 陀螺积分偏航角 (-PI..PI)，逆时针为正 |
| `read_us` / `sample_hz` / `dt_s` | 一次读耗时 / 实测采样率 / 最近采样间隔，看总线负载体检用 |
| `sample_count` / `error_count` | 累计采样点数 / 累计 I2C 错误数 |
| `err_per_s` | **最近 1 秒新增错误数**（比累计值好用，一眼判断总线当前状态） |
| `addr_switches` / `recover_count` | 运行中自动换地址（0x6A↔0x6B）次数 / 总线解锁次数 |
| `bias_calibrated` | 上电标定是否可信（车在动时会置 false） |

## 6. 配置开关（`imu_config.h`）

| 宏 | 当前值 | 说明 |
|---|---|---|
| `IMU_USE_SOFT_I2C` | **0** | 0 = 硬件 I2C1（PB6/PB7，213 µs/读）；1 = 软件 I2C 兜底（351 µs/读） |
| `IMU_ADDR_SA0_LOW/HIGH` | 0x6A / 0x6B | SA0 接 GND / VCC 的两种 7 位地址 |
| `IMU_PROBE_BOTH_ADDR` | 1 | 两个地址都试 |
| `IMU_I2C_TIMEOUT_MS` | 3 | 单次 I2C 事务超时 |
| `IMU_ERR_RECOVER_COUNT` | 5 | 连续错这么多次就解锁总线 + 重新初始化外设 |
| `IMU_ERR_STALE_COUNT` | 10 | 连续错这么多次判「掉线」（20 ms），外环退回纯运动学 |
| `IMU_RECOVER_MIN_MS` | 100 | 两次总线恢复的最小间隔（限流） |
| `IMU_HW_PIN_SCL/SDA` | PB6 / PB7 | 硬件 I2C 引脚；解锁总线时临时当 GPIO 打 9 个时钟用 |
| `IMU_ODR_GYRO` / `_ACCEL` | 960 Hz / 120 Hz | 陀螺 ODR 必须 ≥ 2×软件采样率 |
| `IMU_FS_GYRO` / `_ACCEL` | ±1000 dps / ±4 g | 自旋上限 4.76 rad/s = 273 dps，留 3.6 倍余量 |
| `IMU_GY_LPF_BW` | `GY_LIGHT` | 片内 LPF1 档位 |
| `IMU_GYRO_PERIOD_MS` | 2 | 软件采样周期（500 Hz）；100 kHz 硬件 I2C 时自动 +1 |
| `IMU_ACCEL_EVERY_N` | 10 | 每 10 个陀螺样本读一次加速度 |
| `IMU_GYRO_LPF_HZ` | 40 | 软件一阶低通截止频率 |
| `IMU_STARTUP_SETTLE_MS` | 1200 | 上电静置多久才标零偏（躲开启动瞬态） |
| `IMU_BIAS_CALIB_SAMPLES` | 300 | 标定点数（0.6 s） |
| `IMU_BIAS_CALIB_STD_MAX` | 0.05 | 标定窗口标准差超它就判「车在动」 |
| `IMU_BIAS_TRACK_TAU_S` / `_MAX` | 4.0 / 0.20 | 静止跟零偏的时间常数 / 允许跟踪的最大角速度 |
| `IMU_MAP_X/Y/Z` | +1 / +2 / +3 | 轴映射（编码 = 符号 × (芯片轴号+1)） |
| `IMU_SOFT_PIN_SCL/SDA` | PB7 / PB6 | 软件 I2C 的引脚角色（与硬件相反） |
| `IMU_SOFT_HALF_US` | 1.3 | 半个 SCL 周期（µs），实际约 230 kHz |

## 7. 上电时序（约 1.8 秒）

```
t=0     上电
t≈0.1s  MX_I2C1_Init()（硬件 I2C 模式）
t≈0.2s  探测 WHO_AM_I → sw_reset → 配置量程/速率/滤波
t≈1.4s  HAL_Delay(1200) 结束：启动瞬态已过去
t≈2.0s  零偏标定 300 点完成 → online=true → 控制环开始能用陀螺
```

> 为什么等 1.2 s：实测上电前 ~5 s 陀螺 x/y 有明显瞬态（gx 峰峰 11 dps、acc_z std 33 mm/s²），
> 0.7 s 就标定会被污染。gz 受影响小，但既然等得起就等。

## 8. 实测指标（2026-10-18，硬件 I2C 400 kHz）

| 项 | 值 | 说明 |
|---|---|---|
| 握手 | `imu ok`，WHO_AM_I=0x70，地址 **0x6A** | SA0 已焊死到 GND（2026-10-19） |
| 采样率 | `hz=499` | 2 ms 一拍 |
| 一次读耗时 | **213 µs**（软件 I2C 351 µs） | 约占 1 ms 控制拍的 10.6% |
| 陀螺噪声 `gz` | std **0.028 dps = 0.49 mrad/s** | 40 Hz 低通后 0.34 mrad/s |
| 陀螺噪声 `gx/gy` | std 0.05~0.08 dps = 0.9~1.4 mrad/s | 比 z 轴略大，不影响 |
| 噪声密度 | gz ≈ **1.3 mdps/√Hz** | 数据手册典型值 2.8 mdps/√Hz |
| 加速度噪声 | 20~25 mm/s²（2~2.5 mg） | |
| 原始零偏 | gz +0.32 dps / gx −0.62 dps / gy +0.11 dps | 块间重复性 0.003 dps，很稳 |
| 零偏后静止 | `gz` 在 ±0.001 rad/s 内 | |
| 静止 130 s 漂移 | yaw < 1°，推算约 0.1~0.3 °/min | 六轴无 yaw 观测，属正常 |
| 地址 | 焊死 SA0 到 GND 后恒为 **0x6A** | 焊死前会偶发漂到 0x6B |
| 20 分钟空闲 soak | 4801 帧全绿：`e=0`、`eps=0`、`sw=0`、`rec=0`、`us` 恒 214 | 焊死 SA0 前同条件是 144 次错误 + 一次 0.7s 爆发 |

对比：四轮测速量化噪声 ±2 RPM ≈ ±0.21 rad/s，是陀螺噪声的 **400 倍** —— 陀螺是底盘库里最干净的速度反馈。

## 9. 七个常见坑

| 现象 | 原因 | 对策 |
|---|---|---|
| 串口显示 `imu --`，全总线扫不到设备 | **H5 座 SDA/SCL 交叉**（本板出厂问题） | 飞线改正；临时用 `IMU_USE_SOFT_I2C=1`（SCL=PB7/SDA=PB6） |
| 硬件 I2C 报 `Err=0x04`（AF） | 地址没人 ACK：没供电 / 线接错 / SDA-SCL 反 | 先量空闲电平（有上拉应读到 1），再用软件 I2C 两种角色各扫一遍定位 |
| `gz` 符号反了 | 模块装反了 | 翻转 `IMU_MAP_Z` 的符号，不用改线 |
| 刚上电前几秒读数偏大 | 芯片启动瞬态 | 已经用 `IMU_STARTUP_SETTLE_MS` 挡住；别在上电瞬间手动标定 |
| 三轴读数完全一样 | 多字节读没开 IF_INC | `imu_configure()` 里已显式 `auto_increment_set`，别删 |
| 静止时 yaw 慢慢漂 | 零偏温漂 | `imu_track_bias()` 静止时自动跟；要更准就停车时手动 `imu_calibrate_bias()` |
| 运行中突然 `us=27 hz=0 ac=0,0,0` | HAL I2C 状态机/总线卡死（从机把 SDA 拉死） | 已内置：DeInit → GPIO 打 9 个时钟 → 重 Init；连续 10 次失败判 `healthy=false` 让外环退回 |
| `us≈27µs` 大量快速 NACK、`sw` 一直涨 | **SA0 没被拉死**，地址在 0x6A/0x6B 之间漂（板载 100k 弱上拉会吊在临界电平） | 焊死 SA0：直连 GND（→0x6A）或 3V3（→0x6B），**不要用 100k 电阻**（会和板载 100k 分压成 1.65V） |

## 10. 为什么没加互补滤波 / 卡尔曼（设计取舍）

经常被问「不加个滤波算法处理零飘和随机误差吗」，这里把账算清楚：

**速率环（小陀螺本体）——不需要，而且闭环对零偏免疫**

- 角速度闭环会把 `gz` 调到参考值，零偏只是让"陀螺零点"偏一点：残余零偏 0.05 mrad/s ≈ **0.003°/s** 的转速误差，可以忽略；
- 噪声：`gz` 0.49 mrad/s（40 Hz 低通后 0.34 mrad/s），而四轮测速的量化噪声是 ±2 RPM ≈ **±0.21 rad/s** —— 陀螺干净 400 倍；
- 再叠一层滤波只会加相位滞后，对 500 Hz 的速率环是负收益。

**角度（yaw 积分）——会漂，但六轴 IMU 靠算法救不回来**

- 重力矢量只约束 roll/pitch，对 yaw **没有任何观测**。所以互补滤波、Mahony、Madgwick、EKF 在只有 6 轴时，yaw 仍然是"gz 纯积分"，漂移一模一样；
- 实测静止 130 s：yaw 显示没动（<1°），按零偏重复性推算约 **0.1~0.3 °/min**；
- 要真正消掉只能引外部参考：磁力计（本模块没有）、**编码器里程计**（底盘已有）、视觉。
  其中里程计最现实：不打滑时用 `chassis_state()->yaw` 慢速校正陀螺积分，自旋打滑时冻结。
  只在做"底盘跟随云台"这种长时角度保持时才需要，小陀螺本身用不到。

**随机误差——已经用最省的手段压到位**

- 片内 LPF1（`GY_LIGHT`）+ 软件 40 Hz 一阶低通：0.49 → 0.34 mrad/s；
- 真正降低零偏不确定度最便宜的办法是「多采点平均」：`IMU_BIAS_CALIB_SAMPLES` 300 → 600，零偏不确定度降 √2 倍。

## 11. 详细文档索引

| 想看什么 | 去哪 |
|---|---|
| 陀螺外环控制律（Kp/Ki/冻结/抗饱和） | `../../docs/CHASSIS_LIBRARY.md` §17.3 |
| 三种模式与小陀螺/云台跟随接口 | 同上 §17.4 / §17.5 |
| 噪声实测原始数据 | `../../test_data/spin_runs/imu_noise*.log` |
| 自旋实验（实验 5）怎么跑 | 同上 §6.8 + `../../tools/README.md`（`spin_analysis.py`） |
| 接线（引脚表） | `../../README.md` |
| 底盘库 | `../Chassis/README.md` |
