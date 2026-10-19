/**
  ******************************************************************************
  * @file    imu_config.h
  * @brief   LSM6DSV16X（正点原子 ATK-MS6DSV 模块）配置表
  *
  * 硬件：模块插在底盘扩展板 IIC 座（H5），SCL=PB6 / SDA=PB7 = I2C1，
  *       模块自带 4.7K 上拉，供电 3V3，INT 与 SA0 未接 MCU（纯轮询）。
  *
  * 采样策略：控制环 1kHz，但 I2C 读一次陀螺约 200us（400kHz），
  *           所以陀螺按 IMU_GYRO_PERIOD_MS 节拍读、加速度更慢，
  *           中间用「样本保持」喂给控制环。晶振/时序都是相对量，不影响闭环。
  ******************************************************************************
  */
#ifndef USERLIB_IMU_IMU_CONFIG_H
#define USERLIB_IMU_IMU_CONFIG_H

#include "lsm6dsv16x_reg.h"

/* ============================== 总线 ====================================== */
/** 使用的 HAL I2C 句柄（CubeMX 里 I2C1：PB6/PB7） */
#define IMU_I2C_HANDLE          hi2c1
/** 7 位从机地址：SA0 接 GND = 0x6A，接 VCC = 0x6B（本板实测 0x6B） */
#define IMU_ADDR_SA0_LOW        0x6AU
#define IMU_ADDR_SA0_HIGH       0x6BU
/** 1 = 两个地址都试（SA0 接线不确定时用），0 = 只用 IMU_ADDR_SA0_LOW */
#define IMU_PROBE_BOTH_ADDR     1
/** 单次 I2C 事务超时 (ms)。400kHz 下 6 字节 <1ms，给 3ms 足够 */
#define IMU_I2C_TIMEOUT_MS      3U
/** 连续错误达到该次数就尝试解锁总线 + 重新初始化 I2C 外设 */
#define IMU_ERR_RECOVER_COUNT   5U
/** 连续错误达到该次数就判「IMU 掉线」：外环退回纯运动学，不再用陈旧数据。
 *  500Hz 下 10 次 = 20ms，速率环等得起 */
#define IMU_ERR_STALE_COUNT     10U
/** 两次总线恢复之间的最小间隔 (ms)，避免刷屏式 DeInit/Init */
#define IMU_RECOVER_MIN_MS      100U
/** 硬件 I2C 的引脚（解锁总线时要当 GPIO 用；软硬件两种模式都靠它打时钟） */
#define IMU_HW_PORT             GPIOB
#define IMU_HW_PIN_SCL          GPIO_PIN_6
#define IMU_HW_PIN_SDA          GPIO_PIN_7

/* ============================ 软件 I2C 兜底 ================================
 * 2026-10-18 实测发现：扩展板 H5 座子上的 SDA / SCL 与 ATK-MS6DSV 模块的
 * 引脚定义是**交叉**的 —— 用硬件 I2C1（SCL=PB6 / SDA=PB7）扫不到任何设备
 * （AF NACK），改用软件 I2C 把 SCL 放 PB7、SDA 放 PB6，立刻读到
 * 0x6B 应答、WHO_AM_I = 0x70，即模块本身是好的。
 *
 *   0 = 硬件 I2C1（**当前用这个**：2026-10-18 已飞线把交叉改回来，实测 213µs/读）
 *   1 = 软件 I2C 兜底（硬件还是交叉、暂时改不了线时用，实测 351µs/读）
 */
#define IMU_USE_SOFT_I2C        0
/** 软件 I2C 的引脚角色（与硬件相反：SCL 放 PB7、SDA 放 PB6） */
#define IMU_SOFT_PORT           GPIOB
#define IMU_SOFT_PIN_SCL        GPIO_PIN_7
#define IMU_SOFT_PIN_SDA        GPIO_PIN_6
/** 半个 SCL 周期 (us)：1.0 ≈ 500kHz，1.3 ≈ 380kHz，2.0 ≈ 250kHz。
 *  太长浪费时间（每次读 6 字节 = 81 个比特），太短会被上拉拖住产生误码 */
#define IMU_SOFT_HALF_US        1.3f
/** DWT 不可用时的退化延时（空转次数），正常情况下用不到 */
#define IMU_SOFT_DELAY_LOOPS    12U

/* ============================ 传感器配置 ================================== */
/** 陀螺输出速率：必须 >= 2 倍软件采样率（500Hz 采样用 960Hz） */
#define IMU_ODR_GYRO            LSM6DSV16X_ODR_AT_960Hz
/** 加速度输出速率（只用来判断倾角/静止，慢点无所谓） */
#define IMU_ODR_ACCEL           LSM6DSV16X_ODR_AT_120Hz
/** 陀螺量程：车体自旋最高约 4.8rad/s = 275dps，±1000dps 留 3.6 倍余量，
 *  分辨率 0.03dps/LSB（±2000dps 会浪费一半分辨率） */
#define IMU_FS_GYRO             LSM6DSV16X_1000dps
/** 加速度量程：车身冲击一般 <2g，±4g 够用 */
#define IMU_FS_ACCEL            LSM6DSV16X_4g
/** 陀螺片内 LPF1 带宽档位（越往后滤得越狠） */
#define IMU_GY_LPF_BW           LSM6DSV16X_GY_LIGHT

/* ============================ 软件采样 ==================================== */
/** 陀螺采样周期 (ms)：400kHz 下取 2（500Hz），100kHz 下代码会自动 +1ms */
#define IMU_GYRO_PERIOD_MS      2U
/** 每读多少次陀螺读一次加速度 */
#define IMU_ACCEL_EVERY_N       10U
/** 陀螺软件一阶低通截止频率 (Hz)，0 = 不滤（只用片内 LPF1） */
#define IMU_GYRO_LPF_HZ         40.0f

/* ============================= 零偏标定 =================================== */
/** 上电后先静置多久再做零偏标定 (ms)。
 *  实测：上电前 ~5s 陀螺 x/y 有明显启动瞬态（gz 峰峰 1.2dps、gx 峰峰 11dps），
 *  0.7s 就标定会被这段噪声污染，等 1.2s 后基本平稳 */
#define IMU_STARTUP_SETTLE_MS   1200U
/** 上电静止零偏标定的采样数（每个 IMU_GYRO_PERIOD_MS 一个点） */
#define IMU_BIAS_CALIB_SAMPLES  300U
/** 零偏标定标准差超过该值 (rad/s) 就认为上电时车在动，标记「未标定」 */
#define IMU_BIAS_CALIB_STD_MAX  0.05f
/** 静止时零偏慢速跟踪时间常数 (s)；0 = 关闭跟踪 */
#define IMU_BIAS_TRACK_TAU_S    4.0f
/** 只有实测角速度全部小于该值 (rad/s) 才允许跟踪零偏 */
#define IMU_BIAS_TRACK_MAX      0.20f

/* ============================= 轴映射 =====================================
 * 车体系：+x 车头，+y 车体左侧，+z 向上（右手系）
 * 取值 = 符号 × (芯片轴号 + 1)：+1=芯片+X  -1=芯片-X  +2=芯片+Y  +3=芯片+Z
 * 当前装法（用户实测）：模块平放，丝印 X 指车头、Y 指车左 -> 恒等映射
 * 若发现转向时 wz 符号反了，把 IMU_MAP_Z 的符号翻转即可（不用改接线）
 */
#define IMU_MAP_X               (+1)
#define IMU_MAP_Y               (+2)
#define IMU_MAP_Z               (+3)

#endif /* USERLIB_IMU_IMU_CONFIG_H */
