/**
  ******************************************************************************
  * @file    imu.h
  * @brief   LSM6DSV16X 六轴 IMU（ATK-MS6DSV 模块）驱动与角速度处理
  *
  * 用法：
  *   imu_init();                        // 探测 -> 配置 -> 静止零偏标定
  *   while (1) {
  *       imu_poll(HAL_GetTick());       // 内部按 MS 节拍采样，未到点立即返回
  *       ... imu_state() 读数据 ...
  *   }
  *
  * 坐标系：车体系 +x 前 / +y 左 / +z 上，与底盘库一致。
  * 所有输出都已经是「去零偏 + 轴映射到车体」之后的值。
  ******************************************************************************
  */
#ifndef USERLIB_IMU_IMU_H
#define USERLIB_IMU_IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    /* ---- 健康状态 ---- */
    bool     online;            /* 1 = 探测到并配置成功 */
    bool     healthy;           /* 1 = 最近在读得通；0 = 掉线（数据不可信，外环会自动退回） */
    uint8_t  whoami;            /* WHO_AM_I 读回值，正常 0x70 */
    uint16_t i2c_addr;          /* 实际使用的 HAL 8 位地址（0xD4 / 0xD6） */
    bool     bias_calibrated;   /* 1 = 零偏标定有效 */
    uint32_t sample_count;      /* 累计采样点数 */
    uint32_t error_count;       /* 累计 I2C 错误次数 */
    float    sample_hz;         /* 实测采样频率 */
    float    dt_s;              /* 最近一次采样间隔 (s) */
    float    read_us;           /* 最近一次陀螺读取耗时 (us)，看总线负载用 */
    uint32_t err_per_s;         /* 最近 1 秒新增的错误数（一眼看总线状态） */
    uint32_t addr_switches;     /* 运行中自动切换 0x6A/0x6B 的次数（SA0 不稳的指纹） */
    uint32_t recover_count;     /* 触发总线解锁的次数 */

    /* ---- 角速度（车体系, rad/s） ---- */
    float gyro_rps[3];          /* 已去零偏，未滤波 */
    float gyro_lpf[3];          /* 已去零偏 + 软件低通，控制用这个 */
    float gyro_bias[3];         /* 当前零偏估计 (rad/s) */

    /* ---- 加速度（车体系, m/s^2） ---- */
    float accel_mps2[3];

    /* ---- 积分量 ---- */
    float yaw_rad;              /* 陀螺积分偏航角，-PI..PI，逆时针为正 */
} imu_state_t;

/**
 * @brief 初始化：探测 I2C 地址 -> 软复位 -> 配置量程/速率/滤波 -> 零偏标定
 * @note  内部有阻塞延时（复位 + 标定约 0.6s），只能在 main 初始化阶段调用
 * @retval true = IMU 在线且配置成功
 */
bool imu_init(void);

/**
 * @brief 采样：按 IMU_GYRO_PERIOD_MS 节拍读取并更新 imu_state()
 * @note  必须在主循环里高频调用（建议跟着 1kHz 控制环）
 */
void imu_poll(uint32_t now_ms);

/** 只读状态 */
const imu_state_t *imu_state(void);

/** 是否在线 */
bool imu_is_online(void);

/**
 * @brief 静止零偏重标定（车静止时调用，约 samples*周期 毫秒）
 * @retval true = 标定结果可信（标准差未超限）
 */
bool imu_calibrate_bias(uint16_t samples);

/**
 * @brief 静止零偏慢速跟踪：车静止时把残余零偏往 0 拉
 * @param stationary 外部判断的「车确实静止」标志
 * @param dt         本次调用间隔 (s)
 */
void imu_track_bias(bool stationary, float dt);

/** 把陀螺积分偏航角设为指定值（云台跟随/角度环对齐用） */
void imu_reset_yaw(float yaw_rad);

/** 手动触发一次 I2C 总线恢复（DeInit + Init） */
void imu_bus_recover(void);

/** 把车体系角速度向量映射回芯片轴序（调试打印原始方向用） */
void imu_body_to_chip(const float body[3], float chip[3]);

#endif /* USERLIB_IMU_IMU_H */
