/**
  ******************************************************************************
  * @file    imu.c
  * @brief   LSM6DSV16X（ATK-MS6DSV 模块）驱动实现
  *
  * 分层：
  *   上层（imu_poll）只干三件事：读原始值 -> 去零偏/映射到车体系 -> 低通 + 积分
  *   下层（plat_*）只负责把 ST 官方驱动的 read_reg/write_reg 翻译成 HAL I2C 事务
  *
  * 注意：ST 官方驱动在 I2C 模式下**不要**给寄存器地址置 0x80 多位读标志，
  *       芯片靠 IF_INC（复位默认开）自动递增地址。
  ******************************************************************************
  */
#include "imu_config.h"
#include "imu.h"

#include "main.h"
#if !IMU_USE_SOFT_I2C
#include "i2c.h"        /* 只有硬件 I2C 后端才需要 hi2c1 / MX_I2C1_Init */
#endif

#include <math.h>
#include <string.h>

#define IMU_REG_WHO_AM_I    0x0FU
#define IMU_WHO_AM_I_VAL    0x70U

static stmdev_ctx_t s_ctx;
static uint16_t s_addr_hal;         /* HAL 8 位地址（7 位 << 1） */
static imu_state_t s_st;
static uint32_t s_last_tick;
static uint32_t s_last_tick_valid;
static uint32_t s_period_ms;
static uint32_t s_accel_div;
static uint32_t s_err_run;
static float s_lpf_alpha;
static float s_dt_f;                /* dt 的平滑值，给低通/统计用 */
static bool  s_dwt_ok;              /* DWT 周期计数器是否可用（量 I2C 读耗时） */
static uint32_t s_last_recover_ms;  /* 上次总线恢复的时刻（限流用） */
static uint32_t s_err_window;       /* 本秒内累计的错误数 */
static uint32_t s_rate_tick;        /* 上一秒的边界 */

/* 轴映射表：车身 (x,y,z) 分别取自芯片的哪一根轴（编码见 imu_config.h） */
static const int8_t s_map[3] = { IMU_MAP_X, IMU_MAP_Y, IMU_MAP_Z };

/* ========================== 平台层（I2C） =================================
 * 两种后端，由 imu_config.h 的 IMU_USE_SOFT_I2C 选择：
 *   0 = 硬件 I2C1（PB6=SCL / PB7=SDA，需要板子接线正确，400kHz 最快）
 *   1 = 软件 I2C（引脚角色可配，用来救「SDA/SCL 接反」的板子）
 * 两条后端都只暴露 plat_write / plat_read 两个函数，上层完全无感。
 */
#if IMU_USE_SOFT_I2C

static uint16_t s_addr7 = IMU_ADDR_SA0_LOW;

/* 直接用 BSRR 翻引脚（HAL_GPIO_WritePin 太慢，-O0 下一次要 ~1us） */
#define SOFT_SDA_HIGH()   (IMU_SOFT_PORT->BSRR = (uint32_t)IMU_SOFT_PIN_SDA)
#define SOFT_SDA_LOW()    (IMU_SOFT_PORT->BSRR = (uint32_t)IMU_SOFT_PIN_SDA << 16)
#define SOFT_SCL_HIGH()   (IMU_SOFT_PORT->BSRR = (uint32_t)IMU_SOFT_PIN_SCL)
#define SOFT_SCL_LOW()    (IMU_SOFT_PORT->BSRR = (uint32_t)IMU_SOFT_PIN_SCL << 16)
#define SOFT_SDA_READ()   ((IMU_SOFT_PORT->IDR & (uint32_t)IMU_SOFT_PIN_SDA) != 0U)

static uint32_t s_half_cycles;      /* 半个 SCL 周期的 DWT 周期数 */
static bool s_soft_dwt;             /* 量时间用的 DWT 是否就绪 */

static void soft_delay_half(void)
{
    if (s_soft_dwt)
    {
        uint32_t t0 = DWT->CYCCNT;
        while ((DWT->CYCCNT - t0) < s_half_cycles)
        {
            __NOP();
        }
    }
    else
    {
        for (volatile uint32_t i = 0; i < IMU_SOFT_DELAY_LOOPS; i++)
        {
            __NOP();
        }
    }
}

static void soft_start(void)
{
    SOFT_SDA_HIGH();
    soft_delay_half();
    SOFT_SCL_HIGH();
    soft_delay_half();
    SOFT_SDA_LOW();
    soft_delay_half();
    SOFT_SCL_LOW();
    soft_delay_half();
}

static void soft_stop(void)
{
    SOFT_SDA_LOW();
    soft_delay_half();
    SOFT_SCL_HIGH();
    soft_delay_half();
    SOFT_SDA_HIGH();
    soft_delay_half();
}

static int soft_write_byte(uint8_t b)
{
    int ack;

    for (int i = 0; i < 8; i++)
    {
        if (b & 0x80U) { SOFT_SDA_HIGH(); } else { SOFT_SDA_LOW(); }
        b = (uint8_t)(b << 1);
        soft_delay_half();
        SOFT_SCL_HIGH();
        soft_delay_half();
        SOFT_SCL_LOW();
        soft_delay_half();
    }
    SOFT_SDA_HIGH();                    /* 释放 SDA 等从机 ACK */
    soft_delay_half();
    SOFT_SCL_HIGH();
    soft_delay_half();
    ack = SOFT_SDA_READ() ? 0 : 1;
    SOFT_SCL_LOW();
    soft_delay_half();
    return ack;
}

static uint8_t soft_read_byte(int ack)
{
    uint8_t b = 0;

    SOFT_SDA_HIGH();
    for (int i = 0; i < 8; i++)
    {
        soft_delay_half();
        SOFT_SCL_HIGH();
        soft_delay_half();
        b = (uint8_t)((b << 1) | (SOFT_SDA_READ() ? 1U : 0U));
        SOFT_SCL_LOW();
    }
    if (ack) { SOFT_SDA_LOW(); } else { SOFT_SDA_HIGH(); }
    soft_delay_half();
    SOFT_SCL_HIGH();
    soft_delay_half();
    SOFT_SCL_LOW();
    SOFT_SDA_HIGH();
    soft_delay_half();
    return b;
}

/** 把引脚切成开漏 GPIO（I2C 空闲态：两条线都被上拉，输出 1 = 释放） */
static void soft_pins_init(void)
{
    GPIO_InitTypeDef g = { 0 };

    g.Pin = (uint16_t)(IMU_SOFT_PIN_SCL | IMU_SOFT_PIN_SDA);
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(IMU_SOFT_PORT, &g);
    HAL_GPIO_WritePin(IMU_SOFT_PORT, (uint16_t)(IMU_SOFT_PIN_SCL | IMU_SOFT_PIN_SDA), GPIO_PIN_SET);
}

/** 总线卡死恢复：手动打 9 个 SCL 脉冲把从机状态机放出来 */
static void soft_bus_recover(void)
{
    soft_pins_init();
    SOFT_SDA_HIGH();
    for (int i = 0; i < 9; i++)
    {
        SOFT_SCL_HIGH();
        soft_delay_half();
        SOFT_SCL_LOW();
        soft_delay_half();
    }
    soft_stop();
}

static int32_t plat_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    (void)handle;
    soft_start();
    if (!soft_write_byte((uint8_t)(s_addr7 << 1)))
    {
        soft_stop();
        return -1;
    }
    if (!soft_write_byte(reg))
    {
        soft_stop();
        return -1;
    }
    for (uint16_t i = 0; i < len; i++)
    {
        if (!soft_write_byte(bufp[i]))
        {
            soft_stop();
            return -1;
        }
    }
    soft_stop();
    return 0;
}

static int32_t plat_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    (void)handle;
    soft_start();
    if (!soft_write_byte((uint8_t)(s_addr7 << 1)))
    {
        soft_stop();
        return -1;
    }
    if (!soft_write_byte(reg))
    {
        soft_stop();
        return -1;
    }
    soft_start();                               /* 重复起始 + 读地址 */
    if (!soft_write_byte((uint8_t)((s_addr7 << 1) | 1U)))
    {
        soft_stop();
        return -1;
    }
    for (uint16_t i = 0; i < len; i++)
    {
        bufp[i] = soft_read_byte((i + 1U < len) ? 1 : 0);
    }
    soft_stop();
    return 0;
}

#else  /* 硬件 I2C1 */

/**
 * @brief 备用地址：SA0 悬空/接触不良时地址会在 0x6A/0x6B 之间漂移，
 *        默认地址读失败时换另一个地址再试一次，成功后锁定新地址。
 *        （2026-10-19 现场：I2C 单次读 27us 快速失败、错误率过半，
 *          物理接线检查无果，属典型的地址应答不稳）
 */
static uint16_t imu_alt_addr(void)
{
    return (s_addr_hal == (uint16_t)(IMU_ADDR_SA0_LOW << 1))
               ? (uint16_t)(IMU_ADDR_SA0_HIGH << 1)
               : (uint16_t)(IMU_ADDR_SA0_LOW << 1);
}

static int32_t plat_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    (void)handle;
    if (HAL_I2C_Mem_Write(&IMU_I2C_HANDLE, s_addr_hal, reg, I2C_MEMADD_SIZE_8BIT,
                          (uint8_t *)bufp, len, IMU_I2C_TIMEOUT_MS) == HAL_OK)
    {
        return 0;
    }
    {
        const uint16_t alt = imu_alt_addr();
        if (HAL_I2C_Mem_Write(&IMU_I2C_HANDLE, alt, reg, I2C_MEMADD_SIZE_8BIT,
                              (uint8_t *)bufp, len, IMU_I2C_TIMEOUT_MS) == HAL_OK)
        {
            s_addr_hal = alt;
            s_st.i2c_addr = alt;
            s_st.addr_switches++;
            return 0;
        }
    }
    return -1;
}

static int32_t plat_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    (void)handle;
    if (HAL_I2C_Mem_Read(&IMU_I2C_HANDLE, s_addr_hal, reg, I2C_MEMADD_SIZE_8BIT,
                         bufp, len, IMU_I2C_TIMEOUT_MS) == HAL_OK)
    {
        return 0;
    }
    {
        const uint16_t alt = imu_alt_addr();
        if (HAL_I2C_Mem_Read(&IMU_I2C_HANDLE, alt, reg, I2C_MEMADD_SIZE_8BIT,
                             bufp, len, IMU_I2C_TIMEOUT_MS) == HAL_OK)
        {
            s_addr_hal = alt;
            s_st.i2c_addr = alt;
            s_st.addr_switches++;
            return 0;
        }
    }
    return -1;
}

#endif /* IMU_USE_SOFT_I2C */

static void plat_delay(uint32_t ms)
{
    HAL_Delay(ms);
}

/** 打开 DWT 周期计数器，用来看一次 I2C 读占了多少 µs */
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_dwt_ok = true;
}

static uint32_t dwt_cyc(void)
{
    return s_dwt_ok ? DWT->CYCCNT : 0U;
}

/** 微秒级忙等；DWT 不可用时退化成 HAL_Delay(1) */
static void dwt_delay_us(uint32_t us)
{
    if (!s_dwt_ok)
    {
        HAL_Delay(1u);
        return;
    }
    {
        const uint32_t start = DWT->CYCCNT;
        const uint32_t cycles = us * (SystemCoreClock / 1000000U);
        while ((DWT->CYCCNT - start) < cycles)
        {
        }
    }
}

/**
 * @brief I2C 总线恢复
 * @note  硬件 I2C：外设 DeInit + Init 重来一遍；软件 I2C：打 9 个 SCL 脉冲
 *        只在连续多次读失败时调用，避免把偶发抖动放大成抖动源
 */
/**
 * @brief 硬件 I2C 模式下把总线救活：
 *        ① 外设 DeInit，引脚交回 GPIO；
 *        ② SDA 释放、SCL 手动打 9 个时钟 —— 把卡在某个字节中间的从机状态机顶出来
 *           （从机若把 SDA 拉死，光 DeInit/Init 是解不开的，必须给时钟）；
 *        ③ 补一个 STOP，再重新初始化外设。
 */
#if !IMU_USE_SOFT_I2C
static void hw_bus_unstick(void)
{
    GPIO_InitTypeDef g = { 0 };

    (void)HAL_I2C_DeInit(&IMU_I2C_HANDLE);

    g.Pin = (uint16_t)(IMU_HW_PIN_SCL | IMU_HW_PIN_SDA);
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IMU_HW_PORT, &g);
    HAL_GPIO_WritePin(IMU_HW_PORT, (uint16_t)(IMU_HW_PIN_SCL | IMU_HW_PIN_SDA), GPIO_PIN_SET);
    dwt_delay_us(10u);

    for (int i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(IMU_HW_PORT, IMU_HW_PIN_SCL, GPIO_PIN_RESET);
        dwt_delay_us(10u);
        HAL_GPIO_WritePin(IMU_HW_PORT, IMU_HW_PIN_SCL, GPIO_PIN_SET);
        dwt_delay_us(10u);
    }
    HAL_GPIO_WritePin(IMU_HW_PORT, IMU_HW_PIN_SDA, GPIO_PIN_RESET);
    dwt_delay_us(10u);
    HAL_GPIO_WritePin(IMU_HW_PORT, IMU_HW_PIN_SCL, GPIO_PIN_SET);
    dwt_delay_us(10u);
    HAL_GPIO_WritePin(IMU_HW_PORT, IMU_HW_PIN_SDA, GPIO_PIN_SET);
    dwt_delay_us(10u);

    MX_I2C1_Init();                 /* MspInit 会把引脚切回 AF */
}
#endif

void imu_bus_recover(void)
{
    uint32_t now = HAL_GetTick();

    /* 恢复动作不便宜（DeInit/Init 期间控制环会掉几拍），限流一下 */
    if ((now - s_last_recover_ms) < IMU_RECOVER_MIN_MS)
    {
        s_err_run = 0;
        return;
    }
    s_last_recover_ms = now;

#if IMU_USE_SOFT_I2C
    soft_bus_recover();
#else
    hw_bus_unstick();
#endif
    s_st.recover_count++;
    s_err_run = 0;
}

/** 记录一次 I2C 事务结果，连续失败到阈值就尝试恢复总线 */
static void imu_note_result(bool ok)
{
    if (ok)
    {
        s_err_run = 0;
        s_st.healthy = true;
        return;
    }

    s_st.error_count++;
    s_err_window++;
    s_err_run++;
    if (s_err_run >= IMU_ERR_STALE_COUNT)
    {
        /* 连读都读不通了：数据不可信，外环应当退回纯运动学 */
        s_st.healthy = false;
    }
    if (s_err_run >= IMU_ERR_RECOVER_COUNT)
    {
        imu_bus_recover();
    }
}

/** 探测一个 7 位地址：读 WHO_AM_I 匹配就认这个 */
static bool imu_probe_addr(uint8_t addr7)
{
    uint8_t id = 0;
    int32_t ret;

    s_addr_hal = (uint16_t)(addr7 << 1);
#if IMU_USE_SOFT_I2C
    s_addr7 = addr7;
#endif
    ret = lsm6dsv16x_read_reg(&s_ctx, IMU_REG_WHO_AM_I, &id, 1);
    if ((ret != 0) || (id != IMU_WHO_AM_I_VAL))
    {
        return false;
    }
    s_st.whoami = id;
    return true;
}

static bool imu_probe(void)
{
    if (imu_probe_addr(IMU_ADDR_SA0_LOW))
    {
        s_st.i2c_addr = s_addr_hal;
        return true;
    }
#if IMU_PROBE_BOTH_ADDR
    if (imu_probe_addr(IMU_ADDR_SA0_HIGH))
    {
        s_st.i2c_addr = s_addr_hal;
        return true;
    }
#endif
    return false;
}

/* ============================ 轴映射 ====================================== */

/**
 * @brief 芯片轴序 -> 车体轴序（有符号置换矩阵）
 * @note  编码值 = 符号 × (轴号 + 1)
 */
static void imu_map_chip_to_body(const float chip[3], float body[3])
{
    for (int i = 0; i < 3; i++)
    {
        int8_t m = s_map[i];
        int idx = (m > 0) ? (m - 1) : (-m - 1);
        body[i] = (m > 0) ? chip[idx] : -chip[idx];
    }
}

/** 车体轴序 -> 芯片轴序（调试打印/反算用） */
void imu_body_to_chip(const float body[3], float chip[3])
{
    for (int i = 0; i < 3; i++)
    {
        int8_t m = s_map[i];
        int idx = (m > 0) ? (m - 1) : (-m - 1);
        chip[idx] = (m > 0) ? body[i] : -body[i];
    }
}

/* ============================== 初始化 ==================================== */

static void imu_configure(void)
{
    lsm6dsv16x_sw_reset(&s_ctx);                                    /* 全部寄存器回默认 */
    lsm6dsv16x_block_data_update_set(&s_ctx, PROPERTY_ENABLE);      /* 高低字节一致 */
    /* IF_INC：多字节读时寄存器地址自动递增。不复位成 1 的话，一次读 6 字节
     * 会拿到同一个寄存器 6 次 —— 症状是 X/Y/Z 三轴读数完全一样。
     * ST 官方 I2C 例程靠的就是这个位（I2C 模式不要给地址置 0x80）。 */
    lsm6dsv16x_auto_increment_set(&s_ctx, PROPERTY_ENABLE);
    lsm6dsv16x_gy_full_scale_set(&s_ctx, IMU_FS_GYRO);
    lsm6dsv16x_xl_full_scale_set(&s_ctx, IMU_FS_ACCEL);
    lsm6dsv16x_filt_gy_lp1_set(&s_ctx, PROPERTY_ENABLE);
    lsm6dsv16x_filt_gy_lp1_bandwidth_set(&s_ctx, IMU_GY_LPF_BW);
    lsm6dsv16x_gy_data_rate_set(&s_ctx, IMU_ODR_GYRO);
    lsm6dsv16x_xl_data_rate_set(&s_ctx, IMU_ODR_ACCEL);

    /* 软件低通系数：alpha = dt / (tau + dt) */
    if (IMU_GYRO_LPF_HZ > 0.0f)
    {
        float tau = 1.0f / (2.0f * 3.14159265f * IMU_GYRO_LPF_HZ);
        s_lpf_alpha = ((float)s_period_ms * 0.001f) / (tau + ((float)s_period_ms * 0.001f));
    }
    else
    {
        s_lpf_alpha = 1.0f;
    }
}

bool imu_init(void)
{
    memset(&s_st, 0, sizeof(s_st));
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.write_reg = plat_write;
    s_ctx.read_reg = plat_read;
    s_ctx.mdelay = plat_delay;
    s_ctx.handle = NULL;

    /* 采样周期：硬件 400kHz 下 2ms 一次；100kHz 硬件 I2C 放宽到 3ms。
     * 软件 I2C 实测一次读 6 字节约 350us（~230kHz SCL），2ms 一拍占 17% CPU，可以接受 */
    s_period_ms = IMU_GYRO_PERIOD_MS;
#if !IMU_USE_SOFT_I2C
    if (IMU_I2C_HANDLE.Init.ClockSpeed < 400000U)
    {
        s_period_ms = IMU_GYRO_PERIOD_MS + 1U;
    }
#endif
    s_accel_div = IMU_ACCEL_EVERY_N;
    s_dt_f = (float)s_period_ms * 0.001f;
    s_err_run = 0;
    s_last_tick = HAL_GetTick();
    s_last_tick_valid = 0;
    dwt_init();

#if IMU_USE_SOFT_I2C
    /* CubeMX 已经把 PB6/PB7 配成 I2C1 复用，这里改回开漏 GPIO 自己发时钟 */
    s_soft_dwt = s_dwt_ok;
    s_half_cycles = (SystemCoreClock / 1000000U) * IMU_SOFT_HALF_US;
    soft_pins_init();
    soft_bus_recover();
#endif

    if (!imu_probe())
    {
        s_st.online = false;
        return false;
    }

    imu_configure();
    HAL_Delay(IMU_STARTUP_SETTLE_MS);       /* 等启动瞬态过去再标零偏（见 imu_config.h） */

    s_st.online = true;
    s_st.healthy = true;
    s_st.addr_switches = 0U;        /* 探测阶段的切换不算「运行中漂移」 */
    imu_reset_yaw(0.0f);
    (void)imu_calibrate_bias(IMU_BIAS_CALIB_SAMPLES);
    return true;
}

/* ============================ 零偏标定 ==================================== */

bool imu_calibrate_bias(uint16_t samples)
{
    int16_t raw[3];
    float chip[3], body[3];
    float sum[3] = { 0.0f, 0.0f, 0.0f };
    float sumsq[3] = { 0.0f, 0.0f, 0.0f };
    uint16_t ok = 0;
    uint16_t n = (samples == 0U) ? 1U : samples;

    if (!s_st.online)
    {
        return false;
    }

    for (uint16_t i = 0; i < n; i++)
    {
        if (lsm6dsv16x_angular_rate_raw_get(&s_ctx, raw) == 0)
        {
            for (int k = 0; k < 3; k++)
            {
                chip[k] = lsm6dsv16x_from_fs1000_to_mdps(raw[k]) * (3.14159265f / 180000.0f);
            }
            imu_map_chip_to_body(chip, body);
            for (int k = 0; k < 3; k++)
            {
                sum[k] += body[k];
                sumsq[k] += body[k] * body[k];
            }
            ok++;
        }
        HAL_Delay(s_period_ms);
    }

    if (ok < (n / 2U))
    {
        s_st.bias_calibrated = false;
        return false;
    }

    s_st.bias_calibrated = true;
    for (int k = 0; k < 3; k++)
    {
        float mean = sum[k] / (float)ok;
        float var = (sumsq[k] / (float)ok) - (mean * mean);

        if (var < 0.0f)
        {
            var = 0.0f;
        }
        if (sqrtf(var) > IMU_BIAS_CALIB_STD_MAX)
        {
            /* 标定期间车在动：这一轴的均值不可信，宁可不用它（后面靠静止跟踪补） */
            s_st.bias_calibrated = false;
        }
        else
        {
            s_st.gyro_bias[k] = mean;
        }
    }

    /* 标定完把滤波状态也一起对齐，避免第一拍滤波跳变 */
    for (int k = 0; k < 3; k++)
    {
        s_st.gyro_rps[k] = 0.0f;
        s_st.gyro_lpf[k] = 0.0f;
    }
    return s_st.bias_calibrated;
}

void imu_track_bias(bool stationary, float dt)
{
    float a;

    if ((!stationary) || (!s_st.online) || (IMU_BIAS_TRACK_TAU_S <= 0.0f))
    {
        return;
    }

    a = dt / IMU_BIAS_TRACK_TAU_S;
    if (a > 0.2f)
    {
        a = 0.2f;
    }

    for (int k = 0; k < 3; k++)
    {
        /* 残余量很小才敢动零偏，否则可能把真实转动当成零偏吃掉 */
        if (fabsf(s_st.gyro_rps[k]) < IMU_BIAS_TRACK_MAX)
        {
            s_st.gyro_bias[k] += a * s_st.gyro_rps[k];
        }
    }
}

void imu_reset_yaw(float yaw_rad)
{
    s_st.yaw_rad = yaw_rad;
}

/* ============================== 采样 ====================================== */

void imu_poll(uint32_t now_ms)
{
    int16_t raw[3];
    float chip[3];
    float dt;

    if (!s_st.online)
    {
        return;             /* 初始化就没连上，不折腾 */
    }

    if (s_last_tick_valid != 0U)
    {
        if ((now_ms - s_last_tick) < s_period_ms)
        {
            return;
        }
    }
    s_last_tick_valid = 1U;

    dt = (float)(now_ms - s_last_tick) * 0.001f;
    s_last_tick = now_ms;
    if (dt < 0.0005f)
    {
        dt = 0.0005f;
    }
    if (dt > 0.05f)
    {
        dt = 0.05f;             /* 断点/长阻塞后限幅，防止 yaw 一下跳飞 */
    }
    s_st.dt_s = dt;
    s_dt_f += 0.1f * (dt - s_dt_f);

    /* 每 1 秒结算一次「新增错误数」，比累计值好用 */
    if ((now_ms - s_rate_tick) >= 1000U)
    {
        s_st.err_per_s = s_err_window;
        s_err_window = 0U;
        s_rate_tick = now_ms;
    }

    /* ---- 陀螺（顺便量一下这一读花了多少 us） ---- */
    {
        uint32_t t0 = dwt_cyc();
        int32_t gyret = lsm6dsv16x_angular_rate_raw_get(&s_ctx, raw);
        uint32_t t1 = dwt_cyc();

        s_st.read_us = ((float)(t1 - t0) * 1000000.0f) / (float)SystemCoreClock;

        if (gyret == 0)
        {
            imu_note_result(true);
            for (int k = 0; k < 3; k++)
            {
                chip[k] = lsm6dsv16x_from_fs1000_to_mdps(raw[k]) * (3.14159265f / 180000.0f);
            }
            imu_map_chip_to_body(chip, s_st.gyro_rps);
            for (int k = 0; k < 3; k++)
            {
                s_st.gyro_rps[k] -= s_st.gyro_bias[k];
                s_st.gyro_lpf[k] += s_lpf_alpha * (s_st.gyro_rps[k] - s_st.gyro_lpf[k]);
            }
            s_st.yaw_rad += s_st.gyro_lpf[2] * dt;
            while (s_st.yaw_rad >= 3.14159265f)
            {
                s_st.yaw_rad -= 6.28318531f;
            }
            while (s_st.yaw_rad < -3.14159265f)
            {
                s_st.yaw_rad += 6.28318531f;
            }
            s_st.sample_count++;
            if (s_dt_f > 1e-6f)
            {
                s_st.sample_hz = 1.0f / s_dt_f;
            }
        }
        else
        {
            imu_note_result(false);
        }
    }

    /* ---- 加速度（低频读一次，只做倾角/静止判断） ---- */
    if ((s_st.sample_count % s_accel_div) == 0U)
    {
        if (lsm6dsv16x_acceleration_raw_get(&s_ctx, raw) == 0)
        {
            imu_note_result(true);
            for (int k = 0; k < 3; k++)
            {
                chip[k] = lsm6dsv16x_from_fs4_to_mg(raw[k]) * 0.00980665f;
            }
            imu_map_chip_to_body(chip, s_st.accel_mps2);
        }
        else
        {
            imu_note_result(false);
        }
    }
}

const imu_state_t *imu_state(void)
{
    return &s_st;
}

bool imu_is_online(void)
{
    return s_st.online;
}
