/**
  ******************************************************************************
  * @file    chassis_config.h
  * @brief   底盘参数配置表（所有可调参数集中在这里，改完重新编译即可）
  *
  * 坐标系约定（RM 约定）：
  *   +x 车体前进方向，+y 车体左方向，+wz 逆时针为正（俯视）
  *   轮子编号顺序：0=FL(左前) 1=FR(右前) 2=BL(左后) 3=BR(右后)
  *   轮速正方向 = 沿 +x 向前滚动
  *
  * 详细说明见 docs/CHASSIS_LIBRARY.md
  ******************************************************************************
  */
#ifndef USERLIB_CHASSIS_CHASSIS_CONFIG_H
#define USERLIB_CHASSIS_CHASSIS_CONFIG_H

/* ============================= 底盘物理参数 =================================
 * 实测值：轮径 80mm、轴距 200.20mm、轮距 240.00mm（用户提供）
 */
/** 轮子半径 (m) */
#define CHASSIS_WHEEL_RADIUS_M       0.040f
/** 半轴距 = 前后轴距 / 2 (m) */
#define CHASSIS_HALF_WHEELBASE_M     0.1001f
/** 半轮距 = 左右轮距 / 2 (m) */
#define CHASSIS_HALF_TRACK_M         0.120f
/** 编码器每轮圈计数 = 11线 * 4倍频 * 40减速比 = 1760（待实测校准） */
#define CHASSIS_ENCODER_CPR          1760

/* ============================= 电机与驱动 ================================== */
/** 轮速上限 (RPM)。实测满占空比 260~296 RPM（四轮不同），指令给 280 会顶限幅，
 *  取 250 保证留出闭环修正余量（对应约 1.05 m/s） */
#define CHASSIS_MAX_WHEEL_RPM        250.0f
/** 轮速斜坡限幅 (RPM/s)，限制加减速斜率，防止打滑/冲击 */
#define CHASSIS_WHEEL_ACCEL_RPM_S    1500.0f

/* ============================ 车体速度限幅 ================================= */
#define CHASSIS_MAX_VX_M_S           1.0f
#define CHASSIS_MAX_VY_M_S           1.0f
#define CHASSIS_MAX_WZ_RAD_S         4.0f

/* ============================= 运动学 ====================================== */
/** 辊筒朝向：+1 标准安装 / -1 全车辊筒镜像安装。
 *  标定方法：前进与平移正常、旋转方向反了 -> 翻转此值；
 *  或平移方向反了 -> 翻转此值并重新标定（见文档 4.2 节） */
#define CHASSIS_ROLLER_SIGN          1

/* ========================= 测速与控制周期 ================================== */
/** 控制周期 (s)，主循环以 1kHz 调用 chassis_poll() */
#define CHASSIS_CONTROL_PERIOD_S     0.001f
/** 测速滑动窗口 (ms)：对窗口内编码器增量求和，降低低转速量化误差。
 *  每 1ms 一个样本，窗口越大测速越平滑、延迟越大。 */
#define CHASSIS_SPEED_WINDOW_MS      10

/* ======================= 速度环 PID（每轮独立） ==============================
 * 四个轮子各一套参数，可单独微调。
 * 轮序：FL 左前 / FR 右前 / BL 左后 / BR 右后
 * 控制律：duty = Kff*ref + Kp*e + Ki*∫e − Kd*d(meas)/dt
 * 参考：
 *   - 到不了目标转速（差一大截且占空比未饱和）-> 加大该轮 KFF（步进 0.5）
 *   - 轮速振荡      -> 减小该轮 KP；KD 默认 0（见下方说明）
 *   - 稳态仍有小误差 -> 加大该轮 KI 或 I_MAX
 *
 * 关于 KD：测速是 1ms 采样 + 10ms 滑动窗口，量化噪声约 ±7RPM，
 * 直接微分会把噪声放大成占空比尖刺（Kd=0.08 时可达数百‰），
 * 速度环基本不需要 D，默认给 0；确实要加也必须配合 CHASSIS_PID_D_LPF_TAU_S。
 */
/* ---- FL 左前 ---- */
#define CHASSIS_PID_KP_FL        3.0f
#define CHASSIS_PID_KI_FL        40.0f
#define CHASSIS_PID_KD_FL        0.0f
#define CHASSIS_PID_I_MAX_FL     20.0f
#define CHASSIS_PID_KFF_FL       3.85f
#define CHASSIS_PID_KACC_FL      0.25f
/* ---- FR 右前 ---- */
#define CHASSIS_PID_KP_FR        3.0f
#define CHASSIS_PID_KI_FR        40.0f
#define CHASSIS_PID_KD_FR        0.0f
#define CHASSIS_PID_I_MAX_FR     20.0f
#define CHASSIS_PID_KFF_FR       3.85f
#define CHASSIS_PID_KACC_FR      0.25f
/* ---- BL 左后 ---- */
#define CHASSIS_PID_KP_BL        3.0f
#define CHASSIS_PID_KI_BL        40.0f
#define CHASSIS_PID_KD_BL        0.0f
#define CHASSIS_PID_I_MAX_BL     20.0f
#define CHASSIS_PID_KFF_BL       3.95f
#define CHASSIS_PID_KACC_BL      0.25f
/* ---- BR 右后 ---- */
#define CHASSIS_PID_KP_BR        3.0f
#define CHASSIS_PID_KI_BR        40.0f
#define CHASSIS_PID_KD_BR        0.0f
#define CHASSIS_PID_I_MAX_BR     20.0f
#define CHASSIS_PID_KFF_BR       3.50f
#define CHASSIS_PID_KACC_BR      0.25f
/** 输出限幅（占空比千分比，四轮共用：这是 PWM 满幅，无理由分轮设） */
#define CHASSIS_PID_OUT_MAX      1000.0f
/** 测量微分的一阶低通时间常数 (s)，仅在 KD>0 时起作用 */
#define CHASSIS_PID_D_LPF_TAU_S  0.005f

/* =================== 每轮编码器方向归一 (+1 / -1) ===========================
 * 作用：让"车体前进时四个编码器计数同向增加"，只修正测量值符号。
 * 电机转向不在这里改：某轮转向反了，去 chassis.c 的 s_bind[] 对调该轮
 * fwd_in1/fwd_in2。
 * 标定：前进时若某轮实测转速为负（表现为该轮满占空比飞转），把对应值取反。
 * 注意：2025-10-06 重新布线后编码器接线全部变化，以下已重置为 +1 默认值，
 * 首次上电必须按 docs/CHASSIS_LIBRARY.md §10 逐轮标定后再落地跑。
 */
#define CHASSIS_ENC_DIR_FL           +1
#define CHASSIS_ENC_DIR_FR           -1
#define CHASSIS_ENC_DIR_BL           +1
#define CHASSIS_ENC_DIR_BR           -1

/* ======================= 陀螺偏航闭环（小陀螺 / 云台跟随） ==================
 * 用底盘 IMU（LSM6DSV16X）实测的 wz 给运动学打补丁：
 *   wz_out = clamp(wz_ref + Kp·e + Ki·∫e, ±MAX_WZ)，e = wz_ref − wz_gyro
 * 详见 chassis_gyro.c 顶部注释与 docs/CHASSIS_LIBRARY.md §17。
 *
 * 调参直觉：
 *   - 自旋速率稳态差得多、要好几百毫秒才拉回来 -> 加 KP（0.2 一档）
 *   - 稳态始终差固定一小截（打滑补不回来）      -> 加 KI、或加大 I_MAX
 *   - 自旋起步冲过头 / 落地后抖                  -> 减 KP
 */
/** 1 = 启用陀螺外环（IMU 没插也能编译，运行时自动退回纯运动学） */
#define CHASSIS_GYRO_ENABLE          1
/** 上电默认模式：0=OFF 1=RATE 2=SPIN 3=YAW_TARGET（见 chassis_gyro.h） */
#define CHASSIS_GYRO_MODE_DEFAULT    1
/** 比例增益 (rad/s 修正 / rad/s 误差) */
#define CHASSIS_GYRO_KP              0.60f
/** 积分增益 */
#define CHASSIS_GYRO_KI              4.00f
/** 积分项限幅 (rad/s·s)，经 KI 后就是积分项能给出的最大修正 */
#define CHASSIS_GYRO_I_MAX           0.40f
/** 外环「能力带」(rad/s)：误差超过它就是物理上还没追上（起步斜坡/急变），
 *  此时冻结 P 与 I，交给轮速斜坡处理，避免起步瞬间把 wz 顶到限幅。
 *  必须大于最大预期打滑误差：30% 打滑 @3.5rad/s ≈ 1.05rad/s，所以给 2.0 */
#define CHASSIS_GYRO_I_BAND          2.00f
/** PI 修正量总限幅 (rad/s)，防止外环把轮子推爆 */
#define CHASSIS_GYRO_CORR_MAX        1.50f
/** 外环输出 wz 限幅 (rad/s)。比用户指令上限 CHASSIS_MAX_WZ_RAD_S 松一点，
 *  因为自旋要补偿打滑：物理极限 wz = 250RPM × 0.10472 × 0.04 / 0.2201 ≈ 4.76rad/s */
#define CHASSIS_GYRO_WZ_LIMIT        4.60f
/** 角度模式（云台跟随）比例增益 (rad/s 每 rad 误差)，误差 1rad 给 6rad/s */
#define CHASSIS_GYRO_KP_ANGLE        6.00f
/** 静止判定：三轴角速度都小于该值 (rad/s) 才认为车停着 */
#define CHASSIS_GYRO_STATIC_GYRO     0.10f
/** 小陀螺默认自旋速率 (rad/s，逆时针为正)。
 *  2.0rad/s ≈ 115°/s，四轮约 105RPM。
 *  为什么不取运动学上限（250RPM ≈ 4.76rad/s）：麦轮原地自旋要四轮横向擦地，
 *  实测地砖上 3.5rad/s 时占空比顶满 1000‰ 也只能到 2.0~2.6rad/s（见 §17.6）。
 *  等有开阔场地 + 不绕线的测试条件，再用扫速实验定实际上限 */
#define CHASSIS_SPIN_RATE_RAD_S      2.00f

/* ============================ 调试输出 ===================================== */
/** 1=通过 USART1(PA9/PA10, 115200-8-N-1) 每 250ms 输出调试数据；
 *  比赛/正式使用时改 0 关闭 */
#define CHASSIS_DEBUG_ENABLE         1

/** 调试协议选择（VOFA+ 数据引擎要选对应的）：
 *  1 = RawData 文本，终端可直接读，人眼友好
 *  2 = JustFloat 二进制，小端 float + 帧尾 00 00 80 7F，给 VOFA+ 画波形
 *  通道定义（JustFloat, 共 17 通道，见 docs/CHASSIS_LIBRARY.md §6.6） */
#define CHASSIS_DEBUG_PROTO_RAWTEXT      1
#define CHASSIS_DEBUG_PROTO_JUSTFLOAT    2
#define CHASSIS_DEBUG_PROTOCOL           CHASSIS_DEBUG_PROTO_RAWTEXT

/* =========================== PID 整定实验开关 ===============================
 * 整定期间打开：chassis_tune 会接管四个轮子（chassis_poll 的控制节拍照常跑），
 * 串口上输出实验数据。整定完成、正式跑车时改 0（或改 CHASSIS_TUNE_EXPERIMENT
 * 为 0）即可，不影响正常功能。
 *
 *   1 = 开环静态标定：四轮同占空比扫一遍档位，测稳态转速
 *       -> 得到方向、启动死区、占空比-转速增益（用来定 KFF）
 *   2 = 闭环跟踪扫参数：按实际跑车的斜坡(1500RPM/s)做 0→+A→0→−A 跟踪，
 *       1kHz 采样存 RAM 后一次性回传；chassis_tune.c 的 s_gain_sets[] 可一次扫多组
 *   3 = 开环转向目检：+300‰/−300‰ 循环，用来人眼确认四轮转向
 *   4 = 惯性滑行自证：驱动后断开 STBY 与 PWM，看编码器随惯性平滑衰减
 *   5 = 陀螺自旋实测：静置→+自旋→停→−自旋 四段，1kHz 采样陀螺/里程计/轮速
 *       -> 看自旋速率稳态误差、纹波、积分漂移（调 CHASSIS_GYRO_* 用）
 */
#define CHASSIS_TUNE_ENABLE          0
#define CHASSIS_TUNE_EXPERIMENT      2
/** 阶跃实验的目标转速 (RPM)，实验 2 使用 */
#define CHASSIS_TUNE_STEP_RPM        120.0f

/** 1=把两块 TB6612 的 STBY 拉低（输出高阻，电机彻底自由），用于手转测编码器。
 *  正常使用必须为 0。 */
#define CHASSIS_DRIVER_DISABLE       0

/* ============================ 演示 / 自检 ================================== */
/** 1=上电 1.5s 后自动跑演示（前进/左移/左转/后退/右移/右转），仅用于架空自检 */
#define CHASSIS_DEMO_ENABLE          0

#endif /* USERLIB_CHASSIS_CHASSIS_CONFIG_H */
