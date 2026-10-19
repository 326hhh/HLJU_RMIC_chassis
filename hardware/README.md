# hardware —— 底盘扩展板硬件资料

本目录存放**底盘扩展板**的设计资料（原理图 + 立创 EDA 工程源文件），
与软件接线相关的引脚表以 [`../README.md`](../README.md) 的「接线」一节为准。

## 目录结构

```
hardware/
├── 底盘扩展板原理图_2026-10-09.pdf                              # 扩展板原理图（PDF 导出，看设计用）
├── 校内赛底盘扩展板_2026-10-06_14-20-59_2026-10-09.epro2         # 立创 EDA 专业版工程（可编辑源文件）
└── README.md
```

## 文件说明

| 文件 | 内容 | 用途 |
|---|---|---|
| `底盘扩展板原理图_2026-10-09.pdf` | 底盘扩展板的完整原理图（电源、TB6612 驱动、编码器接口、CAN 收发、IMU I2C 座、各排针） | 查线、飞线整改、排障时对照 |
| `校内赛底盘扩展板_2026-10-06_14-20-59_2026-10-09.epro2` | 同一块板的 **立创 EDA 专业版**工程源文件（.epro2 是压缩包，用立创 EDA 专业版打开） | 改版、重新导出原理图/PCB |

> 本目录没有 `.c` / `.h` 文件；与硬件相关的固件配置在 `Core/Src/`（引脚/外设初始化）和
> `UserLib/IMU/imu_config.h`（IMU 接线相关的软件兜底）里。

## 已知硬件问题与处理

- **H5（IMU）座 SDA / SCL 与模块交叉**：2026-10-18 实测确认，扩展板 H5 座子上的
  SDA/SCL 与正点原子 ATK-MS6DSV 模块是**交叉**接的，用硬件 I2C1（SCL=PB6 / SDA=PB7）扫不到设备。
  已**飞线改正**。改线前/改线后的软件兜底：`imu_config.h` 里 `IMU_USE_SOFT_I2C`
  置 1 可改用软件 I2C（SCL=PB7 / SDA=PB6）跑通。
- 改线后走**硬件 I2C1 400 kHz**：实测一次读 213 µs、采样 500 Hz；
  软件 I2C 兜底实测 351 µs / 500 Hz。
- 模块 INT 与 SA0 未接 MCU：INT 不用（纯轮询），SA0 由扩展板决定地址（代码对 0x6A / 0x6B 自动探测）。
- 扩展板自带 TJA1050 CAN 收发器（VCC=5 V），发帧时 CANH/CANL 实测 3.10 / 1.704 V，
  断电测终端电阻 118.9 Ω，详见 [`../docs/CAN_TEST.md`](../docs/CAN_TEST.md)。

## 相关文档

| 想看什么 | 去哪 |
|---|---|
| 固件引脚分配（PWM/方向/编码器/USART/IIC/CAN） | [`../README.md`](../README.md) |
| 底盘库接线与标定 | [`../UserLib/Chassis/README.md`](../UserLib/Chassis/README.md) |
| IMU 接线与软件 I2C 兜底 | [`../UserLib/IMU/README.md`](../UserLib/IMU/README.md) |
| CAN 物理层验证 | [`../docs/CAN_TEST.md`](../docs/CAN_TEST.md) |
