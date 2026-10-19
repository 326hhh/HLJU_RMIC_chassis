# tools —— PC 端工具（数据分析 / 协议参考 / 交叉校验）

本目录放**跑在 PC 上**、配合固件做数据分析与链路验证的脚本/程序，
不属于固件构建产物（`CMakeLists.txt` 不会编译它们）。

## 目录结构

```
tools/
├── step_analysis.py      # 阶跃/斜升响应数据解析 + 指标计算（配 chassis_tune 实验 2）
├── spin_analysis.py      # 陀螺自旋实验数据解析（配 chassis_tune 实验 5）
├── uart_link_test.py     # USART2 链路：协议参考实现 + 台架测试 CLI（CRC8 与 C 侧一致）
├── codec_test.c          # 协议编解码 PC 端交叉校验（C 编码 == Python 编码）
├── __pycache__/          # Python 字节码缓存（可删）
└── README.md
```

## 文件说明

### `step_analysis.py` —— 阶跃响应数据解析

- **作用**：解析 `chassis_tune` 实验 2（闭环斜升扫参）从 USART1 吐出的**串口原始抓取**，
  按段计算指标。
- **输入格式**：`SET idx=.. kp=.. ki=.. kacc=.. rpm=..` + `STEP_BEGIN n=.. ch=12 dt_us=1000`
  + `n × 24 字节`（12 通道小端 int16）。
- **输出**（按轮 FL FR BL BR）：稳态轮速 `ss`、稳态误差 `err`、测速噪声 `noise`、
  进入 ±5% 的上升时间 `rise`、超调 `ovs`、调节时间 `settle`、±5% 外占比 `outside`、占空比。
- **用法**：

  ```bash
  python3 tools/step_analysis.py test_data/tune_runs/step1.log
  ```

- 依赖 `numpy`。数据来源与结论见 [`../test_data/README.md`](../test_data/README.md)。

### `spin_analysis.py` —— 陀螺自旋实验数据解析

- **作用**：解析 `chassis_tune` 实验 5（陀螺自旋实测）的串口抓取，评估陀螺外环性能。
- **输入格式**：`SET idx=N spin ref=.. mrad_s imu=1` + `STEP_BEGIN n=.. ch=8`
  + `n × 16 字节`（8 通道小端 int16：参考 wz、陀螺 wz、里程计 wz、陀螺 yaw、里程计 yaw、
  FL 轮速、BR 轮速、FL 占空比）。
- **输出**：每段目标角速度、稳态误差（%）、纹波 std / 峰峰值、超调、进入 ±5% 带时间、
  轮速与占空比；静止段输出零偏与 yaw 漂移（°/s）；本段陀螺/里程计总转角及比值。
- **用法**：

  ```bash
  python3 tools/spin_analysis.py test_data/spin_runs/20261018_spin_air.log
  ```

- 依赖 `numpy`。

### `uart_link_test.py` —— USART2 链路：协议参考实现 + 台架测试

`UserLib/Comm/uart_link_config.h` 协议的 **Python 参考实现**（帧编码/解码/CRC8 与 C 侧逐字节一致），
既是台架测试工具，也是 Jetson 上位机 C++ 工程（`~/gimbal` 的 `UartChassisLink`）
与 `codec_test.c` 交叉校验的参照。**改协议时三处一起改。**

台架模式（把 USB-TTL 接到 F407 的 USART2：PD5/PD6，115200 8N1）：

| 模式 | 作用 |
|---|---|
| `listen` | 只听反馈帧，验证 F407 是否在 100 Hz 回传（看 status 位/四轮转速） |
| `drive --vx/--vy/--wz --secs` | 持续发速度指令（m/s、rad/s），按住时间后停车 |
| `spin --rate --secs` | 小陀螺：发 SPIN 命令定速自旋 |
| `estop` / `clear` | 发急停 / 里程计清零命令 |
| `selfcheck` | 纯软件自检（不接串口）：帧编解码往返 |
| `vectors` | 输出 C 向量头文件（给 `codec_test.c` 用） |
| `check` | 综合台架检查（含低速行驶项，`--move` 控制 mm/s） |
| `soak` | 长时间浸泡测试（丢包/超时统计） |
| `wheels` | 四轮转速/方向检查 |

```bash
python3 tools/uart_link_test.py listen -p /dev/ttyUSB0
python3 tools/uart_link_test.py drive  -p /dev/ttyUSB0 --vx 0.1 --secs 3
python3 tools/uart_link_test.py vectors > /tmp/ulink_vectors.h   # 给 codec_test.c
```

> ⚠️ 坐标约定：`vx` 前进 / `vy` 左移 / `wz` 逆时针为正（底盘库约定，不是上位机
> `ChassisCommand` 的「右/前」老约定）。帧格式细节见 `UserLib/Comm/README.md`。

### `codec_test.c` —— 协议编解码 PC 端交叉校验

- **作用**：证明「C 侧编解码」与「Python 参考实现」逐字节一致 —— 帧格式写错（偏移/大小端/
  CRC 覆盖范围）在实机上只表现为"收不到/乱跳"，先在 PC 上按死。
- **依赖**：`uart_link_test.py vectors` 生成的向量头文件。
- **用法**：

  ```bash
  python3 tools/uart_link_test.py vectors > /tmp/ulink_vectors.h
  gcc -std=c11 -Wall -Wextra -IUserLib/Comm -I/tmp \
      -o /tmp/codec_test tools/codec_test.c && /tmp/codec_test
  ```

  输出 `OK` 即 C 与 Python 编解码完全一致；任何 `[FAIL]` 会打印 C/PY 两边的字节对比。

> `__pycache__/` 是 Python 自动生成的字节码缓存，删掉无害。

## 相关文档

| 想看什么 | 去哪 |
|---|---|
| 整定实验怎么跑、数据从哪来 | [`../UserLib/Chassis/README.md`](../UserLib/Chassis/README.md) §3.2 / §10 |
| 实测数据与历史结论 | [`../test_data/README.md`](../test_data/README.md) |
| USART2 帧格式 | [`../UserLib/Comm/README.md`](../UserLib/Comm/README.md) §4.1 |
| 陀螺自旋实验（实验 5） | [`../UserLib/IMU/README.md`](../UserLib/IMU/README.md) §11 |
