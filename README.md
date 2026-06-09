# Mini45 + ESP32/MC1081 三维力传感器标定上位机

本项目用于五通道柔性电容式三维力传感器的自动标定与训练原始数据采集。

上位机同时连接：

- ATI Mini45 + NETBA：提供六轴力/力矩反馈。
- ESP32 + MC1081：采集 `C0~C4` 五通道电容。
- Arduino 三轴电机控制器：接收 `MOVE_MM` 指令控制丝杆台。

上位机负责设备控制、实验编排和原始数据保存。时间戳匹配、插值、零点扣除、滤波、标准化、静态指标计算和模型训练应在后续独立数据分析项目中完成。

## 目录结构

```text
mini45_esp32_calibration/
├─ app/                         Python 上位机
├─ esp32_mc1081_stream/         正式 ESP32/MC1081 采集固件
├─ arduino_motion_serial/       Arduino 三轴电机串口固件
├─ tests/                       单元测试
├─ runs/                        默认实验批次输出目录
├─ run_app_python3_9.bat        当前电脑推荐启动脚本
├─ requirements.txt             Python 依赖
├─ config.example.yaml          配置示例
└─ CALIBRATION_TECHNICAL_DETAILS.md
```

`esp32_mc1081_diagnostic/` 是独立诊断工具，不属于正式标定流程，也不上传 GitHub。

## 安装与运行

推荐使用现有 Conda `python3-9` 环境：

```powershell
conda activate python3-9
pip install -r requirements.txt
python -m app.main
```

当前电脑也可直接运行：

```powershell
.\run_app_python3_9.bat
```

运行测试：

```powershell
python -m unittest discover -s tests
python -m compileall app tests
```

## 设备连接

1. ESP32 通过 USB 串口连接电脑。
2. Arduino 通过独立 USB 串口连接电脑。
3. Mini45 NETBA 通过网线连接电脑，填写正确 IP、UDP 端口、力计数/单位和力矩计数/单位。
4. 在上位机中设置 Mini45 到待测传感器的坐标映射。
5. 开始正式实验前确认三台设备均有实时数据，电机方向和安全限位正确。

上位机正式力控、稳定判定和标定标签均使用坐标转换后的待测传感器 `Fx/Fy/Fz`。Mini45 原始坐标数据通过 `mini45_raw_*` 字段保留。

## MC1081 采集配置

正式 ESP32 固件支持三种采集配置：

| 配置 | CNT | CAVG | 预计频率 | 用途 |
|---|---:|---:|---:|---|
| `STATIC_PRECISION` | 255 | 32 | 约 2.26 Hz | 零点漂移和静态标定 |
| `TRAINING_BALANCED` | 191 | 8 | 约 11.36 Hz | 主要训练数据 |
| `TRAINING_FAST` | 255 | 1 | 约 50 Hz | 高速补充训练数据 |

串口命令：

```text
PROFILE,STATIC_PRECISION
PROFILE,TRAINING_BALANCED
PROFILE,TRAINING_FAST
GET_PROFILE
START,<rate_hz>
STOP
CAPTURE
INFO
```

配置切换后，ESP32 会重新配置 MC1081，并在正式输出前丢弃前 5 个转换结果。

流式 CAP 数据格式：

```text
CAP,<esp_ms>,<seq>,<C0>,<C1>,<C2>,<C3>,<C4>,<profile>,<cnt>,<cavg>,<nominal_hz>
```

## 推荐使用流程

### 一键完整自动实验

1. 连接 ESP32、Mini45 和 Arduino。
2. 设置传感器坐标映射、实验批次/安装编号和输出目录。
3. 检查空载力、力矩和电机安全状态。
4. 点击 `开始完整自动实验`。
5. 等待完整流程结束，或使用暂停、继续、停止/急停处理异常。

完整自动流程：

```text
设备检查并创建实验批次
→ STATIC_PRECISION
→ 空载零点漂移
→ 自动辨识 K
→ Fz、Fx、Fy 静态正反程标定
→ TRAINING_BALANCED 全部训练轨迹
→ TRAINING_FAST 相同训练轨迹
→ 自动回零
→ 关闭实验批次
```

自动流程运行期间，关键实验参数和实验模式会锁定，避免误操作。界面显示当前阶段、采集配置、实际频率、静态点稳定进度、`45` 帧电容采集进度以及失败/跳过数量。

### 手动子实验

保留原有手动子实验入口，供调试和补测使用：

- 空载零点漂移
- 单目标点标定
- 静态正反程标定
- 训练数据采集
- K 自动辨识

手动子实验需要先点击 `开始实验批次`。

## 静态标定规则

完整自动流程中的静态标定固定使用：

```text
|Fx - target_Fx| <= 0.05 N
|Fy - target_Fy| <= 0.05 N
|Fz - target_Fz| <= 0.08 N
```

每个静态目标点执行：

```text
自动逼近目标
→ 连续稳定保持 5 s
→ 收集 45 个唯一、完整、有效的 STATIC_PRECISION CAP 帧
→ 使用第 1 到第 45 个电容样本时间范围内的 Mini45 数据计算标定点
```

开始采集前仍必须进入上述严格窗口并连续稳定 5 秒。采集开始后改用按量程放宽的保持窗口：

```text
Fx/Fy 采集保持窗口：±0.06 N
Fz 采集保持窗口：±0.10 N
```

采集期间短暂超出保持窗口时，上位机会暂停接收新的电容样本，并保留已经采集的样本；5 秒内恢复后继续采集。持续越界超过 5 秒时：

- 进度低于 80%（默认少于 36/45 帧）：清空当前点并重新稳定。
- 进度达到 80%：继续保留已有样本并等待力恢复，直到完成或触发单点总超时。

这样可避免已经采集四十多个样本时因瞬时波动全部重来。最终标定点仍执行严格均值、标准差和电容质量判定。

单点超时后最多自动重试两次，仍失败则记录无效 marker 并继续下一点。

`calibration_points.csv` 同时保存原始均值、标准差和每个字段独立剔除 P01/P99 外极端值后的 `*_trimmed_mean`。后续静态指标计算优先使用 `*_trimmed_mean`。

## 训练数据采集

平衡频率和高速补充训练均使用相同的：

- Fz 层级：`1,2,3,4,5,6,7,8,9 N`
- 训练轨迹：Fx 往返、Fy 往返、斜向往返、随机小幅扰动
- 目标力变化速率：`0.20 N/s`
- 同一批次随机种子和随机目标

两档训练只改变 MC1081 采集配置，用于比较采样频率与测量质量。训练文件只保存原始异步时序，不进行任何训练预处理。

## 输出文件

每次实验批次创建独立文件夹：

```text
runs/<日期时间_实验编号>/
├─ raw_timeseries.csv
├─ markers.csv
├─ calibration_points.csv
├─ zero_drift_timeseries_001.csv
├─ training_balanced_raw_timeseries.csv
├─ training_balanced_markers.csv
├─ training_fast_raw_timeseries.csv
├─ training_fast_markers.csv
├─ force_control_k.csv
├─ force_control_log.csv
├─ force_frame_mapping.csv
└─ workflow_events.csv
```

主要用途：

- `raw_timeseries.csv`：静态实验相关原始异步时序。
- `markers.csv`：静态实验事件和目标信息。
- `calibration_points.csv`：静态标定点统计，用于论文静态指标。
- `zero_drift_timeseries_XXX.csv`：空载零点漂移原始数据。
- `training_balanced_*`：平衡频率训练原始数据和轨迹 marker。
- `training_fast_*`：高速补充训练原始数据和轨迹 marker。
- `force_control_k.csv`：K 辨识结果。
- `force_control_log.csv`：自动力控过程。
- `force_frame_mapping.csv`：本批次坐标映射。
- `workflow_events.csv`：配置切换、阶段切换、重试、跳过和异常终止记录。

原始时序中的 ESP32 样本包含：

```text
cap_profile,mc1081_cnt,mc1081_cavg,cap_nominal_hz,cap_effective_hz
```

## 安全要求

- 初次验证完整自动流程时，应使用低力、小范围、单循环参数。
- 保持机械限位开关有效，并确保急停可用。
- K 辨识失败、MC1081 配置验证失败、设备断开、力或力矩超限时，完整自动流程会终止并关闭已创建的数据文件。
- 更换传感器、重新安装、重新粘接或改变坐标映射后，必须创建新批次并重新辨识 K。

具体坐标定义、K 辨识和解耦力控原理见 `CALIBRATION_TECHNICAL_DETAILS.md`。
