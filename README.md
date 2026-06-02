# Mini45 + ESP32/MC1081 三维力传感器标定上位机

本项目用于五通道柔性电容式三维力传感器的标定实验。上位机同时采集 ATI Mini45 六轴力/力矩、ESP32 + MC1081 五通道电容，并可通过 Arduino 控制三轴丝杆台自动加载。

上位机只负责实验控制和原始数据保存，不做时间戳匹配、插值、零点扣除、标准化、训练样本筛选或模型训练。为提高自动力控和 K 矩阵辨识的稳定性，界面提供 Mini45 上位机反馈滤波；该滤波只用于实时显示、稳定判定、K 辨识和电机控制，输出的原始时序文件仍保存未滤波的传感器坐标力和 Mini45 原始数据。

## 目录结构

```text
mini45_esp32_calibration/
  app/
    main.py                 # PyQt5 上位机入口
    gui.py                  # 主界面和实验流程
    recorder.py             # CSV 批次文件写入
    calibration.py          # 标定序列和训练轨迹生成
    force_frame.py          # Mini45 到传感器坐标映射
    force_filter.py         # Mini45 反馈滤波，服务于显示、K 辨识和力控
    force_control.py        # K 矩阵辨识与解耦力控算法
    stability.py            # 稳定判定和标定点生成
    esp32_serial.py         # ESP32 串口采集
    mini45_netft.py         # Mini45 NETBA / simulator
    arduino_motion.py       # Arduino 三轴电机控制
  arduino_motion_serial/    # Arduino 串口控制固件
  esp32_mc1081_stream/      # ESP32/MC1081 采集固件
  tests/                    # 无硬件单元测试
```

## 运行方式

推荐直接使用当前电脑的 Python 3.9 启动脚本：

```bat
run_app_python3_9.bat
```

也可以手动运行：

```bash
cd mini45_esp32_calibration
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
python -m app.main
```

## 上位机使用流程

一次完整实验应作为一个“实验批次”保存。同一个传感器、同一次安装、同一套完整标定流程都放在同一个批次文件夹里；换传感器、重新安装、重新粘接或换日期实验时，再新建批次。

推荐流程：

```text
1. 连接 ESP32、Mini45、Arduino。
2. 根据安装方向设置“传感器坐标映射”。
3. 填写实验批次/安装编号，例如 sensor01_mount01。
4. 点击“开始实验批次”。
5. 选择“空载零点漂移”，点击“开始标定”。
6. 点击“自动辨识 K”，完成三轴力-位移灵敏度矩阵辨识。
7. 选择“静态正反程标定”，加载轴选 Fz，点击“开始标定”。
8. 选择“静态正反程标定”，加载轴选 Fx，点击“开始标定”。
9. 选择“静态正反程标定”，加载轴选 Fy，点击“开始标定”。
10. 选择“训练数据采集”，按需要运行不同轨迹。
11. 全部完成后点击“结束实验批次”。
```

注意：

- `开始实验批次` 创建本次实验文件夹并打开所有输出文件。
- `开始标定` 只启动当前子实验，不创建新文件夹。
- 子实验完成后不会关闭批次，可以继续切换模式做下一项。
- `结束实验批次` 才关闭所有文件。
- 未开始实验批次时点击 `开始标定`，上位机会提示先开始实验批次。
- `传感器坐标映射` 应在开始实验批次前设置；批次开始或 K 已辨识后会锁定，避免同一批数据混用不同坐标系。
- `力控参数` 是实验批次级设置，固定显示在主界面中，不随实验模式切换隐藏或移动。
- 同一批次内通常只需点击一次 `自动辨识 K`，之后单目标、静态正反程和训练数据采集都会复用当前 K。
- 如果 Mini45 噪声较大，可启用 `Mini45上位机滤波`。推荐先使用 `截止Hz=3`、`中值点数=5`；若响应太慢可提高截止频率，若尖峰较多可保持 5 点中值滤波。
- 点击 `清零/偏置` 后，上位机会重置 Mini45 上位机滤波、清空稳定窗口和实时力曲线，避免清零前后的数据混入 K 辨识。

## 传感器坐标映射

Mini45 的原始坐标不一定等于待标定传感器坐标。上位机在 Mini45 数据进入主数据流前，先按界面设置把 Mini45 原始力转换为传感器坐标力：

```text
传感器 Fx = +/- Mini45 Fx/Fy/Fz
传感器 Fy = +/- Mini45 Fx/Fy/Fz
传感器 Fz = +/- Mini45 Fx/Fy/Fz
```

映射必须是一一对应，不能让多个传感器轴同时选择同一个 Mini45 轴。默认映射为 `传感器 Fx=+Mini45 Fx`、`传感器 Fy=+Mini45 Fy`、`传感器 Fz=+Mini45 Fz`。

设置完成后，上位机中的实时曲线、目标力、K 辨识、自动力控、稳定点和训练数据中的 `fx/fy/fz` 都表示传感器坐标。Mini45 原始值会额外保存为 `mini45_raw_*` 字段，便于后续排查。

## Mini45 上位机滤波

Mini45 网页低通滤波可以降低硬件输出噪声，但自动 K 辨识仍可能受到偶发尖峰或短时波动影响。上位机因此提供额外反馈滤波：

- `启用`：开启后，实时显示、稳定判定、K 辨识和自动力控使用滤波后的传感器坐标力。
- `截止Hz`：一阶低通截止频率。数值越低越平滑，但控制滞后越大；静态标定和 K 辨识可用 `2~5 Hz`。
- `中值点数`：用于抑制单点尖峰，建议 `3` 或 `5`。点数越大，尖峰越少，但响应更慢。
- `重置滤波`：清空滤波器历史状态。更改 Mini45 清零、重新接触或明显改变安装状态后建议重置。

注意：滤波后的力只进入上位机控制链路。`raw_timeseries.csv`、`zero_drift_timeseries_XXX.csv` 和 `training_raw_timeseries.csv` 中的 `fx/fy/fz` 仍为未滤波的传感器坐标力；`mini45_raw_*` 仍为 Mini45 原始坐标数据。

## 实验模式

### 空载零点漂移

不控制电机，只记录空载状态下的 Mini45 和电容原始数据。

- 设置 `零点采集时间 s`
- 到时间后自动结束该子实验
- 生成 `zero_drift_timeseries_001.csv`
- 如果同一批次内再次做零点漂移，会生成 `zero_drift_timeseries_002.csv`

上位机不生成零点统计摘要。均值、标准差、漂移和峰峰值应由后续数据分析项目根据原始数据计算。

### 单目标点标定

输入 `target_Fx,target_Fy,target_Fz`，上位机根据 Mini45 反馈和自动辨识得到的 `K` 矩阵闭环控制三轴电机逼近目标。三向力进入容差窗口并满足稳定判据后，自动写入 marker 和稳定标定点。

正式自动力控前需要先点击 `自动辨识 K`。若未完成有效 K 辨识，上位机会拒绝启动自动标定。

### 静态正反程标定

用于论文静态指标计算。

- `Fz` 序列默认：`0 -> 1 -> ... -> 9 -> 8 -> ... -> 0 N`
- `Fx/Fy` 序列默认：`0 -> +3.6 -> 0` 和 `0 -> -3.6 -> 0`
- 支持设置最大力、步长、循环次数、剪切方向
- 输出稳定点到 `calibration_points.csv`

### 训练数据采集

用于采集深度学习训练所需的连续原始时序数据。该模式不生成训练样本，不做预处理，只保存转换后的待测传感器坐标力、Mini45 原始力、电容数据以及轨迹阶段 marker。

训练模式不按固定时间播放目标轨迹。上位机会用转换后的传感器坐标力判断当前目标是否到达；到达后立即切换下一个目标，不做稳定保持。若当前目标超过最大等待时间仍未到达，默认自动跳过并记录超时 marker，避免单个不可达目标阻塞整段训练采集。

参数：

- `Fz 层级`：例如 `3,5,7,9`
- `轨迹类型`：`Fx往返 / Fy往返 / 斜向往返 / 随机小幅扰动`
- `剪切最大力 N`
- `目标步距 N`：往返和斜向轨迹中相邻目标点的力间隔
- `训练到达窗口 N`：判断当前目标是否已被实测力覆盖
- `最大等待时间 s`：目标长时间不可达时自动跳过，默认 60 s
- `随机点数`：随机扰动轨迹每个 Fz 层级生成的随机目标数量，默认 30

同一批次内多次运行不同训练轨迹时，数据会追加到同一组训练文件，不会覆盖。

## 输出文件

一次实验批次文件夹示例：

```text
runs/
  20260522_153000_sensor01_mount01/
    raw_timeseries.csv
    markers.csv
    calibration_points.csv
    zero_drift_timeseries_001.csv
    training_raw_timeseries.csv
    training_markers.csv
    force_frame_mapping.csv
    force_control_k.csv
    force_control_log.csv
```

### raw_timeseries.csv

静态标定相关原始异步时序，包括空载零点、Fz、Fx、Fy 等子实验。

字段：

```text
timestamp,monotonic_s,source,
fx,fy,fz,mx,my,mz,
mini45_raw_fx,mini45_raw_fy,mini45_raw_fz,
mini45_raw_mx,mini45_raw_my,mini45_raw_mz,
c0,c1,c2,c3,c4,
mini45_sequence,mini45_status,
esp_ms,esp_sequence
```

Mini45 行保存转换后的传感器坐标力/力矩，同时保存 Mini45 原始力/力矩；ESP32 行保存电容，力/力矩为空。

### markers.csv

静态标定事件 marker，用于把 `raw_timeseries.csv` 与实验阶段对应起来。

字段：

```text
timestamp,marker_id,experiment_id,cycle_id,
branch,axis,direction,preload_N,
target_Fx,target_Fy,target_Fz,note
```

### calibration_points.csv

稳定标定点文件，用于论文静态指标和传统标定分析。每行是一个稳定窗口统计点，转换后的传感器坐标力均值是真实力标签。

### zero_drift_timeseries_XXX.csv

单次空载零点漂移原始时序，字段与 `raw_timeseries.csv` 一致。该文件便于后续单独分析零点稳定性，但上位机不计算统计摘要。

### training_raw_timeseries.csv

训练数据采集模式的原始异步时序。字段与 `raw_timeseries.csv` 一致，只保存训练采集过程。

该文件不做：

- 时间戳匹配
- 插值
- 重采样
- 零点扣除
- 训练预处理滤波
- 标准化
- 样本剔除

### training_markers.csv

训练轨迹阶段 marker，用于后续训练项目切分连续轨迹。

常见 `note` 包括：

- `training_start`
- `fz_level_start`
- `target_start`
- `target_reached`
- `target_timeout`
- `target_skipped`
- `recovery_start`
- `training_end`

字段：

```text
timestamp,marker_id,experiment_id,cycle_id,
trajectory_type,phase,axis,direction,branch,
target_Fx,target_Fy,target_Fz,
target_shear_N,target_angle_deg,note
```

### force_frame_mapping.csv

记录本批次使用的 Mini45 到传感器坐标映射。字段包括：

```text
timestamp,experiment_id,
sensor_Fx_from,sensor_Fx_sign,
sensor_Fy_from,sensor_Fy_sign,
sensor_Fz_from,sensor_Fz_sign
```

### force_control_k.csv

记录每次自动 `K` 矩阵辨识结果，包括扰动位移、扰动前后传感器坐标三向力均值、`K` 九个元素、奇异值、条件数、有效性和失败原因。

### force_control_log.csv

记录自动力控每一步的目标力、当前力、三向误差、`ΔX/ΔY/ΔZ`、脉冲量、自动阻尼、信任域比例和预测力变化。该文件用于复现实验控制过程和排查自动加载问题。

## 硬件连接

电脑需要同时连接：

- USB 连接 ESP32：采集五通道电容
- 网线连接 Mini45 NETBA：采集六轴力/力矩
- USB 连接 Arduino Mega：控制三轴电机

手动小步调试区域使用固定的力轴到电机轴映射，默认关系：

```text
Mini45 Fx -> Z 电机
Mini45 Fy -> Y 电机
Mini45 Fz -> X 电机
```

如果点击力轴正向小步后 Mini45 对应力反向变化，应优先检查 Mini45 坐标符号配置和电机方向设置。

正式自动力控不使用上述单轴映射，而使用自动辨识得到的完整 `3×3` 灵敏度矩阵。矩阵列固定对应 Arduino `X/Y/Z` 电机轴，行固定对应传感器坐标 `Fx/Fy/Fz`。

## ESP32 串口协议

默认波特率 `115200`，可根据稳定性提高到 `921600`。

命令：

```text
INFO
CAPTURE
START,50
STOP
```

数据：

```text
DATA0,c0,c1,c2,c3,c4
CAP,esp_ms,seq,c0,c1,c2,c3,c4
L:message
E:code,message
```

正式实验建议使用流式采集模式 `START,<rate_hz>`。

## Mini45 / NETBA 配置

默认参数：

- NETBA IP：`192.168.1.1`
- UDP 端口：`49152`
- 静态标定推荐 RDT 输出频率：`100~200 Hz`

Win11 网卡建议设置为同网段静态 IP，例如：

```text
IP 地址: 192.168.1.10
子网掩码: 255.255.255.0
网关: 可留空或 192.168.1.1
```

排查顺序：

1. 浏览器打开 `http://192.168.1.1`，确认 NETBA 网页可访问。
2. 上位机点击 `读取系数`，读取力/力矩比例系数。
3. 若显示连接但无实时数据，检查 Windows 防火墙是否拦截 UDP。
4. 必要时用 Wireshark 过滤 `udp.port == 49152` 查看是否有 NETBA 回包。

## Arduino 三轴电机控制

当前丝杆台参数：

```text
丝杆导程: 2 mm/rev
驱动脉冲: 400 pulse/rev
位移分辨率: 0.005 mm/pulse
```

`0.005 mm` 只是 1 个脉冲，肉眼几乎看不到，也可能被丝杆间隙或静摩擦抵消。建议首次调试使用较低目标力和较小加载范围，例如：

```text
Fz = 0.5 N
剪切最大力 = 0.3 N
```

自动力控仍使用 Arduino 固件已有的 `MOVE_MM` 指令，不需要修改 Arduino 串口协议。

## 后续数据分析建议

上位机输出的是原始数据和 marker。后续项目应基于这些文件完成：

- 时间戳匹配
- Mini45 与电容同步
- 零点扣除
- 滤波
- 标准化
- 训练样本切片
- 静态指标计算
- 深度学习模型训练

建议用途：

- `calibration_points.csv`：论文静态指标、传统标定模型
- `zero_drift_timeseries_XXX.csv`：零点漂移分析
- `training_raw_timeseries.csv + training_markers.csv`：深度学习实时力预测模型训练
- `force_control_k.csv + force_control_log.csv`：自动力控过程复核和论文方法说明

更具体的标定控制算法和实验技术细节见 `CALIBRATION_TECHNICAL_DETAILS.md`。

## 测试

无硬件测试：

```bash
python -m unittest discover -s tests
```
