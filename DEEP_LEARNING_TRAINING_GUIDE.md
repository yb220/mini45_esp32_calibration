# 八传感器共享深度学习模型训练建议

本文档用于规划五通道柔性电容式三维力传感器的深度学习训练和 ESP32-S3 部署方案。目标是训练一个可共享给 8 个完全相同传感器使用的模型，并在实际使用时实时输出每个传感器的三维力 `Fx,Fy,Fz`。

## 基本假设

- 8 个传感器结构、材料、尺寸和电容采集通道定义一致。
- 每个传感器输出 `C0,C1,C2,C3,C4`。
- 单个传感器对应输出 `Fx,Fy,Fz`。
- 8 个传感器安装位置不同，但受力相互独立，不考虑传感器之间的机械耦合。
- ESP32-S3-N16R8 负责采集 8 个传感器，并通过 PCA9548A 进行 I2C 通道复用。
- 训练在电脑端完成，ESP32-S3 只负责模型推理。
- Mini45 仍然作为训练数据采集时的真实力标签来源。

## 总体建议

建议采用“单传感器共享模型”：

```text
输入: 单个传感器最近一段时间的 C0~C4
输出: 该传感器当前 Fx,Fy,Fz
```

ESP32-S3 实际运行时对 8 个传感器循环调用同一个模型：

```text
sensor_0: C0~C4 历史窗口 -> Fx,Fy,Fz
sensor_1: C0~C4 历史窗口 -> Fx,Fy,Fz
...
sensor_7: C0~C4 历史窗口 -> Fx,Fy,Fz
```

这样做的优点：

- 模型参数只保存一份，Flash 和 RAM 占用低。
- 训练数据可以来自一个或多个传感器，数据格式统一。
- 8 个传感器之间无耦合时，不需要训练 40 输入、24 输出的大模型。
- 后续如果某个传感器零点略有差异，可以通过每个传感器独立零点归一化处理，而不是重新训练 8 个模型。

## 推荐模型路线

### 第一阶段：MLP 基线模型

先训练一个简单 MLP 作为基线：

```text
输入特征:
当前 ΔC0~ΔC4
最近窗口均值 mean(ΔC0~ΔC4)
最近窗口斜率 dC/dt
最近窗口峰峰值 ptp(ΔC0~ΔC4)

输出:
Fx,Fy,Fz
```

推荐结构：

```text
Dense 32 + ReLU
Dense 16 + ReLU
Dense 3
```

用途：

- 快速验证电容数据是否能拟合三维力。
- 作为后续 1D-CNN/TCN 的对照组。
- 部署最简单，适合先在 ESP32-S3 上验证完整推理链路。

### 第二阶段：小型 1D-CNN

如果要考虑迟滞、蠕变和加载历史，推荐使用小型 1D-CNN。

推荐输入：

```text
采样率: 50 Hz
窗口长度: 0.5~1.0 s
输入尺寸: 25~50 个时间点 x 5 通道
```

推荐结构：

```text
Conv1D 8 filters, kernel=5, ReLU
Conv1D 16 filters, kernel=3, ReLU
GlobalAveragePooling1D
Dense 16, ReLU
Dense 3
```

优点：

- 能利用短时间历史信息。
- 推理比 LSTM/GRU 更容易优化。
- 适合 int8 量化后部署到 ESP32-S3。

### 第三阶段：轻量 TCN

如果 1D-CNN 对迟滞和蠕变拟合不够，可以尝试轻量 TCN。

推荐结构：

```text
Dilated Conv1D 8 filters, kernel=3, dilation=1
Dilated Conv1D 8 filters, kernel=3, dilation=2
Dilated Conv1D 16 filters, kernel=3, dilation=4
GlobalAveragePooling1D
Dense 16
Dense 3
```

注意事项：

- TCN 比普通 1D-CNN 更能看见较长历史。
- 模型不要过深，否则 ESP32-S3 推理延迟会增加。
- 第一版窗口建议不超过 `50 x 5`。

### 不推荐作为第一版的模型

`LSTM/GRU` 可以考虑，但不是第一优先级。它们对时间序列有效，但部署和优化比 1D-CNN/TCN 麻烦。

`Transformer Encoder` 不建议部署在 ESP32-S3 上作为实时模型。它对内存和计算量要求较高，除非模型极小、窗口很短并且严格量化。

## 训练数据采集建议

深度学习训练应以连续同步时序为主，不只使用稳定点均值。

每条训练样本应来自：

```text
过去一段时间的 C0~C4 序列 -> 当前 Mini45 Fx,Fy,Fz
```

建议采集以下数据：

1. 空载零点漂移
   - 每次安装后采集 `2~5 min`。
   - 用于零点归一化、漂移分析和模型基线数据。

2. 单轴连续加载
   - `Fz: 0 -> 9 -> 0 N`
   - `Fx: 0 -> +3.6 -> 0 -> -3.6 -> 0 N`
   - `Fy: 0 -> +3.6 -> 0 -> -3.6 -> 0 N`
   - 保留加载、卸载、稳定、回零全过程。

3. 预载下剪切连续加载
   - `Fz = 3,5,7,9 N`
   - 在每个预载下分别做 `Fx` 和 `Fy` 往返加载。

4. 组合连续轨迹
   - 不作为论文静态指标主数据，主要用于训练实际使用模型。
   - 推荐方向：`45,135,225,315 deg`
   - 剪切合力轨迹：`0 -> 3.6 -> 0 N`
   - 可增加小幅随机扰动，覆盖实际使用中的非理想路径。

## 建议训练原始文件格式

上位机采集阶段只保存原始异步数据，不在采集软件中做时间戳匹配或训练预处理。

训练采集模式生成：

```text
training_raw_timeseries.csv
training_markers.csv
```

`training_raw_timeseries.csv` 每行是 Mini45 或 ESP32 的一条原始采样：

```text
timestamp,monotonic_s,source,
fx,fy,fz,mx,my,mz,
c0,c1,c2,c3,c4,
mini45_sequence,mini45_status,
esp_ms,esp_sequence
```

`training_markers.csv` 记录组合加载轨迹阶段：

```text
timestamp,marker_id,experiment_id,cycle_id,
trajectory_type,phase,axis,direction,branch,
target_Fx,target_Fy,target_Fz,
target_shear_N,target_angle_deg,
note
```

字段说明：

- `source`：原始数据来源，`mini45` 或 `esp32`。
- `C0~C4`：原始电容。
- `Fx,Fy,Fz`：Mini45 原始力数据，后续训练项目再按时间戳与电容匹配。
- `phase`：当前轨迹阶段，例如 `preload,moving,holding,recovery`。
- `target_*`：控制目标，只作为实验标签，不作为真实力标签。

时间戳匹配、插值、零点扣除、滤波、标准化和样本筛选全部在后续独立训练项目中完成。

## 数据预处理建议

### 零点归一化

每个传感器独立计算零点：

```text
ΔCi = Ci - Ci_zero
```

即使 8 个传感器结构相同，也建议保留独立零点。这样共享模型对不同安装位置和初始电容更稳。

### 标准化

训练集统计全局均值和标准差：

```text
X_norm = (X - mean_train) / std_train
```

部署到 ESP32-S3 时，需要把 `mean_train` 和 `std_train` 固化到固件中。

### 时间窗口

对 1D-CNN/TCN，建议窗口：

```text
25 点 x 5 通道   对应 50 Hz 下 0.5 s
50 点 x 5 通道   对应 50 Hz 下 1.0 s
```

如果模型延迟要求高，优先用 `25 x 5`。

## 数据集划分

不要把连续时序随机逐行打散后再划分训练集和测试集，这会导致相邻时间点泄漏。

推荐按实验段划分：

```text
训练集: 若干完整加载循环
验证集: 另一些完整加载循环
测试集: 独立日期、独立重新安装、独立传感器或独立轨迹
```

如果最终 8 个传感器都使用共享模型，建议测试时至少包含：

- 未参与训练的传感器编号。
- 未参与训练的组合加载轨迹。
- 未参与训练的重新安装数据。

## 损失函数和评价指标

训练损失建议：

```text
Loss = MSE(Fx,Fy,Fz)
```

如果三个方向量程不同，建议对输出力归一化，或使用加权损失：

```text
Loss = wx*MSE(Fx) + wy*MSE(Fy) + wz*MSE(Fz)
```

评价指标建议：

```text
MAE
RMSE
R2
最大绝对误差
Fx/Fy/Fz 分轴误差
不同 phase 下误差
不同 Fz 预载下误差
不同剪切方向下误差
```

必须单独报告：

- `stable` 阶段误差：代表静态准确性。
- `moving` 阶段误差：代表实时使用准确性。
- `unloading/recovery` 阶段误差：代表迟滞和回零恢复能力。

## ESP32-S3 部署建议

### 推荐部署方式

第一版建议：

```text
电脑端训练 PyTorch/TensorFlow 模型
导出 ONNX 或 TFLite
int8 量化
转换为 ESP-DL 或 TensorFlow Lite Micro 可部署模型
ESP32-S3 上只做推理
```

优先考虑：

- ESP-DL
- TensorFlow Lite Micro

### 推理流程

ESP32-S3 对 8 个传感器维护 8 个环形窗口：

```text
sensor_buffer[8][window_len][5]
```

每次完成一轮 8 传感器采集后：

```text
for sensor_id in 0..7:
    更新该传感器 C0~C4 窗口
    执行零点扣除和标准化
    调用共享模型推理
    得到 Fx,Fy,Fz
```

### 推荐运行频率

建议先按以下目标设计：

```text
8 个传感器总输出频率: 20~50 Hz
单个传感器模型窗口: 25~50 点
模型推理: int8
```

如果 8 个传感器采集和推理同时运行后出现丢帧，优先降低：

- 输出频率
- 窗口长度
- 卷积 filters 数量
- Dense 层宽度

### 上传到上位机的数据

调试阶段建议同时上传原始电容和推理结果：

```text
timestamp,seq,
S0_C0,S0_C1,S0_C2,S0_C3,S0_C4,S0_Fx,S0_Fy,S0_Fz,
...
S7_C0,S7_C1,S7_C2,S7_C3,S7_C4,S7_Fx,S7_Fy,S7_Fz
```

正式使用阶段可以只上传推理后的力：

```text
timestamp,seq,
S0_Fx,S0_Fy,S0_Fz,
...
S7_Fx,S7_Fy,S7_Fz
```

但建议固件保留调试命令，可以切换是否上传原始电容，便于排查传感器异常。

## 推荐实施顺序

1. 先用单个传感器采集完整训练数据。
2. 训练 MLP 基线，确认 `C0~C4 -> Fx,Fy,Fz` 可拟合。
3. 训练小型 1D-CNN，比较 `stable/moving/unloading` 各阶段误差。
4. 如果 1D-CNN 动态误差不够，再尝试轻量 TCN。
5. 将最小可用模型 int8 量化并部署到 ESP32-S3。
6. ESP32-S3 先跑 1 个传感器实时推理。
7. 再扩展到 PCA9548A 复用的 8 个传感器轮询推理。
8. 最后优化上传协议和上位机显示。

## 对上位机采集模式的要求

上位机只需要完成：

- `训练数据采集模式`，区别于论文静态标定模式。
- `training_raw_timeseries.csv` 原始异步时序保存。
- `training_markers.csv` 轨迹阶段保存。
- 组合连续轨迹采集，而不是只记录组合稳定点。

核心原则：

```text
论文静态指标使用 calibration_points.csv
深度学习实时模型训练从 training_raw_timeseries.csv 和 training_markers.csv 开始
ESP32-S3 部署使用小型 int8 MLP / 1D-CNN / TCN
8 个同型独立传感器使用同一个共享模型循环推理
```
