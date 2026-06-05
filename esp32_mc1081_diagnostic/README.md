# MC1081 独立诊断工具

本文件夹用于单独排查 MC1081 电容采集链路稳定性，不依赖 Mini45 标定上位机。

## 文件

- `esp32_mc1081_diagnostic.ino`：ESP32/MC1081 诊断固件。
- `mc1081_diagnostic_host.py`：配套 Python 上位机，读取串口 `DIAG` 数据并保存 CSV。
- `captures/`：上位机运行后自动创建，保存每次测试数据。

## 固件输出

诊断固件输出行格式：

```text
DIAG,esp_ms,seq,valid,error,status,overflow,dref,d0,d1,d2,d3,d4,c0,c1,c2,c3,c4,dt_us
```

字段含义：

- `dref`：参考通道原始计数。
- `d0~d4`：五个测量通道原始计数。
- `c0~c4`：由 `d0~d4 / dref * Cref` 换算得到的电容值，单位 pF。
- `status`：MC1081 状态寄存器。
- `overflow`：OSC2 溢出标志。
- `valid`：本次采样是否有效，`1` 为有效，`0` 为异常。
- `error`：诊断固件内部错误码，`0` 表示无错误。
- `dt_us`：本次测量耗时，单位 us。

## 上位机运行

先列出串口：

```powershell
python .\mc1081_diagnostic_host.py --list
```

连续采集 10 分钟，10 Hz：

```powershell
python .\mc1081_diagnostic_host.py --port COM6 --duration 600 --rate 10 --label fixed_cap_test
```

连续采集 30 分钟：

```powershell
python .\mc1081_diagnostic_host.py --port COM6 --duration 1800 --rate 10 --label zero_drift_long
```

一直采到手动停止：

```powershell
python .\mc1081_diagnostic_host.py --port COM6 --duration 0 --rate 10 --label manual_stop
```

改变诊断配置：

```powershell
python .\mc1081_diagnostic_host.py --port COM6 --rate 10 --cavg 4 --discard 1 --i2c 100000 --reset --label cavg4_discard1
```

## 输出文件

每次运行会在 `captures/` 下新建一个文件夹，例如：

```text
captures/
  20260605_223000_fixed_cap_test/
    mc1081_diagnostic_raw.csv
    serial_log.txt
    metadata.json
```

- `mc1081_diagnostic_raw.csv`：后续数据分析主要读取这个文件。
- `serial_log.txt`：保存全部串口行，包括启动日志、错误行和 DIAG 行。
- `metadata.json`：保存本次采集参数、输出路径、行数统计。

## 建议测试顺序

1. 固定电容接入 C0~C4，运行 `--duration 600 --rate 10`。
2. 交换 C3 与其他通道接线，观察跳变跟着物理通道走还是跟着传感器电极走。
3. 电机驱动断电，只保留 ESP32 和 MC1081，重复测试。
4. 改变 `--cavg 1/4`、`--discard 0/1/2`，比较单点跳变和长时漂移。
5. 若 `dref` 同时跳变，优先检查参考电容、供电和 MC1081 配置；若只有某个 `dX` 跳变，优先检查对应通道输入、线缆和焊接。
