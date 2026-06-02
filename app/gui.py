from __future__ import annotations

import queue
import time
from pathlib import Path

try:
    import serial.tools.list_ports
except ImportError:  # pragma: no cover
    serial = None

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QComboBox,
    QCheckBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)
import pyqtgraph as pg

from .buffers import SampleBuffer
from .calibration import (
    CalibrationTarget,
    TrainingTarget,
    generate_training_trajectory,
    generate_fz_sequence,
    generate_shear_sequence,
    parse_force_levels,
    training_target_reached,
    training_target_timed_out,
)
from .arduino_motion import (
    ArduinoMotionAdapter,
    AUTO_DEFAULT_INTERVAL_S,
    AUTO_DEFAULT_MAX_STEP_MM,
    AUTO_DEFAULT_SPEED_MM_S,
    DEFAULT_FORCE_TO_MOTOR,
    DEFAULT_FORCE_TO_MOTOR_SIGN,
    MANUAL_DEFAULT_SPEED_MM_S,
    MANUAL_DEFAULT_STEP_MM,
    MM_PER_PULSE,
    MotionMessage,
    PULSES_PER_MM,
    PULSES_PER_REV,
    SCREW_LEAD_MM,
    mapped_motor_delta,
    mm_to_pulses,
    parse_axis_position,
)
from .esp32_serial import Esp32Log, Esp32SerialAdapter
from .force_filter import ForceFilterSettings, ForceLowPassFilter
from .force_frame import AxisFrameMap, ForceFrameMapping, transform_force_sample
from .force_control import (
    MOTOR_AXES,
    DecoupledControlSettings,
    DecoupledControlState,
    compute_decoupled_command,
    force_stats,
    force_vector_from_sample,
    identify_k_matrix,
)
from .mini45_netft import Mini45Log, Mini45NetFTAdapter, Mini45Simulator, fetch_netft_config
from .models import (
    CapSample,
    CombinedSnapshot,
    ExperimentMeta,
    ForceSample,
    SafetySettings,
    StabilitySettings,
)
from .recorder import CsvRecorder
from .stability import build_calibration_point, evaluate_three_axis_stability


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Mini45 + ESP32/MC1081 三维力标定上位机")
        self.resize(1280, 860)

        self.buffer = SampleBuffer(max_seconds=600)
        self.esp32 = None
        self.mini45 = None
        self.motion: ArduinoMotionAdapter | None = None
        self.recorder: CsvRecorder | None = None
        self.marker_id = 0
        self.last_force_time = 0.0
        self.last_cap_time = 0.0
        self.latest_force_sample: ForceSample | None = None
        self.force_filter = ForceLowPassFilter()
        self.motion_positions = {"X": None, "Y": None, "Z": None}
        self.auto_force_active = False
        self.auto_force_holding = False
        self.auto_force_marker_done = False
        self.auto_force_in_window_since = 0.0
        self.auto_force_last_move = 0.0
        self.auto_force_next_move_time = 0.0
        self.motion_last_query = 0.0
        self.calibration_mode = ""
        self.calibration_paused = False
        self.sequence_targets: list[CalibrationTarget] = []
        self.sequence_index = 0
        self.active_target: CalibrationTarget | None = None
        self.current_cycle_id = "cycle_001"
        self.zero_drift_count = 0
        self.zero_drift_active = False
        self.zero_drift_start_s = 0.0
        self.zero_drift_samples: list[CombinedSnapshot] = []
        self.zero_drift_file = ""
        self.training_count = 0
        self.training_active = False
        self.training_targets: list[TrainingTarget] = []
        self.training_target_index = 0
        self.training_target_start_s = 0.0
        self.training_current_target: TrainingTarget | None = None
        self.training_pause_started_s = 0.0
        self.force_control_result = None
        self.force_control_state = DecoupledControlState()
        self.k_ident_active = False
        self.k_ident_axis_index = 0
        self.k_ident_phase = ""
        self.k_ident_phase_start_s = 0.0
        self.k_ident_wait_until_s = 0.0
        self.k_ident_before_means: dict[str, list[float]] = {}
        self.k_ident_after_means: dict[str, list[float]] = {}
        self.k_ident_before_stds: dict[str, list[float]] = {}
        self.k_ident_after_stds: dict[str, list[float]] = {}
        self.force_mapping_error_logged = False

        self.force_x: list[float] = []
        self.force_y = {key: [] for key in ("fx", "fy", "fz")}
        self.cap_x: list[float] = []
        self.cap_y = {key: [] for key in ("c0", "c1", "c2", "c3", "c4")}

        self._build_ui()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(50)

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        conn = QHBoxLayout()
        conn.addWidget(self._build_esp32_group(), stretch=1)
        conn.addWidget(self._build_mini45_group(), stretch=1)
        conn.addWidget(self._build_motion_group(), stretch=1)
        layout.addLayout(conn)

        layout.addWidget(self._build_force_frame_group())
        layout.addWidget(self._build_force_control_group())
        layout.addWidget(self._build_calibration_group())

        status_plots = QHBoxLayout()
        status_plots.addWidget(self._build_status_group(), stretch=1)
        status_plots.addWidget(self._build_record_group(), stretch=1)
        layout.addLayout(status_plots)

        plot_layout = QHBoxLayout()
        self.force_plot = pg.PlotWidget(title="传感器坐标力数据")
        self.force_plot.setBackground("w")
        self.force_plot.addLegend()
        self.force_curves = {
            "fx": self.force_plot.plot([], [], pen=pg.mkPen("r", width=2), name="Fx"),
            "fy": self.force_plot.plot([], [], pen=pg.mkPen("g", width=2), name="Fy"),
            "fz": self.force_plot.plot([], [], pen=pg.mkPen("b", width=2), name="Fz"),
        }
        self.cap_plot = pg.PlotWidget(title="五通道电容 C0-C4")
        self.cap_plot.setBackground("w")
        self.cap_plot.addLegend()
        colors = {"c0": "r", "c1": "g", "c2": "b", "c3": "m", "c4": "k"}
        self.cap_curves = {
            key: self.cap_plot.plot([], [], pen=pg.mkPen(color, width=2), name=key.upper())
            for key, color in colors.items()
        }
        plot_layout.addWidget(self.force_plot)
        plot_layout.addWidget(self.cap_plot)
        layout.addLayout(plot_layout, stretch=1)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.document().setMaximumBlockCount(1000)
        layout.addWidget(self.log)

    def _build_esp32_group(self) -> QGroupBox:
        box = QGroupBox("ESP32 / MC1081 电容采集")
        form = QFormLayout(box)
        row = QHBoxLayout()
        self.esp_port = QComboBox()
        self.refresh_ports()
        self.refresh_btn = QPushButton("刷新")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        row.addWidget(self.esp_port, stretch=1)
        row.addWidget(self.refresh_btn)
        form.addRow("串口", row)

        self.esp_baud = QComboBox()
        self.esp_baud.addItems(["115200", "921600"])
        form.addRow("波特率", self.esp_baud)
        self.esp_mode = QComboBox()
        self.esp_mode.addItem("流式采集", "stream")
        self.esp_mode.addItem("定时轮询", "poll")
        form.addRow("采集模式", self.esp_mode)
        self.esp_rate = QSpinBox()
        self.esp_rate.setRange(1, 200)
        self.esp_rate.setValue(50)
        form.addRow("频率 Hz", self.esp_rate)
        self.esp_btn = QPushButton("连接 ESP32")
        self.esp_btn.clicked.connect(self.toggle_esp32)
        form.addRow(self.esp_btn)
        return box

    def _build_mini45_group(self) -> QGroupBox:
        box = QGroupBox("Mini45 / NETBA 力传感器")
        form = QFormLayout(box)
        self.mini_mode = QComboBox()
        self.mini_mode.addItem("模拟器", "simulator")
        self.mini_mode.addItem("NETBA / Net F/T", "netft")
        form.addRow("模式", self.mini_mode)
        self.mini_ip = QLineEdit("192.168.1.1")
        form.addRow("IP 地址", self.mini_ip)
        self.mini_port = QSpinBox()
        self.mini_port.setRange(1, 65535)
        self.mini_port.setValue(49152)
        form.addRow("UDP 端口", self.mini_port)
        self.force_scale = QDoubleSpinBox()
        self.force_scale.setRange(1.0, 1_000_000_000.0)
        self.force_scale.setDecimals(0)
        self.force_scale.setValue(1_000_000.0)
        form.addRow("力计数/单位", self.force_scale)
        self.torque_scale = QDoubleSpinBox()
        self.torque_scale.setRange(1.0, 1_000_000_000.0)
        self.torque_scale.setDecimals(0)
        self.torque_scale.setValue(1_000_000.0)
        form.addRow("力矩计数/单位", self.torque_scale)
        btns = QHBoxLayout()
        self.mini_btn = QPushButton("连接 Mini45")
        self.mini_btn.clicked.connect(self.toggle_mini45)
        self.bias_btn = QPushButton("清零/偏置")
        self.bias_btn.clicked.connect(self.bias_mini45)
        self.read_scale_btn = QPushButton("读取系数")
        self.read_scale_btn.clicked.connect(self.read_mini45_scales)
        btns.addWidget(self.mini_btn)
        btns.addWidget(self.bias_btn)
        btns.addWidget(self.read_scale_btn)
        form.addRow(btns)
        self.mini_status = QLabel("Mini45 状态：未连接")
        form.addRow(self.mini_status)
        return box

    def _build_motion_group(self) -> QGroupBox:
        box = QGroupBox("Arduino 三轴电机")
        form = QFormLayout(box)
        form.addRow(
            QLabel(
                f"丝杆导程 {SCREW_LEAD_MM:g} mm/rev，{PULSES_PER_REV:g} pulse/rev，"
                f"分辨率 {MM_PER_PULSE:.3f} mm/pulse"
            )
        )

        port_row = QHBoxLayout()
        self.motion_port = QComboBox()
        self.motion_baud = QComboBox()
        self.motion_baud.addItems(["115200", "230400"])
        port_row.addWidget(self.motion_port, stretch=1)
        port_row.addWidget(self.motion_baud)
        form.addRow("串口/波特率", port_row)

        btn_row = QHBoxLayout()
        self.motion_btn = QPushButton("连接 Arduino")
        self.motion_btn.clicked.connect(self.toggle_motion)
        self.motion_pc_btn = QPushButton("上位机模式")
        self.motion_pc_btn.clicked.connect(lambda: self.motion_set_mode("PC"))
        self.motion_manual_btn = QPushButton("摇杆模式")
        self.motion_manual_btn.clicked.connect(lambda: self.motion_set_mode("MANUAL"))
        btn_row.addWidget(self.motion_btn)
        btn_row.addWidget(self.motion_pc_btn)
        btn_row.addWidget(self.motion_manual_btn)
        form.addRow(btn_row)

        home_row = QHBoxLayout()
        for axis in ("X", "Y", "Z", "ALL"):
            btn = QPushButton(f"回零 {axis}")
            btn.clicked.connect(lambda _checked=False, a=axis: self.motion_home(a))
            home_row.addWidget(btn)
        form.addRow(home_row)

        step_row = QHBoxLayout()
        self.motion_force_axis = QComboBox()
        self.motion_force_axis.addItems(["Fx", "Fy", "Fz"])
        self.motion_force_axis.setCurrentText("Fz")
        self.motion_step_mm = self._spin(MM_PER_PULSE, 5.0, MANUAL_DEFAULT_STEP_MM)
        self.motion_step_mm.setSingleStep(MM_PER_PULSE)
        self.motion_speed_mm_s = self._spin(0.001, 20.0, MANUAL_DEFAULT_SPEED_MM_S)
        self.motion_speed_mm_s.setSingleStep(0.1)
        step_row.addWidget(QLabel("力轴"))
        step_row.addWidget(self.motion_force_axis)
        step_row.addWidget(QLabel("步长mm"))
        step_row.addWidget(self.motion_step_mm)
        step_row.addWidget(QLabel("速度mm/s"))
        step_row.addWidget(self.motion_speed_mm_s)
        form.addRow(step_row)

        move_row = QHBoxLayout()
        self.motion_plus_btn = QPushButton("力轴正向小步")
        self.motion_plus_btn.clicked.connect(lambda: self.motion_force_step(1))
        self.motion_minus_btn = QPushButton("力轴负向小步")
        self.motion_minus_btn.clicked.connect(lambda: self.motion_force_step(-1))
        move_row.addWidget(self.motion_plus_btn)
        move_row.addWidget(self.motion_minus_btn)
        form.addRow(move_row)

        self.motion_status = QLabel("电机状态：未连接")
        form.addRow(self.motion_status)
        self.refresh_ports()
        return box

    def _build_force_frame_group(self) -> QGroupBox:
        box = QGroupBox("传感器坐标映射")
        grid = QGridLayout(box)

        self.frame_sign_combos: dict[str, QComboBox] = {}
        self.frame_axis_combos: dict[str, QComboBox] = {}
        defaults = {"Fx": "Fx", "Fy": "Fy", "Fz": "Fz"}
        for row, sensor_axis in enumerate(("Fx", "Fy", "Fz")):
            sign_combo = QComboBox()
            sign_combo.addItem("+", 1)
            sign_combo.addItem("-", -1)
            axis_combo = QComboBox()
            for mini_axis in ("Fx", "Fy", "Fz"):
                axis_combo.addItem(f"Mini45 {mini_axis}", mini_axis)
            self._set_combo_by_data(axis_combo, defaults[sensor_axis])
            sign_combo.currentIndexChanged.connect(self.on_force_frame_mapping_changed)
            axis_combo.currentIndexChanged.connect(self.on_force_frame_mapping_changed)
            self.frame_sign_combos[sensor_axis] = sign_combo
            self.frame_axis_combos[sensor_axis] = axis_combo
            grid.addWidget(QLabel(f"传感器 {sensor_axis} ="), row, 0)
            grid.addWidget(sign_combo, row, 1)
            grid.addWidget(axis_combo, row, 2)

        self.force_frame_status = QLabel("当前映射：传感器坐标 = Mini45 原始坐标")
        grid.addWidget(self.force_frame_status, 3, 0, 1, 3)
        return box

    def _build_force_control_group(self) -> QGroupBox:
        box = QGroupBox("力控参数")
        grid = QGridLayout(box)
        self.auto_interval_s = self._spin(0.05, 5.0, AUTO_DEFAULT_INTERVAL_S)
        self.auto_interval_s.setSingleStep(0.05)
        self.auto_step_mm = self._spin(MM_PER_PULSE, 2.0, AUTO_DEFAULT_MAX_STEP_MM)
        self.auto_step_mm.setSingleStep(MM_PER_PULSE)
        self.auto_speed_mm_s = self._spin(0.001, 20.0, AUTO_DEFAULT_SPEED_MM_S)
        self.auto_speed_mm_s.setSingleStep(0.1)
        self.k_delta_x = self._spin(MM_PER_PULSE, 0.5, 0.05)
        self.k_delta_y = self._spin(MM_PER_PULSE, 0.5, 0.05)
        self.k_delta_z = self._spin(MM_PER_PULSE, 0.5, 0.05)
        for spin in (self.k_delta_x, self.k_delta_y, self.k_delta_z):
            spin.setSingleStep(MM_PER_PULSE)
        self.k_wait_s = self._spin(0.1, 5.0, 1.0)
        self.k_sample_s = self._spin(0.1, 5.0, 1.0)
        self.k_condition_limit = self._spin(10.0, 1000.0, 300.0)
        self.force_filter_enabled = QCheckBox("启用")
        self.force_filter_enabled.setChecked(True)
        self.force_filter_cutoff_hz = self._spin(0.1, 30.0, 3.0)
        self.force_filter_cutoff_hz.setSingleStep(0.5)
        self.force_filter_median_points = QSpinBox()
        self.force_filter_median_points.setRange(1, 9)
        self.force_filter_median_points.setSingleStep(2)
        self.force_filter_median_points.setValue(5)
        self.force_filter_reset_btn = QPushButton("重置滤波")
        self.force_filter_reset_btn.clicked.connect(self.reset_force_filter)
        self.control_style = QComboBox()
        self.control_style.addItem("保守", "conservative")
        self.control_style.addItem("标准", "standard")
        self.control_style.addItem("快速", "fast")

        grid.addWidget(QLabel("Mini45上位机滤波"), 0, 0)
        grid.addWidget(self.force_filter_enabled, 0, 1)
        grid.addWidget(QLabel("截止Hz"), 0, 2)
        grid.addWidget(self.force_filter_cutoff_hz, 0, 3)
        grid.addWidget(QLabel("中值点数"), 1, 0)
        grid.addWidget(self.force_filter_median_points, 1, 1)
        grid.addWidget(self.force_filter_reset_btn, 1, 2, 1, 2)

        grid.addWidget(QLabel("δX/δY/δZ mm"), 2, 0)
        delta_row = QHBoxLayout()
        delta_row.addWidget(self.k_delta_x)
        delta_row.addWidget(self.k_delta_y)
        delta_row.addWidget(self.k_delta_z)
        grid.addLayout(delta_row, 2, 1, 1, 3)
        grid.addWidget(QLabel("等待/采样 s"), 3, 0)
        wait_row = QHBoxLayout()
        wait_row.addWidget(self.k_wait_s)
        wait_row.addWidget(self.k_sample_s)
        grid.addLayout(wait_row, 3, 1)
        grid.addWidget(QLabel("条件数上限"), 3, 2)
        grid.addWidget(self.k_condition_limit, 3, 3)
        grid.addWidget(QLabel("最大单步 mm"), 4, 0)
        grid.addWidget(self.auto_step_mm, 4, 1)
        grid.addWidget(QLabel("控制间隔 s"), 4, 2)
        grid.addWidget(self.auto_interval_s, 4, 3)
        grid.addWidget(QLabel("速度 mm/s"), 5, 0)
        grid.addWidget(self.auto_speed_mm_s, 5, 1)
        grid.addWidget(QLabel("控制风格"), 5, 2)
        grid.addWidget(self.control_style, 5, 3)

        k_buttons = QHBoxLayout()
        self.k_ident_btn = QPushButton("自动辨识 K")
        self.k_ident_btn.clicked.connect(self.start_k_identification)
        self.k_clear_btn = QPushButton("清除 K")
        self.k_clear_btn.clicked.connect(self.clear_force_control_k)
        k_buttons.addWidget(self.k_ident_btn)
        k_buttons.addWidget(self.k_clear_btn)
        grid.addLayout(k_buttons, 6, 0, 1, 4)

        self.k_status = QLabel("K 状态：未辨识")
        grid.addWidget(self.k_status, 7, 0, 1, 4)
        return box

    def _build_calibration_group(self) -> QGroupBox:
        box = QGroupBox("标定控制")
        layout = QVBoxLayout(box)

        self.basic_group = QGroupBox("基础信息")
        grid = QGridLayout(self.basic_group)
        self.experiment_id = QLineEdit("sensor01_mount01")
        self.note = QLineEdit()
        self.experiment_mode = QComboBox()
        self.experiment_mode.addItem("空载零点漂移", "zero")
        self.experiment_mode.addItem("单目标点标定", "single")
        self.experiment_mode.addItem("静态正反程标定", "sequence")
        self.experiment_mode.addItem("训练数据采集", "combined")
        self.load_axis = QComboBox()
        self.load_axis.addItems(["Fx", "Fy", "Fz"])
        self.load_axis.setCurrentText("Fz")
        self.branch = QComboBox()
        self.branch.addItem("加载", "loading")
        self.branch.addItem("卸载", "unloading")
        self.direction = QComboBox()
        self.direction.addItem("无", "none")
        self.direction.addItem("正向", "positive")
        self.direction.addItem("负向", "negative")
        grid.addWidget(QLabel("实验批次/安装编号"), 0, 0)
        grid.addWidget(self.experiment_id, 0, 1)
        grid.addWidget(QLabel("实验模式"), 0, 2)
        grid.addWidget(self.experiment_mode, 0, 3)
        self.load_axis_label = QLabel("加载轴")
        self.branch_label = QLabel("分支")
        self.direction_label = QLabel("方向")
        grid.addWidget(self.load_axis_label, 1, 0)
        grid.addWidget(self.load_axis, 1, 1)
        grid.addWidget(self.branch_label, 1, 2)
        grid.addWidget(self.branch, 1, 3)
        grid.addWidget(self.direction_label, 2, 0)
        grid.addWidget(self.direction, 2, 1)
        grid.addWidget(QLabel("备注"), 2, 2)
        grid.addWidget(self.note, 2, 3)
        layout.addWidget(self.basic_group)

        self.zero_group = QGroupBox("空载零点漂移参数")
        form = QFormLayout(self.zero_group)
        self.zero_duration_s = self._spin(1.0, 3600.0, 180.0)
        self.zero_duration_s.setSingleStep(10.0)
        form.addRow("零点采集时间 s", self.zero_duration_s)
        layout.addWidget(self.zero_group)

        self.target_group = QGroupBox("目标力与稳定判定")
        grid = QGridLayout(self.target_group)
        self.target_fx = self._spin(-20, 20, 0)
        self.target_fy = self._spin(-20, 20, 0)
        self.target_fz = self._spin(-20, 20, 0)
        self.tol_fx = self._spin(0, 5, 0.10)
        self.tol_fy = self._spin(0, 5, 0.10)
        self.tol_fz = self._spin(0, 5, 0.10)
        for row, name, target, tolerance in (
            (0, "Fx", self.target_fx, self.tol_fx),
            (1, "Fy", self.target_fy, self.tol_fy),
            (2, "Fz", self.target_fz, self.tol_fz),
        ):
            grid.addWidget(QLabel(f"目标 {name}"), row, 0)
            grid.addWidget(target, row, 1)
            grid.addWidget(QLabel(f"容差 {name}"), row, 2)
            grid.addWidget(tolerance, row, 3)
        self.stable_window = self._spin(0.5, 20, 2.0)
        self.hold_window = self._spin(0.5, 30, 5.0)
        grid.addWidget(QLabel("稳定时间 s"), 3, 0)
        grid.addWidget(self.stable_window, 3, 1)
        grid.addWidget(QLabel("保持时间 s"), 3, 2)
        grid.addWidget(self.hold_window, 3, 3)
        layout.addWidget(self.target_group)

        self.sequence_group = QGroupBox("静态正反程标定参数")
        grid = QGridLayout(self.sequence_group)
        self.seq_fz_max = self._spin(0.0, 10.0, 9.0)
        self.seq_fz_step = self._spin(0.1, 10.0, 1.0)
        self.seq_shear_max = self._spin(0.0, 4.0, 3.6)
        self.seq_shear_step = self._spin(0.1, 4.0, 0.6)
        self.seq_cycles = QSpinBox()
        self.seq_cycles.setRange(1, 20)
        self.seq_cycles.setValue(3)
        self.seq_shear_direction = QComboBox()
        self.seq_shear_direction.addItem("正负都做", "both")
        self.seq_shear_direction.addItem("正向", "positive")
        self.seq_shear_direction.addItem("负向", "negative")
        self.seq_fz_label = QLabel("法向最大/步长")
        self.seq_shear_label = QLabel("剪切最大/步长")
        grid.addWidget(self.seq_fz_label, 0, 0)
        row = QHBoxLayout()
        row.addWidget(self.seq_fz_max)
        row.addWidget(self.seq_fz_step)
        self.seq_fz_layout = row
        grid.addLayout(row, 0, 1)
        grid.addWidget(self.seq_shear_label, 0, 2)
        row = QHBoxLayout()
        row.addWidget(self.seq_shear_max)
        row.addWidget(self.seq_shear_step)
        self.seq_shear_layout = row
        grid.addLayout(row, 0, 3)
        grid.addWidget(QLabel("循环次数"), 1, 0)
        grid.addWidget(self.seq_cycles, 1, 1)
        self.seq_shear_direction_label = QLabel("剪切方向")
        grid.addWidget(self.seq_shear_direction_label, 1, 2)
        grid.addWidget(self.seq_shear_direction, 1, 3)
        layout.addWidget(self.sequence_group)

        self.combined_group = QGroupBox("训练数据采集")
        form = QFormLayout(self.combined_group)
        self.training_fz_levels = QLineEdit("3,5,7,9")
        self.training_trajectory_type = QComboBox()
        self.training_trajectory_type.addItem("Fx往返", "fx_roundtrip")
        self.training_trajectory_type.addItem("Fy往返", "fy_roundtrip")
        self.training_trajectory_type.addItem("斜向往返", "diagonal_roundtrip")
        self.training_trajectory_type.addItem("随机小幅扰动", "random_perturb")
        self.training_trajectory_type.currentIndexChanged.connect(self.update_calibration_mode_ui)
        self.training_shear_max = self._spin(0.0, 4.0, 3.6)
        self.training_target_step = self._spin(0.02, 2.0, 0.2)
        self.training_arrival_window = self._spin(0.01, 1.0, 0.15)
        self.training_max_wait_s = self._spin(1.0, 300.0, 60.0)
        self.training_random_points = QSpinBox()
        self.training_random_points.setRange(1, 500)
        self.training_random_points.setValue(30)
        form.addRow("Fz 层级 N", self.training_fz_levels)
        form.addRow("轨迹类型", self.training_trajectory_type)
        form.addRow("剪切最大力 N", self.training_shear_max)
        form.addRow("目标步距 N", self.training_target_step)
        form.addRow("训练到达窗口 N", self.training_arrival_window)
        form.addRow("最大等待时间 s", self.training_max_wait_s)
        self.training_random_points_label = QLabel("随机点数")
        form.addRow(self.training_random_points_label, self.training_random_points)
        layout.addWidget(self.combined_group)

        buttons = QHBoxLayout()
        self.cal_start_btn = QPushButton("开始标定")
        self.cal_start_btn.clicked.connect(self.start_calibration)
        self.cal_pause_btn = QPushButton("暂停")
        self.cal_pause_btn.clicked.connect(self.pause_calibration)
        self.cal_resume_btn = QPushButton("继续")
        self.cal_resume_btn.clicked.connect(self.resume_calibration)
        self.cal_skip_btn = QPushButton("跳过当前点")
        self.cal_skip_btn.clicked.connect(self.skip_calibration_point)
        self.cal_stop_btn = QPushButton("停止/急停")
        self.cal_stop_btn.clicked.connect(lambda: self.stop_calibration("人工停止"))
        for button in (self.cal_start_btn, self.cal_pause_btn, self.cal_resume_btn, self.cal_skip_btn, self.cal_stop_btn):
            buttons.addWidget(button)
        layout.addLayout(buttons)
        self.cal_status = QLabel("标定状态：空闲")
        layout.addWidget(self.cal_status)
        self.experiment_mode.currentIndexChanged.connect(self.update_calibration_mode_ui)
        self.load_axis.currentIndexChanged.connect(self.update_calibration_mode_ui)
        self.update_calibration_mode_ui()
        return box

    def _build_status_group(self) -> QGroupBox:
        box = QGroupBox("实时状态")
        grid = QGridLayout(box)
        self.value_labels = {}
        names = ["Fx", "Fy", "Fz", "Mx", "My", "Mz", "C0", "C1", "C2", "C3", "C4"]
        for idx, name in enumerate(names):
            grid.addWidget(QLabel(name), idx // 4, (idx % 4) * 2)
            label = QLabel("--")
            self.value_labels[name] = label
            grid.addWidget(label, idx // 4, (idx % 4) * 2 + 1)
        self.window_label = QLabel("目标窗口：--")
        self.stable_label = QLabel("稳定状态：--")
        self.safe_label = QLabel("安全状态：--")
        grid.addWidget(self.window_label, 3, 0, 1, 2)
        grid.addWidget(self.stable_label, 3, 2, 1, 2)
        grid.addWidget(self.safe_label, 3, 4, 1, 2)
        return box

    def _build_record_group(self) -> QGroupBox:
        box = QGroupBox("记录与导出")
        form = QFormLayout(box)
        out_row = QHBoxLayout()
        self.output_dir = QLineEdit(str(Path.cwd() / "runs"))
        browse = QPushButton("浏览")
        browse.clicked.connect(self.choose_output_dir)
        out_row.addWidget(self.output_dir)
        out_row.addWidget(browse)
        form.addRow("输出目录", out_row)
        btns = QHBoxLayout()
        self.record_btn = QPushButton("开始实验批次")
        self.record_btn.clicked.connect(self.toggle_recording)
        self.marker_btn = QPushButton("添加标记/标定点")
        self.marker_btn.clicked.connect(self.add_marker)
        btns.addWidget(self.record_btn)
        btns.addWidget(self.marker_btn)
        form.addRow(btns)
        self.record_status = QLabel("未开始实验批次")
        form.addRow(self.record_status)
        return box

    def _spin(self, minimum: float, maximum: float, value: float) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(3)
        spin.setValue(value)
        spin.setSingleStep(0.1)
        return spin

    def update_calibration_mode_ui(self) -> None:
        mode = self._combo_value(self.experiment_mode)
        axis = self.load_axis.currentText()
        is_zero = mode == "zero"
        is_single = mode == "single"
        is_sequence = mode == "sequence"
        is_combined = mode == "combined"
        for widget in (self.load_axis_label, self.load_axis):
            widget.setVisible(is_single or is_sequence)
        for widget in (self.branch_label, self.branch, self.direction_label, self.direction):
            widget.setVisible(is_single)

        self.zero_group.setVisible(is_zero)
        self.target_group.setVisible(is_single or is_sequence)
        self.sequence_group.setVisible(is_sequence)
        self.combined_group.setVisible(is_combined)
        random_training = is_combined and self._combo_value(self.training_trajectory_type) == "random_perturb"
        self.training_random_points_label.setVisible(random_training)
        self.training_random_points.setVisible(random_training)

        shear_axis = axis in {"Fx", "Fy"}
        for widget in (self.seq_fz_label, self.seq_fz_max, self.seq_fz_step):
            widget.setVisible(is_sequence and axis == "Fz")
        for widget in (self.seq_shear_label, self.seq_shear_max, self.seq_shear_step, self.seq_shear_direction_label, self.seq_shear_direction):
            widget.setVisible(is_sequence and shear_axis)
        self.cal_pause_btn.setVisible(is_single or is_sequence or is_combined)
        self.cal_resume_btn.setVisible(is_single or is_sequence or is_combined)
        self.cal_skip_btn.setVisible(is_sequence)

    def refresh_ports(self) -> None:
        current_esp = self.esp_port.currentText() if hasattr(self, "esp_port") else ""
        current_motion = self.motion_port.currentText() if hasattr(self, "motion_port") else ""
        try:
            ports = [port.device for port in serial.tools.list_ports.comports()]
        except Exception:
            ports = []
        if hasattr(self, "esp_port"):
            self.esp_port.clear()
            self.esp_port.addItems(ports)
            if current_esp in ports:
                self.esp_port.setCurrentText(current_esp)
        if hasattr(self, "motion_port"):
            self.motion_port.clear()
            self.motion_port.addItems(ports)
            if current_motion in ports:
                self.motion_port.setCurrentText(current_motion)

    def choose_output_dir(self) -> None:
        path = QFileDialog.getExistingDirectory(self, "选择输出目录", self.output_dir.text())
        if path:
            self.output_dir.setText(path)

    def toggle_esp32(self) -> None:
        if self.esp32:
            self.esp32.stop()
            self.esp32 = None
            self.esp_btn.setText("连接 ESP32")
            self._log("ESP32 已断开")
            return
        port = self.esp_port.currentText()
        if not port:
            QMessageBox.warning(self, "ESP32", "未选择串口")
            return
        try:
            self.esp32 = Esp32SerialAdapter(
                port=port,
                baud=int(self.esp_baud.currentText()),
                mode=self._combo_value(self.esp_mode),
                rate_hz=self.esp_rate.value(),
            )
            self.esp32.start()
            self.esp_btn.setText("断开 ESP32")
            self._log(f"ESP32 已连接：{port}")
        except Exception as exc:
            self.esp32 = None
            QMessageBox.critical(self, "ESP32", str(exc))

    def toggle_mini45(self) -> None:
        if self.mini45:
            self.mini45.stop()
            self.mini45 = None
            self.last_force_time = 0.0
            self.latest_force_sample = None
            self.reset_force_filter(log=False)
            self.mini_btn.setText("连接 Mini45")
            self.mini_status.setText("Mini45 状态：未连接")
            self._log("Mini45 已断开")
            return
        try:
            mini_mode = self._combo_value(self.mini_mode)
            if mini_mode == "simulator":
                self.mini45 = Mini45Simulator(rate_hz=100)
            else:
                self.mini45 = Mini45NetFTAdapter(
                    ip=self.mini_ip.text().strip(),
                    port=self.mini_port.value(),
                    force_counts_per_unit=self.force_scale.value(),
                    torque_counts_per_unit=self.torque_scale.value(),
            )
            self.last_force_time = 0.0
            self.latest_force_sample = None
            self.reset_force_filter(log=False)
            self.mini45.start()
            self.mini_btn.setText("断开 Mini45")
            if mini_mode == "simulator":
                self.mini_status.setText("Mini45 状态：模拟器已启动")
                self._log("Mini45 模拟器已启动")
            else:
                self.mini_status.setText("Mini45 状态：已发送 RDT 启动命令，等待第一帧数据")
                self._log("Mini45 已发送 RDT 启动命令，等待第一帧数据")
        except Exception as exc:
            self.mini45 = None
            QMessageBox.critical(self, "Mini45", str(exc))

    def read_mini45_scales(self) -> None:
        ip = self.mini_ip.text().strip()
        if not ip:
            QMessageBox.warning(self, "Mini45", "请先填写 NETBA IP 地址")
            return
        try:
            config = fetch_netft_config(ip)
            if "cfgcpf" in config:
                self.force_scale.setValue(float(config["cfgcpf"]))
            if "cfgcpt" in config:
                self.torque_scale.setValue(float(config["cfgcpt"]))
            force_unit = config.get("scfgfu", "")
            torque_unit = config.get("scfgtu", "")
            rdt_rate = config.get("comrdtrate", "")
            rdt_enabled = config.get("comrdte", "")
            details = []
            if force_unit:
                details.append(f"力单位 {force_unit}")
            if torque_unit:
                details.append(f"力矩单位 {torque_unit}")
            if rdt_rate:
                details.append(f"RDT 频率 {rdt_rate} Hz")
            if rdt_enabled:
                details.append(f"RDT 启用状态 {rdt_enabled}")
            self._log(f"已从 netftapi2.xml 读取系数：cfgcpf={config.get('cfgcpf', '未知')}，cfgcpt={config.get('cfgcpt', '未知')}；{'，'.join(details)}")
        except Exception as exc:
            QMessageBox.warning(self, "Mini45", f"读取 NETBA 系数失败：{exc}")

    def bias_mini45(self) -> None:
        if self.auto_force_active or self.k_ident_active or self.zero_drift_active or self.training_active:
            QMessageBox.warning(self, "Mini45", "自动标定、K 辨识、零点漂移或训练采集过程中不能清零/偏置")
            return
        if self.mini45 and hasattr(self.mini45, "bias"):
            self.mini45.bias()
            self.reset_force_filter(log=False)
            self.buffer.clear()
            self._clear_force_plot()
            self._log("Mini45 清零/偏置命令已发送，已重置上位机滤波、稳定窗口和力曲线")

    def _force_filter_settings(self) -> ForceFilterSettings:
        return ForceFilterSettings(
            enabled=self.force_filter_enabled.isChecked(),
            cutoff_hz=self.force_filter_cutoff_hz.value(),
            median_window=self.force_filter_median_points.value(),
        )

    def reset_force_filter(self, log: bool = True) -> None:
        self.force_filter.reset()
        if log:
            self._log("Mini45 上位机滤波状态已重置")

    def _clear_force_plot(self) -> None:
        self.force_x.clear()
        for values in self.force_y.values():
            values.clear()
        for curve in self.force_curves.values():
            curve.setData([], [])

    def toggle_motion(self) -> None:
        if self.motion:
            self.stop_auto_force("Arduino 已断开")
            self.abort_k_identification("Arduino 已断开")
            self.motion.stop()
            self.motion = None
            self.motion_btn.setText("连接 Arduino")
            self.motion_status.setText("电机状态：未连接")
            self._log("Arduino 电机控制已断开")
            return
        port = self.motion_port.currentText()
        if not port:
            QMessageBox.warning(self, "Arduino 电机", "未选择串口")
            return
        try:
            self.motion = ArduinoMotionAdapter(port=port, baud=int(self.motion_baud.currentText()))
            self.motion.start()
            self.motion_btn.setText("断开 Arduino")
            self.motion_status.setText("电机状态：已连接，默认仍为摇杆模式")
            self._log(f"Arduino 电机控制已连接：{port}")
        except Exception as exc:
            self.motion = None
            QMessageBox.critical(self, "Arduino 电机", str(exc))

    def motion_set_mode(self, mode: str) -> None:
        if not self.motion:
            QMessageBox.warning(self, "Arduino 电机", "请先连接 Arduino")
            return
        try:
            self.motion.set_mode(mode)
            text = "上位机模式" if mode == "PC" else "摇杆模式"
            self.motion_status.setText(f"电机状态：已切换到{text}")
            self._log(f"Arduino 已请求切换到{text}")
        except Exception as exc:
            QMessageBox.warning(self, "Arduino 电机", str(exc))

    def motion_enable(self, enabled: bool) -> None:
        if not self.motion:
            QMessageBox.warning(self, "Arduino 电机", "请先连接 Arduino")
            return
        try:
            self.motion.enable(enabled)
            self._log("Arduino 电机已请求使能" if enabled else "Arduino 电机已请求失能")
        except Exception as exc:
            QMessageBox.warning(self, "Arduino 电机", str(exc))

    def motion_stop(self) -> None:
        self.stop_auto_force("急停")
        self.abort_k_identification("急停")
        if not self.motion:
            return
        try:
            self.motion.stop_all()
            self._log("Arduino 电机急停命令已发送")
        except Exception as exc:
            QMessageBox.warning(self, "Arduino 电机", str(exc))

    def motion_home(self, axis: str) -> None:
        if not self.motion:
            QMessageBox.warning(self, "Arduino 电机", "请先连接 Arduino")
            return
        try:
            self.stop_auto_force("回零")
            self.abort_k_identification("回零")
            self.motion.home(axis)
            self._log(f"Arduino 回零命令已发送：{axis}")
        except Exception as exc:
            QMessageBox.warning(self, "Arduino 电机", str(exc))

    def motion_force_step(self, direction: int) -> None:
        if not self.motion:
            QMessageBox.warning(self, "Arduino 电机", "请先连接 Arduino")
            return
        force_axis = self._combo_value(self.motion_force_axis)
        motor_axis, delta_mm = mapped_motor_delta(
            force_axis=force_axis,
            force_error=float(direction),
            step_mm=self.motion_step_mm.value(),
            mapping=self.motion_mapping(),
            signs=self.motion_signs(),
            min_pulses=1,
        )
        try:
            self.motion.set_mode("PC")
            self.motion.enable(True)
            self.motion.move_mm(motor_axis, delta_mm, self.motion_speed_mm_s.value())
            self._log(
                f"{force_axis} 小步移动：电机 {motor_axis} {delta_mm:+.4f} mm，"
                f"{mm_to_pulses(delta_mm):+d} pulse"
            )
        except Exception as exc:
            QMessageBox.warning(self, "Arduino 电机", str(exc))

    def motion_mapping(self) -> dict[str, str]:
        return dict(DEFAULT_FORCE_TO_MOTOR)

    def motion_signs(self) -> dict[str, int]:
        return dict(DEFAULT_FORCE_TO_MOTOR_SIGN)

    def current_force_frame_mapping(self) -> ForceFrameMapping:
        return ForceFrameMapping(
            sensor_fx=AxisFrameMap(
                self._combo_value(self.frame_axis_combos["Fx"]),
                int(self.frame_sign_combos["Fx"].currentData()),
            ),
            sensor_fy=AxisFrameMap(
                self._combo_value(self.frame_axis_combos["Fy"]),
                int(self.frame_sign_combos["Fy"].currentData()),
            ),
            sensor_fz=AxisFrameMap(
                self._combo_value(self.frame_axis_combos["Fz"]),
                int(self.frame_sign_combos["Fz"].currentData()),
            ),
        )

    def on_force_frame_mapping_changed(self) -> None:
        if not hasattr(self, "force_frame_status"):
            return
        try:
            mapping = self.current_force_frame_mapping()
            mapping.validate()
        except ValueError as exc:
            self.force_frame_status.setText(f"坐标映射无效：{exc}")
            self.force_frame_status.setStyleSheet("color: red")
            return
        self.force_mapping_error_logged = False
        self.force_frame_status.setStyleSheet("")
        self.force_frame_status.setText(
            "当前映射："
            f"Fx={mapping.sensor_fx.sign:+d} Mini45 {mapping.sensor_fx.source_axis}，"
            f"Fy={mapping.sensor_fy.sign:+d} Mini45 {mapping.sensor_fy.source_axis}，"
            f"Fz={mapping.sensor_fz.sign:+d} Mini45 {mapping.sensor_fz.source_axis}"
        )
        if self.force_control_result:
            self.clear_force_control_k()
            self._log("坐标映射已修改，当前 K 已清除，需要重新自动辨识")

    def _set_force_frame_mapping_enabled(self, enabled: bool) -> None:
        for combo in list(self.frame_sign_combos.values()) + list(self.frame_axis_combos.values()):
            combo.setEnabled(enabled)

    def _update_force_frame_mapping_lock(self) -> None:
        locked = bool(self.recorder or self.k_ident_active or self.force_control_result)
        self._set_force_frame_mapping_enabled(not locked)

    def k_delta_values(self) -> dict[str, float]:
        return {
            "X": self.k_delta_x.value(),
            "Y": self.k_delta_y.value(),
            "Z": self.k_delta_z.value(),
        }

    def start_k_identification(self) -> None:
        if not self.motion:
            QMessageBox.warning(self, "K 辨识", "请先连接 Arduino 电机控制串口")
            return
        if not self.mini45:
            QMessageBox.warning(self, "K 辨识", "请先连接 Mini45 并确认有实时力数据")
            return
        try:
            self.current_force_frame_mapping().validate()
        except ValueError as exc:
            QMessageBox.warning(self, "K 辨识", f"请先修正传感器坐标映射：{exc}")
            return
        if not self.latest_force_sample or time.monotonic() - self.last_force_time > 1.0:
            QMessageBox.warning(self, "K 辨识", "Mini45 暂无实时力数据")
            return
        self.stop_auto_force("开始 K 辨识")
        self.k_ident_active = True
        self.k_ident_axis_index = 0
        self.k_ident_phase = "before"
        self.k_ident_phase_start_s = time.monotonic()
        self.k_ident_wait_until_s = 0.0
        self.k_ident_before_means = {}
        self.k_ident_after_means = {}
        self.k_ident_before_stds = {}
        self.k_ident_after_stds = {}
        self.force_control_result = None
        self.force_control_state = DecoupledControlState()
        self._update_force_frame_mapping_lock()
        try:
            self.motion.set_mode("PC")
            self.motion.enable(True)
        except Exception as exc:
            self.abort_k_identification(str(exc))
            return
        self.k_status.setText("K 状态：正在辨识 X 轴扰动前均值")
        self.cal_status.setText("标定状态：K 自动辨识中")
        self._log("开始自动辨识 K：列顺序固定为 Arduino X/Y/Z，行顺序为传感器坐标 Fx/Fy/Fz")

    def clear_force_control_k(self) -> None:
        self.force_control_result = None
        self.force_control_state = DecoupledControlState()
        self.k_status.setText("K 状态：未辨识")
        self._update_force_frame_mapping_lock()
        self._log("已清除当前 K")

    def abort_k_identification(self, reason: str) -> None:
        if not self.k_ident_active:
            return
        self.k_ident_active = False
        try:
            if self.motion:
                self.motion.stop_all()
        except Exception:
            pass
        self.k_status.setText(f"K 状态：辨识失败，{reason}")
        self.cal_status.setText(f"标定状态：K 辨识失败，{reason}")
        self._log(f"K 辨识失败：{reason}")
        self._update_force_frame_mapping_lock()

    def _force_sample_window(self, seconds: float):
        return force_stats(self.buffer.window(time.monotonic(), seconds))

    def _current_force_safe(self) -> bool:
        if not self.latest_force_sample:
            return False
        sample = self.latest_force_sample
        safety = SafetySettings()
        torque_limit = self._stability_settings().torque_abs_max
        return (
            abs(sample.fx) <= safety.fx_abs_max_n
            and abs(sample.fy) <= safety.fy_abs_max_n
            and abs(sample.fz) <= safety.fz_abs_max_n
            and abs(sample.mx) <= torque_limit
            and abs(sample.my) <= torque_limit
            and abs(sample.mz) <= torque_limit
        )

    def _update_k_identification(self) -> None:
        if not self.k_ident_active:
            return
        if not self.motion:
            self.abort_k_identification("Arduino 未连接")
            return
        if not self.latest_force_sample or time.monotonic() - self.last_force_time > 1.0:
            self.abort_k_identification("Mini45 数据超过 1 秒未更新")
            return
        if not self._current_force_safe():
            self.abort_k_identification("力值超过安全限值")
            return

        now = time.monotonic()
        axis = MOTOR_AXES[self.k_ident_axis_index]
        sample_window = self.k_sample_s.value()
        if self.k_ident_phase == "before":
            if now - self.k_ident_phase_start_s < sample_window:
                return
            stats = self._force_sample_window(sample_window)
            if stats.count < 2:
                self.abort_k_identification("扰动前 Mini45 数据不足")
                return
            self.k_ident_before_means[axis] = stats.mean
            self.k_ident_before_stds[axis] = stats.std
            delta = self.k_delta_values()[axis]
            try:
                self.motion.move_mm(axis, delta, self.auto_speed_mm_s.value())
            except Exception as exc:
                self.abort_k_identification(str(exc))
                return
            move_time = abs(delta) / max(self.auto_speed_mm_s.value(), 1e-6)
            self.k_ident_phase = "after_wait"
            self.k_ident_wait_until_s = now + move_time + self.k_wait_s.value() + 0.05
            self.k_status.setText(f"K 状态：{axis} 轴扰动 {delta:+.4f} mm，等待稳定")
            return

        if self.k_ident_phase == "after_wait":
            if now < self.k_ident_wait_until_s:
                return
            self.k_ident_phase = "after"
            self.k_ident_phase_start_s = now
            self.k_status.setText(f"K 状态：正在采集 {axis} 轴扰动后均值")
            return

        if self.k_ident_phase == "after":
            if now - self.k_ident_phase_start_s < sample_window:
                return
            stats = self._force_sample_window(sample_window)
            if stats.count < 2:
                self.abort_k_identification("扰动后 Mini45 数据不足")
                return
            self.k_ident_after_means[axis] = stats.mean
            self.k_ident_after_stds[axis] = stats.std
            delta = -self.k_delta_values()[axis]
            try:
                self.motion.move_mm(axis, delta, self.auto_speed_mm_s.value())
            except Exception as exc:
                self.abort_k_identification(str(exc))
                return
            move_time = abs(delta) / max(self.auto_speed_mm_s.value(), 1e-6)
            self.k_ident_phase = "back_wait"
            self.k_ident_wait_until_s = now + move_time + self.k_wait_s.value() + 0.05
            self.k_status.setText(f"K 状态：{axis} 轴回退 {delta:+.4f} mm")
            return

        if self.k_ident_phase == "back_wait":
            if now < self.k_ident_wait_until_s:
                return
            self.k_ident_axis_index += 1
            if self.k_ident_axis_index >= len(MOTOR_AXES):
                self.finish_k_identification()
                return
            next_axis = MOTOR_AXES[self.k_ident_axis_index]
            self.k_ident_phase = "before"
            self.k_ident_phase_start_s = now
            self.k_status.setText(f"K 状态：正在辨识 {next_axis} 轴扰动前均值")

    def finish_k_identification(self) -> None:
        self.k_ident_active = False
        result = identify_k_matrix(
            before_means=self.k_ident_before_means,
            after_means=self.k_ident_after_means,
            before_stds=self.k_ident_before_stds,
            after_stds=self.k_ident_after_stds,
            deltas_mm=self.k_delta_values(),
            condition_limit=self.k_condition_limit.value(),
        )
        self.force_control_result = result if result.valid else None
        self.force_control_state = DecoupledControlState()
        self.update_k_display(result)
        self.write_k_identification_result(result)
        self._update_force_frame_mapping_lock()
        if result.valid:
            self._log(f"K 辨识完成：条件数 {result.condition:.3f}")
            self.cal_status.setText("标定状态：K 辨识完成，可开始自动力控")
        else:
            self._log(f"K 辨识无效：{result.reject_reason}")
            QMessageBox.warning(self, "K 辨识", f"K 辨识无效：{result.reject_reason}")

    def update_k_display(self, result=None) -> None:
        result = result or self.force_control_result
        if not result:
            self.k_status.setText("K 状态：未辨识")
            return
        status = "有效" if result.valid else "无效"
        if result.debug:
            status = "调试矩阵"
        self.k_status.setText(f"K 状态：{status}，条件数 {result.condition:.3f}，噪声 {result.noise_norm:.4f} N")

    def write_k_identification_result(self, result=None) -> None:
        result = result or self.force_control_result
        if not result or not self.recorder:
            return
        row = {
            "experiment_id": self.experiment_id.text().strip() or "exp001",
            "valid": result.valid,
            "reject_reason": result.reject_reason,
            "debug": result.debug,
            "delta_X_mm": result.deltas_mm.get("X", ""),
            "delta_Y_mm": result.deltas_mm.get("Y", ""),
            "delta_Z_mm": result.deltas_mm.get("Z", ""),
            "wait_s": self.k_wait_s.value(),
            "sample_window_s": self.k_sample_s.value(),
            "noise_norm": result.noise_norm,
            "condition": result.condition,
        }
        for index in range(3):
            row[f"singular_{index + 1}"] = result.singular_values[index] if index < len(result.singular_values) else ""
        for force_index, force_axis in enumerate(("Fx", "Fy", "Fz")):
            for motor_index, motor_axis in enumerate(("X", "Y", "Z")):
                row[f"K_{force_axis}_{motor_axis}"] = result.k[force_index][motor_index]
        for motor_axis in ("X", "Y", "Z"):
            before = result.before_means.get(motor_axis, [float("nan")] * 3)
            after = result.after_means.get(motor_axis, [float("nan")] * 3)
            for force_index, force_axis in enumerate(("Fx", "Fy", "Fz")):
                row[f"before_{motor_axis}_{force_axis}"] = before[force_index]
                row[f"after_{motor_axis}_{force_axis}"] = after[force_index]
        self.recorder.write_force_control_k(row)

    def ensure_recording(self) -> bool:
        if self.recorder:
            return True
        QMessageBox.warning(self, "实验批次", "请先点击“开始实验批次”，再开始当前子实验")
        return False

    def start_calibration(self) -> None:
        mode = self._combo_value(self.experiment_mode)
        if not self.ensure_recording():
            return
        if mode == "combined" and not self._training_devices_ready():
            return
        self.stop_auto_force("")
        self.sequence_targets = []
        self.sequence_index = 0
        self.active_target = None
        self.calibration_paused = False
        self.calibration_mode = mode
        if mode == "zero":
            self.start_zero_drift()
            return
        if mode == "single":
            self.current_cycle_id = "cycle_001"
            self.active_target = CalibrationTarget(
                axis=self._combo_value(self.load_axis),
                direction=self._combo_value(self.direction),
                branch=self._combo_value(self.branch),
                target_fx=self.target_fx.value(),
                target_fy=self.target_fy.value(),
                target_fz=self.target_fz.value(),
            )
            self.start_auto_force()
            return
        if mode == "sequence":
            self.sequence_targets = self._build_sequence_targets()
            if not self.sequence_targets:
                QMessageBox.warning(self, "静态正反程标定", "当前参数没有生成任何标定点")
                return
            self.sequence_index = 0
            self.start_next_sequence_target()
            return
        if mode == "combined":
            self.start_training_collection()

    def _training_devices_ready(self) -> bool:
        if not self.esp32:
            QMessageBox.warning(self, "训练数据采集", "请先连接 ESP32 电容采集串口")
            return False
        if not self.mini45:
            QMessageBox.warning(self, "训练数据采集", "请先连接 Mini45")
            return False
        if not self.motion:
            QMessageBox.warning(self, "训练数据采集", "请先连接 Arduino 电机控制串口")
            return False
        return True

    def _build_sequence_targets(self) -> list[CalibrationTarget]:
        axis = self._combo_value(self.load_axis)
        if axis == "Fz":
            return generate_fz_sequence(self.seq_fz_max.value(), self.seq_fz_step.value(), self.seq_cycles.value())
        return generate_shear_sequence(
            axis=axis,
            max_force=self.seq_shear_max.value(),
            step=self.seq_shear_step.value(),
            target_fz=self.target_fz.value(),
            direction_mode=self._combo_value(self.seq_shear_direction),
            cycles=self.seq_cycles.value(),
        )

    def start_next_sequence_target(self) -> None:
        if self.sequence_index >= len(self.sequence_targets):
            self.stop_calibration("静态正反程标定完成")
            return
        self.active_target = self.sequence_targets[self.sequence_index]
        self._apply_target_to_ui(self.active_target)
        self.cal_status.setText(f"标定状态：正反程点 {self.sequence_index + 1}/{len(self.sequence_targets)}")
        self.start_auto_force()

    def _apply_target_to_ui(self, target: CalibrationTarget) -> None:
        self._set_combo_by_data(self.load_axis, target.axis)
        self._set_combo_by_data(self.branch, target.branch)
        self._set_combo_by_data(self.direction, target.direction)
        self.target_fx.setValue(target.target_fx)
        self.target_fy.setValue(target.target_fy)
        self.target_fz.setValue(target.target_fz)

    def start_training_collection(self) -> None:
        try:
            fz_levels = parse_force_levels(self.training_fz_levels.text())
            self.training_targets = generate_training_trajectory(
                fz_levels=fz_levels,
                shear_max=self.training_shear_max.value(),
                trajectory_type=self._combo_value(self.training_trajectory_type),
                target_step_n=self.training_target_step.value(),
                random_points=self.training_random_points.value(),
            )
        except Exception as exc:
            QMessageBox.warning(self, "训练数据采集", str(exc))
            return
        if not self.training_targets:
            QMessageBox.warning(self, "训练数据采集", "当前参数没有生成训练轨迹")
            return

        self.training_count += 1
        self.current_cycle_id = f"training_{self.training_count:03d}"
        self.training_active = True
        self.training_target_index = 0
        self.training_current_target = None
        if self.recorder:
            self.recorder.start_training_files()
        self._write_training_marker("training_start")
        self._enter_training_target(0)
        self.start_auto_force()
        if not self.auto_force_active:
            self.finish_training_collection("启动失败")
            return
        self._log("训练数据采集开始：仅写入 training_raw_timeseries.csv 和 training_markers.csv")

    def _enter_training_target(self, index: int) -> None:
        self.training_target_index = index
        self.training_current_target = self.training_targets[index]
        self.training_target_start_s = time.monotonic()
        self._set_training_target(self.training_current_target)
        self._write_training_marker(self._training_start_marker_note(index))

    def _set_training_target(self, target: TrainingTarget) -> None:
        self.active_target = CalibrationTarget(
            axis="combined",
            direction=target.direction,
            branch=target.branch,
            target_fx=target.target_fx,
            target_fy=target.target_fy,
            target_fz=target.target_fz,
        )

    def _training_start_marker_note(self, index: int) -> str:
        target = self.training_targets[index]
        previous = self.training_targets[index - 1] if index > 0 else None
        if target.phase == "preload" and (previous is None or previous.phase != "preload"):
            return "fz_level_start"
        if target.phase == "recovery" and (previous is None or previous.phase != "recovery"):
            return "recovery_start"
        return "target_start"

    def _update_training_collection(self) -> None:
        if not self.training_active or self.calibration_paused:
            return
        if not self.training_current_target:
            return
        now = time.monotonic()
        elapsed = now - self.training_target_start_s
        target = self.training_current_target
        if self.latest_force_sample and training_target_reached(self.latest_force_sample, target, self.training_arrival_window.value()):
            self._write_training_marker("target_reached")
            self._advance_training_target()
            return
        if training_target_timed_out(elapsed, self.training_max_wait_s.value()):
            self._write_training_marker("target_timeout")
            self._write_training_marker("target_skipped")
            self._advance_training_target()
            return
        self.cal_status.setText(
            f"训练采集：{self.training_target_index + 1}/{len(self.training_targets)}，"
            f"{target.phase}，目标 Fx={target.target_fx:.3f}, "
            f"Fy={target.target_fy:.3f}, Fz={target.target_fz:.3f}，"
            f"等待 {elapsed:.1f}/{self.training_max_wait_s.value():.1f}s"
        )

    def _advance_training_target(self) -> None:
        next_index = self.training_target_index + 1
        if next_index >= len(self.training_targets):
            self.stop_calibration("训练数据采集完成")
            return
        self._enter_training_target(next_index)

    def finish_training_collection(self, reason: str = "停止") -> None:
        if not self.training_active:
            return
        self._write_training_marker("training_end")
        if self.recorder:
            self.recorder.stop_training_files()
        self.training_active = False
        self.training_targets = []
        self.training_target_index = 0
        self.training_current_target = None
        self.training_pause_started_s = 0.0
        self._log(f"训练数据采集结束：{reason}")

    def _write_training_marker(self, note: str) -> None:
        if not self.recorder:
            return
        self.marker_id += 1
        meta = self._meta()
        meta.note = f"{meta.note}; {note}" if meta.note else note
        target = self.training_current_target
        self.recorder.write_training_marker(
            self.marker_id,
            meta,
            trajectory_type=target.trajectory_type if target else self._combo_value(self.training_trajectory_type),
            phase=target.phase if target else note,
            target_shear_n=target.target_shear_n if target else "",
            target_angle_deg=target.target_angle_deg if target else "",
        )

    def pause_calibration(self) -> None:
        self.calibration_paused = True
        if self.training_active:
            self.training_pause_started_s = time.monotonic()
        try:
            if self.motion:
                self.motion.stop_all()
        except Exception:
            pass
        self.cal_status.setText("标定状态：已暂停")

    def resume_calibration(self) -> None:
        if not self.calibration_mode:
            return
        if self.training_active and self.training_pause_started_s > 0.0:
            self.training_target_start_s += time.monotonic() - self.training_pause_started_s
            self.training_pause_started_s = 0.0
        self.calibration_paused = False
        self.cal_status.setText("标定状态：继续")

    def skip_calibration_point(self) -> None:
        if self.training_active:
            self._write_training_marker("target_skipped")
            self._advance_training_target()
            return
        if self.calibration_mode == "sequence":
            self.sequence_index += 1
            self.start_next_sequence_target()
        else:
            self.stop_calibration("已跳过当前点")

    def stop_calibration(self, reason: str = "") -> None:
        if self.training_active:
            self.finish_training_collection(reason or "停止")
        if self.zero_drift_active:
            self.finish_zero_drift(reason or "停止")
        self.stop_auto_force(reason)
        self.calibration_mode = ""
        self.calibration_paused = False
        self.sequence_targets = []
        self.sequence_index = 0
        self.active_target = None
        self.training_current_target = None
        if reason:
            self.cal_status.setText(f"标定状态：{reason}")

    def start_zero_drift(self) -> None:
        self.stop_auto_force("零点漂移采集中")
        self.zero_drift_count += 1
        self.current_cycle_id = f"zero_{self.zero_drift_count:03d}"
        self.zero_drift_active = True
        self.zero_drift_start_s = time.monotonic()
        self.zero_drift_samples = []
        path = self.recorder.start_zero_drift_timeseries() if self.recorder else None
        self.zero_drift_file = path.name if path else ""
        self._write_marker_with_note("zero_start")
        self.cal_status.setText(f"标定状态：空载零点漂移采集中，文件 {self.zero_drift_file}")
        self._log(f"空载零点漂移开始：{self.zero_drift_file}")

    def finish_zero_drift(self, reason: str = "完成") -> None:
        if not self.zero_drift_active:
            return
        self.zero_drift_active = False
        self._write_marker_with_note("zero_end")
        if self.recorder:
            self.recorder.stop_zero_drift_timeseries()
        self.cal_status.setText(f"标定状态：空载零点漂移{reason}，共 {len(self.zero_drift_samples)} 行")
        self._log(f"空载零点漂移{reason}：{len(self.zero_drift_samples)} 行")

    def _write_marker_with_note(self, note: str) -> None:
        self.marker_id += 1
        meta = self._meta()
        meta.note = f"{meta.note}; {note}" if meta.note else note
        if self.recorder:
            self.recorder.write_marker(self.marker_id, meta)

    def start_auto_force(self) -> None:
        if not self.motion:
            QMessageBox.warning(self, "自动标定", "请先连接 Arduino 电机控制串口")
            return
        if not self.mini45:
            QMessageBox.warning(self, "自动标定", "请先连接 Mini45 并确认有实时力数据")
            return
        if self.k_ident_active:
            QMessageBox.warning(self, "自动标定", "K 正在自动辨识，请等待辨识结束")
            return
        if not self.force_control_result or not self.force_control_result.valid:
            QMessageBox.warning(self, "自动标定", "请先完成有效的 K 自动辨识")
            return
        self.auto_force_active = True
        self.auto_force_holding = False
        self.auto_force_marker_done = False
        self.auto_force_in_window_since = 0.0
        self.auto_force_last_move = 0.0
        self.auto_force_next_move_time = 0.0
        self.force_control_state = DecoupledControlState()
        try:
            self.motion.set_mode("PC")
            self.motion.enable(True)
        except Exception as exc:
            self.auto_force_active = False
            QMessageBox.warning(self, "自动标定", str(exc))
            return
        meta = self._meta()
        self.motion_status.setText(f"电机状态：自动逼近 {meta.axis}")
        self.cal_status.setText(f"标定状态：自动逼近目标 Fx={meta.target_fx:.3f}, Fy={meta.target_fy:.3f}, Fz={meta.target_fz:.3f}")
        self._log(f"开始解耦自动力控，目标 Fx={meta.target_fx:.3f}, Fy={meta.target_fy:.3f}, Fz={meta.target_fz:.3f}")

    def stop_auto_force(self, reason: str = "") -> None:
        if not self.auto_force_active and not self.auto_force_holding:
            return
        self.auto_force_active = False
        self.auto_force_holding = False
        self.auto_force_marker_done = False
        self.auto_force_next_move_time = 0.0
        try:
            if self.motion:
                self.motion.stop_all()
        except Exception:
            pass
        if reason:
            self.motion_status.setText(f"电机状态：自动停止，{reason}")
            self._log(f"自动逼近停止：{reason}")

    def toggle_recording(self) -> None:
        if self.recorder:
            if self.training_active or self.zero_drift_active or self.auto_force_active:
                self.stop_calibration("实验批次结束")
            self.recorder.stop()
            self.recorder = None
            self.clear_force_control_k()
            self._update_force_frame_mapping_lock()
            self.record_btn.setText("开始实验批次")
            self.record_status.setText("未开始实验批次")
            self._log("实验批次已结束")
            return
        try:
            mapping = self.current_force_frame_mapping()
            mapping.validate()
        except ValueError as exc:
            QMessageBox.warning(self, "实验批次", f"请先修正传感器坐标映射：{exc}")
            return
        suffix = self._safe_experiment_folder_suffix(self.experiment_id.text().strip())
        folder_name = time.strftime("%Y%m%d_%H%M%S") + (f"_{suffix}" if suffix else "")
        output = Path(self.output_dir.text()) / folder_name
        self.recorder = CsvRecorder(output)
        self.recorder.start()
        self.recorder.write_force_frame_mapping(mapping.as_row("", self.experiment_id.text().strip() or "exp001"))
        self.marker_id = 0
        self.zero_drift_count = 0
        self.training_count = 0
        self.current_cycle_id = "cycle_001"
        self.buffer.clear()
        self.clear_force_control_k()
        self._update_force_frame_mapping_lock()
        self.record_btn.setText("结束实验批次")
        self.record_status.setText(str(output))
        self._log(f"实验批次已开始：{output}")

    def _safe_experiment_folder_suffix(self, text: str) -> str:
        invalid = '<>:"/\\|?*'
        cleaned = "".join("_" if char in invalid or ord(char) < 32 else char for char in text)
        cleaned = "_".join(part for part in cleaned.strip(" ._").split() if part)
        return cleaned[:80]

    def add_marker(self) -> None:
        self.marker_id += 1
        meta = self._meta()
        if self.recorder:
            self.recorder.write_marker(self.marker_id, meta)
        end = time.monotonic()
        samples = self.buffer.window(end, self.stable_window.value())
        settings = self._stability_settings()
        result = evaluate_three_axis_stability(samples, meta, settings, SafetySettings())
        point = build_calibration_point(samples, meta, self.marker_id, result.stable, result.reject_reason)
        if point and self.recorder:
            self.recorder.write_calibration_point(point)
        reason = self._display_reason(result.reject_reason) if result.reject_reason else "通过"
        self._log(f"标记 {self.marker_id}：有效={self._yes_no(result.stable)}，原因={reason}")

    def _tick(self) -> None:
        self._drain_esp32()
        self._drain_mini45()
        self._drain_motion()
        self._update_k_identification()
        if self.zero_drift_active and time.monotonic() - self.zero_drift_start_s >= self.zero_duration_s.value():
            self.finish_zero_drift("完成")
        self._update_training_collection()
        self._update_auto_force()
        self._update_status()

    def _drain_esp32(self) -> None:
        if not self.esp32:
            return
        while True:
            try:
                item = self.esp32.out_queue.get_nowait()
            except queue.Empty:
                break
            if isinstance(item, Esp32Log):
                self._log(f"ESP32 {self._display_level(item.level)}：{item.message}")
            elif isinstance(item, CapSample):
                self.last_cap_time = item.monotonic_s
                snapshot = CombinedSnapshot.from_cap(item)
                self.buffer.append(snapshot)
                if self.recorder:
                    if self.training_active:
                        self.recorder.write_training_raw(snapshot)
                    else:
                        self.recorder.write_raw(snapshot)
                    if self.zero_drift_active:
                        self.recorder.write_zero_drift_raw(snapshot)
                if self.zero_drift_active:
                    self.zero_drift_samples.append(snapshot)
                self._add_cap_plot(item)

    def _drain_mini45(self) -> None:
        if not self.mini45:
            return
        while True:
            try:
                item = self.mini45.out_queue.get_nowait()
            except queue.Empty:
                break
            if isinstance(item, Mini45Log):
                self._log(f"Mini45 {self._display_level(item.level)}：{item.message}")
                if item.level in {"warning", "error"}:
                    self.mini_status.setText(f"Mini45 状态：{self._display_level(item.level)}：{item.message}")
            elif isinstance(item, ForceSample):
                first_sample = self.last_force_time <= 0.0
                try:
                    mapped_item = transform_force_sample(item, self.current_force_frame_mapping())
                except ValueError as exc:
                    if not self.force_mapping_error_logged:
                        self.force_mapping_error_logged = True
                        self._log(f"传感器坐标映射无效，Mini45 数据暂不进入标定数据流：{exc}")
                    self.force_frame_status.setText(f"坐标映射无效：{exc}")
                    self.force_frame_status.setStyleSheet("color: red")
                    continue
                filtered_item = self.force_filter.update(mapped_item, self._force_filter_settings())
                self.last_force_time = filtered_item.monotonic_s
                self.latest_force_sample = filtered_item
                if first_sample:
                    self.mini_status.setText("Mini45 状态：数据正常")
                    self._log("Mini45 已收到第一帧可解析数据")
                # 控制和稳定判定使用滤波后的传感器坐标力；CSV 原始时序仍写入未滤波数据。
                control_snapshot = CombinedSnapshot.from_force(filtered_item, raw_sample=item)
                raw_snapshot = CombinedSnapshot.from_force(mapped_item, raw_sample=item)
                self.buffer.append(control_snapshot)
                if self.recorder:
                    if self.training_active:
                        self.recorder.write_training_raw(raw_snapshot)
                    else:
                        self.recorder.write_raw(raw_snapshot)
                    if self.zero_drift_active:
                        self.recorder.write_zero_drift_raw(raw_snapshot)
                if self.zero_drift_active:
                    self.zero_drift_samples.append(raw_snapshot)
                self._add_force_plot(filtered_item)

    def _drain_motion(self) -> None:
        if not self.motion:
            return
        while True:
            try:
                item = self.motion.out_queue.get_nowait()
            except queue.Empty:
                break
            if isinstance(item, MotionMessage):
                if item.kind == "POS":
                    for axis in ("X", "Y", "Z"):
                        pos = parse_axis_position(item.values, axis)
                        if pos is not None:
                            self.motion_positions[axis] = pos
                    self._update_motion_status_from_position()
                elif item.kind in {"STATE", "LIMIT"}:
                    self._log(f"Arduino {item.kind}: {item.message}")
                elif item.level in {"warning", "error"}:
                    self.motion_status.setText(f"电机状态：{self._display_level(item.level)}，{item.message}")
                    self._log(f"Arduino {self._display_level(item.level)}：{item.message}")
                elif item.kind == "OK":
                    self._log(f"Arduino：{item.message or 'OK'}")
                elif item.kind != "UNKNOWN":
                    self._log(f"Arduino {item.kind}: {item.message}")

        now = time.monotonic()
        if now - self.motion_last_query > 1.0:
            self.motion_last_query = now
            try:
                self.motion.query_pos()
            except Exception:
                pass

    def _update_motion_status_from_position(self) -> None:
        parts = []
        for axis in ("X", "Y", "Z"):
            value = self.motion_positions.get(axis)
            parts.append(f"{axis}={value:.4f} mm" if value is not None else f"{axis}=--")
        prefix = "自动逼近中" if self.auto_force_active else "已连接"
        self.motion_status.setText(f"电机状态：{prefix}，" + "，".join(parts))

    def _update_auto_force(self) -> None:
        if not self.auto_force_active:
            return
        if not self.motion:
            self.stop_auto_force("Arduino 未连接")
            return
        if self.calibration_paused:
            return
        if not self.latest_force_sample or self.last_force_time <= 0.0:
            return

        now = time.monotonic()
        if now - self.last_force_time > 1.0:
            self.stop_auto_force("Mini45 数据超过 1 秒未更新")
            return

        sample = self.latest_force_sample
        if abs(sample.fz) > 10.0 or abs(sample.fx) > 4.0 or abs(sample.fy) > 4.0:
            self.stop_auto_force("力值超过安全限值")
            return
        if not self.force_control_result or not self.force_control_result.valid:
            self.stop_auto_force("K 未辨识或无效")
            return

        meta = self._meta()
        settings = self._stability_settings()
        target_force = [meta.target_fx, meta.target_fy, meta.target_fz]
        current_force = force_vector_from_sample(sample)
        error = [target_force[index] - current_force[index] for index in range(3)]
        tolerances = [settings.tolerance_fx, settings.tolerance_fy, settings.tolerance_fz]
        all_in_window = all(abs(error[index]) <= tolerances[index] for index in range(3))

        if all_in_window:
            if self.training_active:
                # 训练采集由 _update_training_collection 负责到达后立即切换目标，
                # 这里不停车等待，避免连续加载数据出现人为停顿。
                return
            if not self.auto_force_holding:
                self.auto_force_holding = True
                self.auto_force_in_window_since = now
                try:
                    self.motion.stop_all()
                except Exception:
                    pass
                self._log("三向力已进入目标窗口，开始稳定计时")

            window_samples = self.buffer.window(now, self.stable_window.value())
            result = evaluate_three_axis_stability(window_samples, meta, settings, SafetySettings())
            holding_long_enough = now - self.auto_force_in_window_since >= self.stable_window.value()
            if holding_long_enough and result.stable and not self.auto_force_marker_done:
                self.auto_force_marker_done = True
                self.add_marker()
                if self.calibration_mode == "sequence":
                    self.sequence_index += 1
                    self.stop_auto_force("")
                    self.start_next_sequence_target()
                else:
                    self.stop_calibration("已达到稳定窗口并记录标定点")
            return

        self.auto_force_holding = False
        if now < self.auto_force_next_move_time:
            return

        try:
            speed_mm_s = self.auto_speed_mm_s.value()
            command = compute_decoupled_command(
                k=self.force_control_result.k,
                target_force=target_force,
                current_force=current_force,
                state=self.force_control_state,
                settings=DecoupledControlSettings(
                    max_step_mm=self.auto_step_mm.value(),
                    min_pulse=1,
                    style=self._combo_value(self.control_style),
                ),
                safety=SafetySettings(),
                noise_norm=self.force_control_result.noise_norm,
            )
            sent_axes = []
            max_move_time = 0.0
            for motor_axis, delta_mm in command.delta_mm.items():
                if abs(delta_mm) < 1e-12:
                    continue
                self.motion.move_mm(motor_axis, delta_mm, speed_mm_s)
                sent_axes.append(f"{motor_axis}{delta_mm:+.4f}mm/{command.pulses[motor_axis]:+d}pulse")
                max_move_time = max(max_move_time, abs(delta_mm) / max(speed_mm_s, 1e-6))
            if not sent_axes:
                return
            self.auto_force_last_move = now
            self.auto_force_next_move_time = now + max(self.auto_interval_s.value(), max_move_time + 0.05)
            self.motion_status.setText(
                f"电机状态：解耦控制，" + "，".join(sent_axes)
            )
            self.cal_status.setText(
                f"标定状态：误差 Fx={error[0]:+.3f}, Fy={error[1]:+.3f}, Fz={error[2]:+.3f}，"
                f"trust={command.trust_scale:.2f}"
            )
            if self.recorder:
                self.recorder.write_force_control_log(
                    {
                        "experiment_id": meta.experiment_id,
                        "cycle_id": meta.cycle_id,
                        "target_Fx": meta.target_fx,
                        "target_Fy": meta.target_fy,
                        "target_Fz": meta.target_fz,
                        "current_Fx": current_force[0],
                        "current_Fy": current_force[1],
                        "current_Fz": current_force[2],
                        "error_Fx": error[0],
                        "error_Fy": error[1],
                        "error_Fz": error[2],
                        "delta_X_mm": command.delta_mm["X"],
                        "delta_Y_mm": command.delta_mm["Y"],
                        "delta_Z_mm": command.delta_mm["Z"],
                        "pulses_X": command.pulses["X"],
                        "pulses_Y": command.pulses["Y"],
                        "pulses_Z": command.pulses["Z"],
                        "damping_eta": command.damping_eta,
                        "trust_scale": command.trust_scale,
                        "condition": self.force_control_result.condition,
                        "predicted_dFx": command.predicted_delta_force[0],
                        "predicted_dFy": command.predicted_delta_force[1],
                        "predicted_dFz": command.predicted_delta_force[2],
                        "note": command.note,
                    }
                )
        except Exception as exc:
            self.stop_auto_force(str(exc))

    def _update_status(self) -> None:
        samples = self.buffer.window(time.monotonic(), self.stable_window.value())
        meta = self._meta()
        result = evaluate_three_axis_stability(samples, meta, self._stability_settings(), SafetySettings())
        self.window_label.setText(f"目标窗口：{self._yes_no(result.in_window)}")
        self.stable_label.setText(f"稳定状态：{self._yes_no(result.stable)}")
        self.safe_label.setText(f"安全状态：{self._yes_no(result.safe)}")
        self.window_label.setStyleSheet("color: green" if result.in_window else "color: #a66")
        self.stable_label.setStyleSheet("color: green" if result.stable else "color: #a66")
        self.safe_label.setStyleSheet("color: green" if result.safe else "color: red")
        if self.mini45 and self.last_force_time > 0.0 and time.monotonic() - self.last_force_time > 1.0:
            self.mini_status.setText("Mini45 状态：数据超过 1 秒未更新")

    def _meta(self) -> ExperimentMeta:
        if self.training_active and self.active_target:
            return ExperimentMeta(
                experiment_id=self.experiment_id.text().strip() or "exp001",
                cycle_id=self.current_cycle_id,
                branch=self.active_target.branch,
                axis="combined",
                direction=self.active_target.direction,
                preload_n=self.active_target.target_fz,
                target_fx=self.active_target.target_fx,
                target_fy=self.active_target.target_fy,
                target_fz=self.active_target.target_fz,
                note=self.note.text().strip(),
            )
        if self.training_active:
            return ExperimentMeta(
                experiment_id=self.experiment_id.text().strip() or "exp001",
                cycle_id=self.current_cycle_id,
                branch="training",
                axis="combined",
                direction="none",
                note=self.note.text().strip(),
            )
        if self.active_target:
            return self.active_target.to_meta(
                ExperimentMeta(
                    experiment_id=self.experiment_id.text().strip() or "exp001",
                    cycle_id=self.current_cycle_id,
                    note=self.note.text().strip(),
                )
            )
        return ExperimentMeta(
            experiment_id=self.experiment_id.text().strip() or "exp001",
            cycle_id=self.current_cycle_id,
            branch=self._combo_value(self.branch),
            axis=self._combo_value(self.load_axis),
            direction=self._combo_value(self.direction),
            preload_n=self.target_fz.value(),
            target_fx=self.target_fx.value(),
            target_fy=self.target_fy.value(),
            target_fz=self.target_fz.value(),
            note=self.note.text().strip(),
        )

    def _stability_settings(self) -> StabilitySettings:
        return StabilitySettings(
            stable_window_s=self.stable_window.value(),
            hold_window_s=self.hold_window.value(),
            tolerance_fx=self.tol_fx.value(),
            tolerance_fy=self.tol_fy.value(),
            tolerance_fz=self.tol_fz.value(),
        )

    def _add_force_plot(self, sample: ForceSample) -> None:
        x = sample.monotonic_s
        self.force_x.append(x)
        for key in self.force_y:
            self.force_y[key].append(getattr(sample, key))
        self._trim_plot(self.force_x, self.force_y)
        for key, curve in self.force_curves.items():
            curve.setData(self.force_x, self.force_y[key])
        for label, attr in (("Fx", "fx"), ("Fy", "fy"), ("Fz", "fz"), ("Mx", "mx"), ("My", "my"), ("Mz", "mz")):
            self.value_labels[label].setText(f"{getattr(sample, attr):.4f}")

    def _add_cap_plot(self, sample: CapSample) -> None:
        x = sample.monotonic_s
        self.cap_x.append(x)
        for key in self.cap_y:
            self.cap_y[key].append(getattr(sample, key))
        self._trim_plot(self.cap_x, self.cap_y)
        for key, curve in self.cap_curves.items():
            curve.setData(self.cap_x, self.cap_y[key])
        for label, attr in (("C0", "c0"), ("C1", "c1"), ("C2", "c2"), ("C3", "c3"), ("C4", "c4")):
            self.value_labels[label].setText(f"{getattr(sample, attr):.6f}")

    def _trim_plot(self, x_values: list[float], y_values: dict[str, list[float]], limit: int = 500) -> None:
        if len(x_values) <= limit:
            return
        del x_values[:-limit]
        for values in y_values.values():
            del values[:-limit]

    def _log(self, message: str) -> None:
        self.log.appendPlainText(f"[{time.strftime('%H:%M:%S')}] {message}")

    def _combo_value(self, combo: QComboBox) -> str:
        value = combo.currentData()
        return str(value if value is not None else combo.currentText())

    def _set_combo_by_data(self, combo: QComboBox, value: str) -> None:
        for index in range(combo.count()):
            data = combo.itemData(index)
            if str(data if data is not None else combo.itemText(index)) == value:
                combo.setCurrentIndex(index)
                return

    def _yes_no(self, value: bool) -> str:
        return "是" if value else "否"

    def _display_level(self, level: str) -> str:
        return {"info": "信息", "warning": "警告", "error": "错误", "debug": "调试"}.get(level, level)

    def _display_reason(self, reason: str) -> str:
        mapping = {
            "missing target-axis force samples": "没有目标方向力数据",
            "target-axis force outside tolerance window": "目标方向力未进入容差窗口",
            "target-axis force std too high": "目标方向力标准差过大",
            "force safety limit exceeded": "力值超过安全限值",
            "torque safety limit exceeded": "力矩超过安全限值",
        }
        translated = []
        for item in reason.split("; "):
            if item in mapping:
                translated.append(mapping[item])
            elif item.endswith(" cross-axis force too high"):
                translated.append(item.replace(" cross-axis force too high", " 非目标方向力过大"))
            elif item.endswith(" capacitance jump too high"):
                translated.append(item.replace(" capacitance jump too high", " 电容跳变过大"))
            else:
                translated.append(item)
        return "；".join(translated)

    def closeEvent(self, event) -> None:
        if self.esp32:
            self.esp32.stop()
        if self.mini45:
            self.mini45.stop()
        if self.motion:
            self.motion.stop()
        if self.recorder:
            self.recorder.stop()
        event.accept()
