from __future__ import annotations

import queue
import random
import time
from dataclasses import asdict
from pathlib import Path

try:
    import serial.tools.list_ports
except ImportError:  # pragma: no cover
    serial = None

from PyQt5.QtCore import Qt, QTimer
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
    QScrollArea,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
import pyqtgraph as pg

from .buffers import SampleBuffer
from .calibration import (
    CalibrationTarget,
    STATIC_FULL_FLOWS,
    TrainingTarget,
    filter_static_full_targets,
    generate_missing_static_full_diagonal_targets,
    generate_training_trajectory,
    generate_fz_sequence,
    generate_shear_sequence,
    generate_three_axis_sequence,
    generate_static_full_sequence,
    is_static_collection_mode,
    advance_ramped_force_target,
    parse_force_levels,
    parse_angles_deg,
    static_full_target_angle_deg,
    static_full_target_shear_n,
    training_target_reached,
    training_target_timed_out,
    validate_force_targets,
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
from .esp32_serial import Esp32Log, Esp32ProfileStatus, Esp32SerialAdapter
from .acquisition_profiles import STATIC_PRECISION, TRAINING_BALANCED, TRAINING_FAST, get_acquisition_profile
from .force_filter import ForceFilterSettings, ForceLowPassFilter
from .force_frame import AxisFrameMap, ForceFrameMapping, transform_force_sample
from .force_control import (
    MOTOR_AXES,
    DecoupledControlSettings,
    DecoupledControlState,
    KIdentificationResult,
    compute_decoupled_command,
    force_stats,
    force_vector_from_sample,
    identify_k_matrix,
)
from .mini45_netft import Mini45Log, Mini45NetFTAdapter, Mini45Simulator, fetch_netft_config
from .mini45_precomp import (
    ZERO_BIAS,
    compute_precomp_summary,
    save_precomp_summary,
    subtract_precomp_bias,
)
from .models import (
    CapSample,
    CombinedSnapshot,
    ExperimentMeta,
    ForceSample,
    SafetySettings,
    StabilitySettings,
    utc_timestamp,
)
from .recorder import CsvRecorder
from .stability import build_calibration_point, evaluate_three_axis_stability
from .static_point import StaticPointCollector, collection_tolerances
from .static_full_checkpoint import (
    checkpoint_path,
    legacy_completed_point_count,
    legacy_last_marker_id,
    load_checkpoint,
    load_last_force_mapping,
    load_last_valid_k_result,
    load_latest_precomp,
    save_checkpoint,
)
from .workflow import WorkflowState


STATIC_FULL_FLOW_UI = (
    ("fz", "Fz 单轴", "纯法向 Fz 0→最大→0"),
    ("fx", "Fx 预载剪切", "在每个 Fz 预载层级下扫 Fx 正负方向"),
    ("fy", "Fy 预载剪切", "在每个 Fz 预载层级下扫 Fy 正负方向"),
    ("diagonal", "斜向预载剪切", "在每个 Fz 预载层级下按斜向角度径向扫剪切"),
)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Mini45 + ESP32/MC1081 三维力标定上位机")
        self.resize(1280, 860)

        self.buffer = SampleBuffer(max_seconds=120)
        self.esp32 = None
        self.mini45 = None
        self.motion: ArduinoMotionAdapter | None = None
        self.recorder: CsvRecorder | None = None
        self.marker_id = 0
        self.last_force_time = 0.0
        self.last_cap_time = 0.0
        self.latest_force_sample: ForceSample | None = None
        self.force_filter = ForceLowPassFilter()
        self.mini45_precomp_active: bool = False
        self.mini45_precomp_enabled: bool = False
        self.mini45_precomp_samples: list[ForceSample] = []
        self.mini45_precomp_bias: dict[str, float] = dict(ZERO_BIAS)
        self.mini45_precomp_start_monotonic_s: float | None = None
        self.mini45_precomp_duration_s: float = 60.0
        self.mini45_precomp_quality: str = "none"
        self._mini45_cfgcpf: float | None = None
        self._mini45_cfgcpt: float | None = None
        self.motion_positions = {"X": None, "Y": None, "Z": None}
        self.auto_force_active = False
        self.auto_force_holding = False
        self.auto_force_marker_done = False
        self.auto_force_in_window_since = 0.0
        self.auto_force_last_move = 0.0
        self.auto_force_next_move_time = 0.0
        self.force_zero_active = False
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
        self.zero_drift_sample_count = 0
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
        self.current_cap_profile = ""
        self.current_cap_effective_hz = 0.0
        self.profile_switch_target = ""
        self.profile_switch_started_s = 0.0
        self.profile_switch_retry = 0
        self.static_point_collector: StaticPointCollector | None = None
        self.workflow = WorkflowState()
        self.workflow_stage_started = False
        self.workflow_pause_started_s = 0.0
        self.workflow_training_trajectory_index = 0
        self.workflow_training_trajectories = ("fx_roundtrip", "fy_roundtrip", "diagonal_roundtrip", "random_perturb")
        self.workflow_random_targets: dict[str, list[TrainingTarget]] = {}
        self.training_profile = TRAINING_BALANCED.name
        self.training_ramp_target = (0.0, 0.0, 0.0)
        self.training_last_ramp_update_s = 0.0

        # ---- 全静态标定 ----
        self.static_full_active: bool = False
        self.static_full_paused: bool = False
        self.static_full_points_completed: int = 0
        self.static_full_points_invalid: int = 0
        self.static_full_returning_zero: bool = False
        self._static_full_profile_wait: bool = False
        self._static_full_profile_start_s: float = 0.0
        self._static_full_pending_targets: list[CalibrationTarget] = []
        self._static_full_pending_resume: dict | None = None
        self.static_full_setup_stage: str = ""
        self.static_full_auto_recording: bool = False
        self.static_full_recovering_mini45: bool = False
        self.static_full_recovering_esp32: bool = False
        self.static_full_retest_active: bool = False
        self.static_full_retest_folder: Path | None = None
        self.static_full_retest_source_targets: list[CalibrationTarget] = []
        self.static_full_retest_targets: list[CalibrationTarget] = []
        self.static_full_retest_document: dict = {}
        self.static_full_retest_filter_spec: dict = {}
        self.static_full_retest_id: str = ""
        self.static_full_retest_source_experiment_id: str = ""
        self.static_full_retest_k_source: str = ""
        self.static_full_retest_precomp_source: str = ""
        self.static_full_retest_remeasure_precomp: bool = False
        self.static_full_retest_remeasure_k: bool = False
        self._mini45_reconnect_attempts: int = 0
        self._mini45_reconnect_next_s: float = 0.0
        self._mini45_reconnect_started_s: float = 0.0
        self._mini45_reconnect_adapter_started_s: float = 0.0
        self._esp32_reconnect_attempts: int = 0
        self._esp32_reconnect_next_s: float = 0.0
        self._esp32_reconnect_started_s: float = 0.0
        self._esp32_reconnect_adapter_started_s: float = 0.0
        self._esp32_reconnect_profile_next_s: float = 0.0

        self.force_x: list[float] = []
        self.force_y = {key: [] for key in ("fx", "fy", "fz")}
        self.cap_x: list[float] = []
        self.cap_y = {key: [] for key in ("c0", "c1", "c2", "c3", "c4")}
        self.force_plot_dirty = False
        self.cap_plot_dirty = False
        self.pending_force_plot_sample: ForceSample | None = None
        self.pending_cap_plot_sample: CapSample | None = None
        self.last_plot_flush_wall_s = 0.0
        self.plot_flush_interval_s = 0.10
        self.plot_tick_budget_s = 0.030
        self.last_force_plot_update_s = 0.0
        self.last_cap_plot_update_s = 0.0
        self.last_cal_progress_update_s = 0.0
        self.last_status_update_s = 0.0
        self.max_mini45_items_per_tick = 500
        self.max_esp32_items_per_tick = 200
        self.max_motion_items_per_tick = 100
        self.mini45_drain_budget_s = 0.008
        self.esp32_drain_budget_s = 0.003
        self.motion_drain_budget_s = 0.002

        self._build_ui()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(50)

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        tabs = QTabWidget()
        layout.addWidget(tabs, stretch=3)

        device_page, device_layout = self._tab_page()
        device_layout.addWidget(self._build_esp32_group())
        device_layout.addWidget(self._build_mini45_group())
        device_layout.addWidget(self._build_motion_group())
        device_layout.addStretch(1)
        tabs.addTab(device_page, "设备连接")

        experiment_page, experiment_layout = self._tab_page()
        experiment_layout.addWidget(self._build_force_frame_group())
        experiment_layout.addWidget(self._build_force_control_group())
        experiment_layout.addWidget(self._build_record_group())
        experiment_layout.addStretch(1)
        tabs.addTab(experiment_page, "实验配置")

        workflow_page, workflow_layout = self._tab_page()
        workflow_layout.addWidget(self._build_workflow_group())
        workflow_layout.addStretch(1)
        tabs.addTab(workflow_page, "完整流程")

        static_full_page, static_full_layout = self._tab_page()
        static_full_layout.addWidget(self._build_static_full_group())
        static_full_layout.addStretch(1)
        tabs.addTab(static_full_page, "全静态标定")

        calibration_page, calibration_layout = self._tab_page()
        calibration_layout.addWidget(self._build_calibration_group())
        calibration_layout.addStretch(1)
        tabs.addTab(calibration_page, "标定与训练")

        monitor_layout = QHBoxLayout()
        monitor_sidebar = QVBoxLayout()
        monitor_sidebar.addWidget(self._build_status_group())

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setPlaceholderText("运行日志")
        self.log.document().setMaximumBlockCount(1000)
        monitor_sidebar.addWidget(self.log, stretch=1)
        monitor_layout.addLayout(monitor_sidebar, stretch=1)

        plot_layout = QHBoxLayout()
        self.force_plot = pg.PlotWidget(title="Mini45 力数据（传感器坐标）")
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
        monitor_layout.addLayout(plot_layout, stretch=3)
        layout.addLayout(monitor_layout, stretch=2)

    def _tab_page(self) -> tuple[QScrollArea, QVBoxLayout]:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content = QWidget()
        layout = QVBoxLayout(content)
        scroll.setWidget(content)
        return scroll, layout

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
        self.force_scale_label = QLabel("—")
        form.addRow("力计数/单位（自动读取）", self.force_scale_label)
        self.torque_scale_label = QLabel("—")
        form.addRow("力矩计数/单位（自动读取）", self.torque_scale_label)
        btns = QHBoxLayout()
        self.mini_btn = QPushButton("连接 Mini45")
        self.mini_btn.clicked.connect(self.toggle_mini45)
        self.bias_btn = QPushButton("清零/偏置")
        self.bias_btn.clicked.connect(self.bias_mini45)
        btns.addWidget(self.mini_btn)
        btns.addWidget(self.bias_btn)
        form.addRow(btns)
        self.mini45_precomp_btn = QPushButton("Mini45 预补偿 60s")
        self.mini45_precomp_btn.clicked.connect(self.start_mini45_precomp)
        form.addRow(self.mini45_precomp_btn)
        self.mini45_precomp_status = QLabel("预补偿：未补偿")
        self.mini45_precomp_status.setWordWrap(True)
        form.addRow(self.mini45_precomp_status)
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
        self.auto_min_effective_step_mm = self._spin(0.0, 0.5, 0.02)
        self.auto_min_effective_step_mm.setSingleStep(MM_PER_PULSE)
        self.k_delta_x = self._spin(MM_PER_PULSE, 0.5, 0.10)
        self.k_delta_y = self._spin(MM_PER_PULSE, 0.5, 0.10)
        self.k_delta_z = self._spin(MM_PER_PULSE, 0.5, 0.10)
        for spin in (self.k_delta_x, self.k_delta_y, self.k_delta_z):
            spin.setSingleStep(MM_PER_PULSE)
        self.k_wait_s = self._spin(0.1, 5.0, 1.0)
        self.k_sample_s = self._spin(0.1, 5.0, 1.0)
        self.k_condition_limit = self._spin(10.0, 1000.0, 300.0)
        self.force_filter_enabled = QCheckBox("启用")
        self.force_filter_enabled.setChecked(True)
        self.force_filter_cutoff_hz = self._spin(0.1, 30.0, 1.5)
        self.force_filter_cutoff_hz.setSingleStep(0.5)
        self.force_filter_cutoff_hz.setToolTip("截止频率越低越平滑，静态标定推荐 1.5 Hz")
        self.force_filter_median_points = QSpinBox()
        self.force_filter_median_points.setRange(1, 9)
        self.force_filter_median_points.setSingleStep(2)
        self.force_filter_median_points.setValue(7)
        self.force_filter_median_points.setToolTip("中值窗口越大抗脉冲越强，推荐 5~9 点")
        self.force_filter_reset_btn = QPushButton("重置滤波")
        self.force_filter_reset_btn.clicked.connect(self.reset_force_filter)
        self.control_style = QComboBox()
        self.control_style.addItem("保守", "conservative")
        self.control_style.addItem("标准", "standard")
        self.control_style.addItem("快速", "fast")
        self._set_combo_by_data(self.control_style, "standard")

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

        grid.addWidget(QLabel("最小有效步 mm"), 6, 0)
        grid.addWidget(self.auto_min_effective_step_mm, 6, 1)

        k_buttons = QHBoxLayout()
        self.k_ident_btn = QPushButton("自动辨识 K")
        self.k_ident_btn.clicked.connect(self.start_k_identification)
        self.k_clear_btn = QPushButton("清除 K")
        self.k_clear_btn.clicked.connect(self.clear_force_control_k)
        k_buttons.addWidget(self.k_ident_btn)
        k_buttons.addWidget(self.k_clear_btn)
        grid.addLayout(k_buttons, 7, 0, 1, 4)

        self.k_status = QLabel("K 状态：未辨识")
        grid.addWidget(self.k_status, 8, 0, 1, 4)
        return box

    def _build_workflow_group(self) -> QGroupBox:
        box = QGroupBox("完整自动实验")
        grid = QGridLayout(box)
        self.workflow_balanced_enabled = QCheckBox("执行平衡频率训练")
        self.workflow_balanced_enabled.setChecked(True)
        self.workflow_fast_enabled = QCheckBox("执行高速补充训练")
        self.workflow_fast_enabled.setChecked(True)
        self.workflow_start_btn = QPushButton("开始完整自动实验")
        self.workflow_start_btn.clicked.connect(self.start_full_workflow)
        self.workflow_pause_btn = QPushButton("暂停")
        self.workflow_pause_btn.clicked.connect(self.pause_full_workflow)
        self.workflow_resume_btn = QPushButton("继续")
        self.workflow_resume_btn.clicked.connect(self.resume_full_workflow)
        self.workflow_stop_btn = QPushButton("停止/急停")
        self.workflow_stop_btn.clicked.connect(lambda: self.abort_full_workflow("人工停止"))
        self.workflow_status_label = QLabel("完整流程：未运行")
        self.workflow_profile_label = QLabel("MC1081 配置：未确认")
        self.workflow_point_label = QLabel("静态点：--，稳定保持 0.0/5.0 s，电容样本 0/45")
        self.workflow_count_label = QLabel("完成 0，无效 0，训练跳过 0")

        grid.addWidget(self.workflow_balanced_enabled, 0, 0)
        grid.addWidget(self.workflow_fast_enabled, 0, 1)
        grid.addWidget(self.workflow_start_btn, 0, 2)
        grid.addWidget(self.workflow_pause_btn, 0, 3)
        grid.addWidget(self.workflow_resume_btn, 0, 4)
        grid.addWidget(self.workflow_stop_btn, 0, 5)
        grid.addWidget(self.workflow_status_label, 1, 0, 1, 3)
        grid.addWidget(self.workflow_profile_label, 1, 3, 1, 3)
        grid.addWidget(self.workflow_point_label, 2, 0, 1, 4)
        grid.addWidget(self.workflow_count_label, 2, 4, 1, 2)
        self._update_workflow_ui()
        return box

    def _build_static_full_group(self) -> QGroupBox:
        box = QGroupBox("全静态标定流程")
        form = QFormLayout(box)

        # ── Fz 单轴标定（纯法向，Fx=Fy=0）──
        self.static_full_fz_label = QLabel("Fz 单轴标定")
        self.static_full_fz_label.setStyleSheet("font-weight: bold;")
        fz_row = QHBoxLayout()
        self.static_full_fz_max = self._spin(0.0, 10.0, 9.0)
        self.static_full_fz_max.setToolTip("Fz 单轴最大力 (N)")
        self.static_full_fz_max.valueChanged.connect(self._update_static_full_estimate)
        self.static_full_fz_step = self._spin(0.1, 10.0, 1.0)
        self.static_full_fz_step.setToolTip("Fz 单轴步长 (N)")
        self.static_full_fz_step.valueChanged.connect(self._update_static_full_estimate)
        fz_row.addWidget(QLabel("最大"))
        fz_row.addWidget(self.static_full_fz_max)
        fz_row.addWidget(QLabel("步长"))
        fz_row.addWidget(self.static_full_fz_step)
        form.addRow(self.static_full_fz_label, fz_row)

        # ── Fz 预载层级（保持 Fz 不变，扫剪切）──
        self.static_full_preload_levels = QLineEdit("0,1,3,5")
        self.static_full_preload_levels.setToolTip("逗号分隔：保持法向力不变时扫剪切的层级 (N)。含 0 做纯剪切基线")
        self.static_full_preload_levels.textChanged.connect(self._update_static_full_estimate)
        form.addRow("Fz 预载层级 N", self.static_full_preload_levels)

        # ── 剪切力（Fx / Fy / 斜向共用）──
        self.static_full_shear_label = QLabel("剪切力（Fx / Fy / 斜向）")
        self.static_full_shear_label.setStyleSheet("font-weight: bold;")
        shear_row = QHBoxLayout()
        self.static_full_shear_max = self._spin(0.0, 4.0, 3.6)
        self.static_full_shear_max.setToolTip("剪切最大力 (N)")
        self.static_full_shear_max.valueChanged.connect(self._update_static_full_estimate)
        self.static_full_shear_step = self._spin(0.1, 4.0, 0.6)
        self.static_full_shear_step.setToolTip("剪切力步长 (N)")
        self.static_full_shear_step.valueChanged.connect(self._update_static_full_estimate)
        shear_row.addWidget(QLabel("最大"))
        shear_row.addWidget(self.static_full_shear_max)
        shear_row.addWidget(QLabel("步长"))
        shear_row.addWidget(self.static_full_shear_step)
        form.addRow(self.static_full_shear_label, shear_row)

        # ── 斜向角度 ──
        self.static_full_angles = QLineEdit("30,60,120,150,210,240,300,330")
        self.static_full_angles.setToolTip("斜向加载角度（度），逗号分隔。0/90/180/270 已由 Fx/Fy 覆盖")
        self.static_full_angles.textChanged.connect(self._update_static_full_estimate)
        form.addRow("斜向角度 °", self.static_full_angles)

        self.static_full_cap_samples = QSpinBox()
        self.static_full_cap_samples.setRange(1, 1000)
        self.static_full_cap_samples.setValue(45)
        self.static_full_cap_samples.setToolTip("每个稳定点采集的唯一电容样本数量")
        self.static_full_cap_samples.valueChanged.connect(self._update_static_full_estimate)
        form.addRow("稳定点电容样本数", self.static_full_cap_samples)

        # ── 测量流程 ──
        flow_box = QGroupBox("测量流程")
        flow_grid = QGridLayout(flow_box)
        self.static_full_flow_checks: dict[str, QCheckBox] = {}
        self.static_full_flow_cycles: dict[str, QSpinBox] = {}
        flow_grid.addWidget(QLabel("启用"), 0, 0)
        flow_grid.addWidget(QLabel("流程"), 0, 1)
        flow_grid.addWidget(QLabel("重复组数"), 0, 2)
        for row, (flow, label, tooltip) in enumerate(STATIC_FULL_FLOW_UI, start=1):
            check = QCheckBox()
            check.setChecked(True)
            check.setToolTip(tooltip)
            check.stateChanged.connect(self._update_static_full_flow_controls)
            cycles = QSpinBox()
            cycles.setRange(1, 20)
            cycles.setValue(3)
            cycles.setToolTip(f"{label}重复组数")
            cycles.valueChanged.connect(self._update_static_full_estimate)
            self.static_full_flow_checks[flow] = check
            self.static_full_flow_cycles[flow] = cycles
            flow_grid.addWidget(check, row, 0)
            flow_grid.addWidget(QLabel(label), row, 1)
            flow_grid.addWidget(cycles, row, 2)
        form.addRow(flow_box)

        # ── 预估时间 ──
        self.static_full_estimate = QLabel("—")
        self.static_full_estimate.setStyleSheet("font-weight: bold; color: #1565C0;")
        form.addRow("预估时间", self.static_full_estimate)

        # ── 按钮 ──
        btns = QHBoxLayout()
        self.static_full_start_btn = QPushButton("开始全静态标定")
        self.static_full_start_btn.clicked.connect(self.start_static_full)
        self.static_full_resume_file_btn = QPushButton("从已有实验继续")
        self.static_full_resume_file_btn.clicked.connect(self.resume_static_full_from_folder)
        self.static_full_pause_btn = QPushButton("暂停")
        self.static_full_pause_btn.clicked.connect(self.pause_static_full)
        self.static_full_resume_btn = QPushButton("继续")
        self.static_full_resume_btn.clicked.connect(self.resume_static_full)
        self.static_full_stop_btn = QPushButton("停止/急停")
        self.static_full_stop_btn.clicked.connect(lambda: self.stop_static_full("人工停止"))
        for b in (self.static_full_start_btn, self.static_full_resume_file_btn, self.static_full_pause_btn,
                  self.static_full_resume_btn, self.static_full_stop_btn):
            btns.addWidget(b)
        form.addRow(btns)

        # ── 状态 ──
        self.static_full_status = QLabel("空闲")
        self.static_full_status.setWordWrap(True)
        self.static_full_status.setMinimumHeight(100)
        self.static_full_status.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        form.addRow(self.static_full_status)

        form.addRow(self._build_static_full_retest_group())

        self._update_static_full_buttons()
        self._update_static_full_estimate()
        return box

    def _update_static_full_flow_controls(self, *args: object) -> None:
        if not hasattr(self, "static_full_flow_checks"):
            return
        active = self._static_full_busy() if hasattr(self, "static_full_active") else False
        for flow, cycles in self.static_full_flow_cycles.items():
            check = self.static_full_flow_checks[flow]
            cycles.setEnabled(check.isChecked() and not active)
        self._update_static_full_estimate()

    def _static_full_enabled_flows(self) -> list[str]:
        if not hasattr(self, "static_full_flow_checks"):
            return list(STATIC_FULL_FLOWS)
        return [flow for flow in STATIC_FULL_FLOWS if self.static_full_flow_checks[flow].isChecked()]

    def _static_full_flow_cycle_values(self) -> dict[str, int]:
        if not hasattr(self, "static_full_flow_cycles"):
            return {flow: 3 for flow in STATIC_FULL_FLOWS}
        return {flow: int(self.static_full_flow_cycles[flow].value()) for flow in STATIC_FULL_FLOWS}

    def _static_full_required_cap_samples(self) -> int:
        if not hasattr(self, "static_full_cap_samples"):
            return 45
        return int(self.static_full_cap_samples.value())

    def _build_static_full_retest_group(self) -> QGroupBox:
        box = QGroupBox("补测")
        layout = QVBoxLayout(box)

        folder_row = QHBoxLayout()
        self.static_full_retest_folder_text = QLineEdit()
        self.static_full_retest_folder_text.setReadOnly(True)
        self.static_full_retest_folder_text.setPlaceholderText("选择已有全静态批次目录")
        self.static_full_retest_load_btn = QPushButton("选择已有批次")
        self.static_full_retest_load_btn.clicked.connect(self.load_static_full_retest_folder)
        folder_row.addWidget(self.static_full_retest_folder_text, stretch=1)
        folder_row.addWidget(self.static_full_retest_load_btn)
        layout.addLayout(folder_row)

        grid = QGridLayout()
        self.static_full_retest_axis_checks: dict[str, QCheckBox] = {}
        axis_row = QHBoxLayout()
        for label, value in (("Fz", "Fz"), ("Fx", "Fx"), ("Fy", "Fy"), ("斜向", "combined")):
            check = QCheckBox(label)
            check.setChecked(True)
            check.stateChanged.connect(self._update_static_full_retest_preview)
            self.static_full_retest_axis_checks[value] = check
            axis_row.addWidget(check)
        grid.addWidget(QLabel("轴/阶段"), 0, 0)
        grid.addLayout(axis_row, 0, 1)

        self.static_full_retest_branch_checks: dict[str, QCheckBox] = {}
        branch_row = QHBoxLayout()
        for label, value in (("加载", "loading"), ("卸载", "unloading")):
            check = QCheckBox(label)
            check.setChecked(True)
            check.stateChanged.connect(self._update_static_full_retest_preview)
            self.static_full_retest_branch_checks[value] = check
            branch_row.addWidget(check)
        grid.addWidget(QLabel("分支"), 1, 0)
        grid.addLayout(branch_row, 1, 1)

        self.static_full_retest_direction_checks: dict[str, QCheckBox] = {}
        direction_row = QHBoxLayout()
        for label, value in (("正向", "positive"), ("负向", "negative"), ("无", "none")):
            check = QCheckBox(label)
            check.setChecked(True)
            check.stateChanged.connect(self._update_static_full_retest_preview)
            self.static_full_retest_direction_checks[value] = check
            direction_row.addWidget(check)
        grid.addWidget(QLabel("方向"), 2, 0)
        grid.addLayout(direction_row, 2, 1)

        self.static_full_retest_cycles = QLineEdit()
        self.static_full_retest_cycles.setPlaceholderText("空=全部，例如 1,2")
        self.static_full_retest_cycles.textChanged.connect(self._update_static_full_retest_preview)
        grid.addWidget(QLabel("循环号"), 3, 0)
        grid.addWidget(self.static_full_retest_cycles, 3, 1)

        self.static_full_retest_preloads = QLineEdit()
        self.static_full_retest_preloads.setPlaceholderText("空=全部，例如 0,1,3")
        self.static_full_retest_preloads.textChanged.connect(self._update_static_full_retest_preview)
        grid.addWidget(QLabel("Fz 预载 N"), 4, 0)
        grid.addWidget(self.static_full_retest_preloads, 4, 1)

        self.static_full_retest_shear_levels = QLineEdit()
        self.static_full_retest_shear_levels.setPlaceholderText("空=全部，例如 0.6,1.2")
        self.static_full_retest_shear_levels.textChanged.connect(self._update_static_full_retest_preview)
        grid.addWidget(QLabel("剪切幅值 N"), 5, 0)
        grid.addWidget(self.static_full_retest_shear_levels, 5, 1)

        self.static_full_retest_angles = QLineEdit()
        self.static_full_retest_angles.setPlaceholderText("空=全部，例如 30,330")
        self.static_full_retest_angles.textChanged.connect(self._update_static_full_retest_preview)
        grid.addWidget(QLabel("斜向角度 °"), 6, 0)
        grid.addWidget(self.static_full_retest_angles, 6, 1)
        layout.addLayout(grid)

        reuse_row = QHBoxLayout()
        self.static_full_retest_reprecomp = QCheckBox("重新做 Mini45 预补偿")
        self.static_full_retest_rek = QCheckBox("重新辨识 K")
        reuse_row.addWidget(self.static_full_retest_reprecomp)
        reuse_row.addWidget(self.static_full_retest_rek)
        layout.addLayout(reuse_row)

        btns = QHBoxLayout()
        self.static_full_retest_start_btn = QPushButton("开始补测")
        self.static_full_retest_start_btn.clicked.connect(self.start_static_full_retest)
        self.static_full_retest_pause_btn = QPushButton("暂停补测")
        self.static_full_retest_pause_btn.clicked.connect(self.pause_static_full)
        self.static_full_retest_resume_btn = QPushButton("继续补测")
        self.static_full_retest_resume_btn.clicked.connect(self.resume_static_full)
        self.static_full_retest_stop_btn = QPushButton("停止补测")
        self.static_full_retest_stop_btn.clicked.connect(lambda: self.stop_static_full("补测停止"))
        for button in (
            self.static_full_retest_start_btn,
            self.static_full_retest_pause_btn,
            self.static_full_retest_resume_btn,
            self.static_full_retest_stop_btn,
        ):
            btns.addWidget(button)
        layout.addLayout(btns)

        self.static_full_retest_status = QLabel("未加载补测批次")
        self.static_full_retest_status.setWordWrap(True)
        layout.addWidget(self.static_full_retest_status)
        return box

    def _build_calibration_group(self) -> QGroupBox:
        box = QGroupBox("标定控制")
        layout = QVBoxLayout(box)

        self.basic_group = QGroupBox("基础信息")
        grid = QGridLayout(self.basic_group)
        self.note = QLineEdit()
        self.experiment_mode = QComboBox()
        self.experiment_mode.addItem("空载零点漂移", "zero")
        self.experiment_mode.addItem("单目标点标定", "single")
        self.experiment_mode.addItem("静态正反程标定", "sequence")
        self.experiment_mode.addItem("训练数据采集", "combined")
        self.load_axis = QComboBox()
        self.load_axis.addItems(["Fx", "Fy", "Fz"])
        self.load_axis.addItem("三轴自动(Fz→Fx→Fy)", "all")
        self.load_axis.setCurrentText("Fz")
        self.branch = QComboBox()
        self.branch.addItem("加载", "loading")
        self.branch.addItem("卸载", "unloading")
        self.direction = QComboBox()
        self.direction.addItem("无", "none")
        self.direction.addItem("正向", "positive")
        self.direction.addItem("负向", "negative")
        grid.addWidget(QLabel("实验模式"), 0, 0)
        grid.addWidget(self.experiment_mode, 0, 1)
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
        self.tol_fx = self._spin(0, 5, 0.05)
        self.tol_fy = self._spin(0, 5, 0.05)
        self.tol_fz = self._spin(0, 5, 0.08)
        for row, name, target, tolerance in (
            (0, "Fx", self.target_fx, self.tol_fx),
            (1, "Fy", self.target_fy, self.tol_fy),
            (2, "Fz", self.target_fz, self.tol_fz),
        ):
            grid.addWidget(QLabel(f"目标 {name}"), row, 0)
            grid.addWidget(target, row, 1)
            grid.addWidget(QLabel(f"容差 {name}"), row, 2)
            grid.addWidget(tolerance, row, 3)
        self.stable_window = self._spin(0.5, 20, 5.0)
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
        self.training_fz_levels = QLineEdit("1,2,3,4,5,6,7,8,9")
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
        self.force_zero_btn = QPushButton("力归0/卸载")
        self.force_zero_btn.clicked.connect(self.start_force_zero_unload)
        self.cal_skip_btn = QPushButton("跳过当前点")
        self.cal_skip_btn.clicked.connect(self.skip_calibration_point)
        self.cal_stop_btn = QPushButton("停止/急停")
        self.cal_stop_btn.clicked.connect(lambda: self.stop_calibration("人工停止"))
        for button in (
            self.cal_start_btn,
            self.cal_pause_btn,
            self.cal_resume_btn,
            self.force_zero_btn,
            self.cal_skip_btn,
            self.cal_stop_btn,
        ):
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
        self.experiment_id = QLineEdit("sensor01_mount01")
        form.addRow("实验批次/安装编号", self.experiment_id)
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
        axis = self._combo_value(self.load_axis)
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

        shear_axis = axis in {"Fx", "Fy", "all"}
        for widget in (self.seq_fz_label, self.seq_fz_max, self.seq_fz_step):
            widget.setVisible(is_sequence and axis in {"Fz", "all"})
        for widget in (self.seq_shear_label, self.seq_shear_max, self.seq_shear_step, self.seq_shear_direction_label, self.seq_shear_direction):
            widget.setVisible(is_sequence and shear_axis)
        self.cal_pause_btn.setVisible(is_single or is_sequence or is_combined)
        self.cal_resume_btn.setVisible(is_single or is_sequence or is_combined)
        self.cal_skip_btn.setVisible(is_sequence or is_combined)
        self._update_calibration_buttons()

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

    def _calibration_active(self) -> bool:
        return bool(
            self.mini45_precomp_active
            or self.zero_drift_active
            or self.training_active
            or self.static_full_active
            or self._static_full_profile_wait
            or bool(self.static_full_setup_stage)
            or self.auto_force_active
            or self.auto_force_holding
            or self.force_zero_active
        )

    def _plot_updates_suspended(self) -> bool:
        # 子实验运行时优先保证采集、保存和力控，不重绘曲线。
        return self._calibration_active() or self.k_ident_active

    def _update_calibration_buttons(self) -> None:
        if not hasattr(self, "cal_start_btn"):
            return
        active = self._calibration_active()
        paused = self.calibration_paused and active
        zero_ready, _reason = self._force_zero_unload_ready()
        self.cal_start_btn.setEnabled(not active and not self.k_ident_active)
        self.cal_start_btn.setText("标定运行中" if active else "开始标定")
        self.cal_pause_btn.setEnabled(active and not paused and not self.zero_drift_active and not self.force_zero_active)
        self.cal_resume_btn.setEnabled(active and paused and not self.force_zero_active)
        self.force_zero_btn.setEnabled(zero_ready)
        self.force_zero_btn.setText("力归0中" if self.force_zero_active else "力归0/卸载")
        self.cal_stop_btn.setEnabled(active)
        self.cal_skip_btn.setEnabled(
            active and not self.force_zero_active and (self.training_active or self.calibration_mode == "sequence")
        )

    def _force_zero_unload_ready(self) -> tuple[bool, str]:
        if self.force_zero_active:
            return False, "力归0/卸载正在运行"
        if self.k_ident_active:
            return False, "K 辨识过程中不能启动力归0/卸载"
        if self.mini45_precomp_active:
            return False, "Mini45 预补偿测量过程中不能启动力归0/卸载"
        if self.zero_drift_active:
            return False, "零点漂移采集过程中不能启动力归0/卸载"
        if self._static_full_profile_wait or self.static_full_setup_stage:
            return False, "全静态标定准备阶段不能启动力归0/卸载"
        if self.static_full_recovering_mini45:
            return False, "Mini45 自动重连恢复过程中不能启动力归0/卸载"
        if self.static_full_recovering_esp32:
            return False, "ESP32 自动重连恢复过程中不能启动力归0/卸载"
        if self.static_full_returning_zero:
            return False, "全静态标定已经在自动卸载回零"
        if not self.motion:
            return False, "请先连接 Arduino 电机控制串口"
        if not self.mini45:
            return False, "请先连接 Mini45 并确认有实时力数据"
        if not self.force_control_result or not self.force_control_result.valid:
            return False, "请先完成有效的 K 自动辨识"
        now = time.monotonic()
        if not self.latest_force_sample or self.last_force_time <= 0.0 or now - self.last_force_time > 1.0:
            return False, "Mini45 最近 1 秒内没有实时力数据"
        return True, ""

    def start_force_zero_unload(self) -> None:
        ready, reason = self._force_zero_unload_ready()
        if not ready:
            QMessageBox.warning(self, "力归0/卸载", reason)
            return

        now = time.monotonic()
        if self.workflow.active and not self.workflow.paused:
            self.workflow.paused = True
            self.workflow_pause_started_s = now
            self._write_workflow_event("force_zero_pause", "paused", "力归0/卸载")
            self._update_workflow_ui()

        if self.static_full_active:
            if not self.static_full_paused:
                self.static_full_paused = True
                self._save_static_full_checkpoint("paused", "力归0/卸载")
            self.static_full_status.setText("已暂停，正在执行力归0/卸载")
            self._update_static_full_buttons()

        if self.training_active and self.training_pause_started_s <= 0.0:
            self.training_pause_started_s = now
        if self.static_point_collector:
            self.static_point_collector.begin(now)

        self.calibration_paused = bool(
            self.training_active
            or self.static_full_active
            or self.calibration_mode
            or self.workflow.active
            or self.auto_force_active
            or self.auto_force_holding
        )
        self.stop_auto_force("力归0/卸载准备")

        self.force_zero_active = True
        if not self.start_auto_force():
            self.force_zero_active = False
            self._update_calibration_buttons()
            return

        self.cal_status.setText("标定状态：力归0/卸载中，完成后保持暂停")
        self._log("开始力归0/卸载：临时目标 Fx=0, Fy=0, Fz=0，完成后保持暂停")
        self._update_calibration_buttons()

    def _finish_force_zero_unload(self, reason: str = "完成") -> None:
        if not self.force_zero_active:
            return
        self.force_zero_active = False
        self.stop_auto_force("")
        self.calibration_paused = bool(
            self.training_active
            or self.static_full_active
            or self.calibration_mode
            or self.workflow.active
        )
        self.cal_status.setText(f"标定状态：力归0/卸载{reason}，保持暂停")
        self.motion_status.setText("电机状态：力归0/卸载完成，保持暂停")
        self._log(f"力归0/卸载{reason}，流程保持暂停")
        self._update_static_full_buttons()
        self._update_workflow_ui()
        self._update_calibration_buttons()

    def toggle_esp32(self) -> None:
        if self.static_full_recovering_esp32:
            return
        if self.esp32:
            if self.static_full_active:
                self._begin_static_full_esp32_recovery("用户请求重新连接 ESP32")
                return
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

    def _start_esp32_reconnect_adapter(self) -> None:
        port = self.esp_port.currentText()
        if not port:
            raise RuntimeError("ESP32 串口为空")
        adapter = Esp32SerialAdapter(
            port=port,
            baud=int(self.esp_baud.currentText()),
            mode=self._combo_value(self.esp_mode),
            rate_hz=self.esp_rate.value(),
        )
        self.last_cap_time = 0.0
        self.current_cap_effective_hz = 0.0
        self.current_cap_profile = ""
        self.esp32 = adapter
        adapter.start()
        self.esp_btn.setText("ESP32 重连中")
        self.workflow_profile_label.setText("MC1081 配置：ESP32 重连中")
        try:
            adapter.set_profile(STATIC_PRECISION.name)
            self._esp32_reconnect_profile_next_s = time.monotonic() + 2.0
        except Exception:
            self._esp32_reconnect_profile_next_s = time.monotonic() + 1.0

    def _begin_static_full_esp32_recovery(self, reason: str) -> None:
        if not self.static_full_active or self.static_full_recovering_esp32:
            return
        self.static_full_recovering_esp32 = True
        self.static_full_paused = True
        self.calibration_paused = True
        self._esp32_reconnect_attempts = 0
        self._esp32_reconnect_next_s = time.monotonic()
        self._esp32_reconnect_started_s = time.monotonic()
        self._esp32_reconnect_adapter_started_s = 0.0
        self._esp32_reconnect_profile_next_s = 0.0
        self.stop_auto_force("ESP32 断流，已安全暂停")
        self.static_point_collector = None
        if self.esp32:
            try:
                self.esp32.stop()
            except Exception:
                pass
        self.esp32 = None
        self.last_cap_time = 0.0
        self.current_cap_effective_hz = 0.0
        self.esp_btn.setEnabled(False)
        self.static_full_status.setText(
            f"ESP32 电容数据中断，流程已安全暂停并保留第 {self.sequence_index + 1} 点进度。\n"
            "正在自动重连；重连后将重新切换静态采集配置并重新采集当前点。"
        )
        self._log(f"{reason}；全静态标定已暂停，开始自动重连 ESP32")
        self._save_static_full_checkpoint("recovering", reason)
        self._update_static_full_buttons()

    def _update_static_full_esp32_recovery(self) -> None:
        if not self.static_full_recovering_esp32:
            return
        now = time.monotonic()
        if self.esp32 and self.last_cap_time > 0.0 and now - self.last_cap_time <= 1.0:
            if self._profile_is_ready(STATIC_PRECISION.name):
                self.static_full_recovering_esp32 = False
                self.static_full_paused = False
                self.calibration_paused = False
                self.esp_btn.setEnabled(True)
                self.esp_btn.setText("断开 ESP32")
                self._log(
                    f"ESP32 自动重连成功（第 {self._esp32_reconnect_attempts} 次尝试），"
                    f"从第 {self.sequence_index + 1} 点重新稳定并继续"
                )
                self._save_static_full_checkpoint("running", "ESP32 自动重连成功")
                self._update_static_full_buttons()
                self.start_next_sequence_target()
                return
            if now >= self._esp32_reconnect_profile_next_s:
                try:
                    self.esp32.set_profile(STATIC_PRECISION.name)
                    self._esp32_reconnect_profile_next_s = now + 2.0
                    self._log(f"ESP32 自动重连：重新请求 {STATIC_PRECISION.name} 配置")
                except Exception as exc:
                    self._log(f"ESP32 自动重连：请求静态采集配置失败：{exc}")
                    self._esp32_reconnect_profile_next_s = now + 2.0
            self.static_full_status.setText(
                f"ESP32 已恢复数据，正在等待 {STATIC_PRECISION.name} 配置生效；"
                f"当前 {self.current_cap_profile or '未确认'}，{self.current_cap_effective_hz:.2f} Hz"
            )
            return
        if self.esp32 and now - self._esp32_reconnect_adapter_started_s > 8.0:
            try:
                self.esp32.stop()
            except Exception:
                pass
            self.esp32 = None
            self._esp32_reconnect_next_s = now + 2.0
        if self.esp32 or now < self._esp32_reconnect_next_s:
            return
        self._esp32_reconnect_attempts += 1
        try:
            self._start_esp32_reconnect_adapter()
            self._esp32_reconnect_adapter_started_s = now
            self._log(f"ESP32 自动重连：第 {self._esp32_reconnect_attempts} 次尝试已打开串口")
        except Exception as exc:
            self.esp32 = None
            self._esp32_reconnect_next_s = now + min(10.0, 2.0 + self._esp32_reconnect_attempts)
            self.static_full_status.setText(
                f"ESP32 自动重连第 {self._esp32_reconnect_attempts} 次失败：{exc}\n"
                "流程保持暂停，稍后继续重试；也可点击停止并之后从已有实验继续。"
            )
            self._log(f"ESP32 自动重连失败：{exc}")

    def toggle_mini45(self) -> None:
        if self.static_full_recovering_mini45:
            return
        if self.mini45:
            if self.static_full_active:
                self._begin_static_full_mini45_recovery("用户请求重新连接 Mini45")
                return
            if self.mini45_precomp_active:
                self.finish_mini45_precomp("Mini45 已断开")
            elif self.mini45_precomp_enabled:
                self._invalidate_mini45_precomp("Mini45 已断开，预补偿已失效")
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
                self.force_scale_label.setText("—")
                self.torque_scale_label.setText("—")
                self._mini45_cfgcpf = None
                self._mini45_cfgcpt = None
            else:
                ip = self.mini_ip.text().strip()
                if not ip:
                    QMessageBox.critical(self, "Mini45", "请填写 NETBA IP 地址")
                    return
                # 强制从 NETBA 自动读取校准系数
                try:
                    config = fetch_netft_config(ip)
                except Exception as fetch_exc:
                    QMessageBox.critical(
                        self, "Mini45",
                        f"无法从 NETBA 读取校准系数（cfgcpf/cfgcpt），请检查 NETBA 网络连接。\n\n"
                        f"错误详情：{fetch_exc}\n\n"
                        f"Mini45 需要 NETBA 盒提供 counts-per-unit 校准值才能正确转换力/力矩单位。"
                    )
                    return
                cfgcpf_str = config.get("cfgcpf")
                cfgcpt_str = config.get("cfgcpt")
                if not cfgcpf_str or not cfgcpt_str:
                    QMessageBox.critical(
                        self, "Mini45",
                        f"NETBA 返回的配置中缺少校准系数。\n"
                        f"cfgcpf={cfgcpf_str or '缺失'}，cfgcpt={cfgcpt_str or '缺失'}\n\n"
                        f"请确认 NETBA 已正确配置 Mini45 传感器的校准参数。"
                    )
                    return
                force_cpu = float(cfgcpf_str)
                torque_cpu = float(cfgcpt_str)
                self._mini45_cfgcpf = force_cpu
                self._mini45_cfgcpt = torque_cpu
                self.force_scale_label.setText(f"{force_cpu:g}")
                self.torque_scale_label.setText(f"{torque_cpu:g}")
                force_unit = config.get("scfgfu", "")
                torque_unit = config.get("scfgtu", "")
                rdt_rate = config.get("comrdtrate", "")
                self._log(
                    f"已从 netftapi2.xml 自动读取校准系数："
                    f"cfgcpf={cfgcpf_str}，cfgcpt={cfgcpt_str}"
                    f"{'，力单位 ' + force_unit if force_unit else ''}"
                    f"{'，力矩单位 ' + torque_unit if torque_unit else ''}"
                    f"{'，RDT 频率 ' + rdt_rate + ' Hz' if rdt_rate else ''}"
                )
                self.mini45 = Mini45NetFTAdapter(
                    ip=ip,
                    port=self.mini_port.value(),
                    force_counts_per_unit=force_cpu,
                    torque_counts_per_unit=torque_cpu,
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

    def _start_mini45_reconnect_adapter(self) -> None:
        mini_mode = self._combo_value(self.mini_mode)
        if mini_mode == "simulator":
            adapter = Mini45Simulator(rate_hz=100)
        else:
            ip = self.mini_ip.text().strip()
            if not ip:
                raise RuntimeError("Mini45 IP 地址为空")
            force_cpu = getattr(self, "_mini45_cfgcpf", None)
            torque_cpu = getattr(self, "_mini45_cfgcpt", None)
            if force_cpu is None or torque_cpu is None:
                config = fetch_netft_config(ip)
                force_cpu = float(config["cfgcpf"])
                torque_cpu = float(config["cfgcpt"])
                self._mini45_cfgcpf = force_cpu
                self._mini45_cfgcpt = torque_cpu
            adapter = Mini45NetFTAdapter(
                ip=ip,
                port=self.mini_port.value(),
                force_counts_per_unit=float(force_cpu),
                torque_counts_per_unit=float(torque_cpu),
            )
        self.last_force_time = 0.0
        self.latest_force_sample = None
        self.reset_force_filter(log=False)
        self.mini45 = adapter
        adapter.start()
        self.mini_btn.setText("Mini45 重连中")
        self.mini_status.setText("Mini45 状态：自动重连后等待第一帧数据")

    def _begin_static_full_mini45_recovery(self, reason: str) -> None:
        if not self.static_full_active or self.static_full_recovering_mini45 or self.static_full_recovering_esp32:
            return
        self.static_full_recovering_mini45 = True
        self.static_full_paused = True
        self.calibration_paused = True
        self._mini45_reconnect_attempts = 0
        self._mini45_reconnect_next_s = time.monotonic()
        self._mini45_reconnect_started_s = time.monotonic()
        self._mini45_reconnect_adapter_started_s = 0.0
        self.stop_auto_force("Mini45 断流，已安全暂停")
        self.static_point_collector = None
        self.buffer.clear()
        self.reset_force_filter(log=False)
        if self.mini45:
            try:
                self.mini45.stop()
            except Exception:
                pass
        self.mini45 = None
        self.last_force_time = 0.0
        self.latest_force_sample = None
        self.mini_btn.setEnabled(False)
        self.static_full_status.setText(
            f"Mini45 数据中断，流程已安全暂停并保留第 {self.sequence_index + 1} 点进度。\n"
            "正在自动重连；重连后将恢复原预补偿并重新采集当前点。"
        )
        self._log(f"{reason}；全静态标定已暂停，开始自动重连 Mini45，原预补偿保持有效")
        self._save_static_full_checkpoint("recovering", reason)
        self._update_static_full_buttons()

    def _update_static_full_mini45_recovery(self) -> None:
        if not self.static_full_recovering_mini45:
            return
        now = time.monotonic()
        if self.mini45 and self.last_force_time > 0.0 and now - self.last_force_time <= 1.0:
            self.static_full_recovering_mini45 = False
            self.static_full_paused = False
            self.calibration_paused = False
            self.mini_btn.setEnabled(True)
            self.mini_btn.setText("断开 Mini45")
            self.mini_status.setText("Mini45 状态：自动重连成功，数据正常")
            compensation = "已恢复原预补偿" if self.mini45_precomp_enabled else "当前未启用软件预补偿"
            self._log(
                f"Mini45 自动重连成功（第 {self._mini45_reconnect_attempts} 次尝试），{compensation}；"
                f"从第 {self.sequence_index + 1} 点重新稳定并继续"
            )
            self._save_static_full_checkpoint("running", "Mini45 自动重连成功")
            self._update_static_full_buttons()
            self.start_next_sequence_target()
            return
        if self.mini45 and now - self._mini45_reconnect_adapter_started_s > 5.0:
            try:
                self.mini45.stop()
            except Exception:
                pass
            self.mini45 = None
            self._mini45_reconnect_next_s = now + 2.0
        if self.mini45 or now < self._mini45_reconnect_next_s:
            return
        self._mini45_reconnect_attempts += 1
        try:
            self._start_mini45_reconnect_adapter()
            self._mini45_reconnect_adapter_started_s = now
            self._log(f"Mini45 自动重连：第 {self._mini45_reconnect_attempts} 次尝试已发送 RDT 启动命令")
        except Exception as exc:
            self.mini45 = None
            self._mini45_reconnect_next_s = now + min(10.0, 2.0 + self._mini45_reconnect_attempts)
            self.static_full_status.setText(
                f"Mini45 自动重连第 {self._mini45_reconnect_attempts} 次失败：{exc}\n"
                "流程保持暂停，稍后继续重试；也可点击停止并之后从已有实验继续。"
            )
            self._log(f"Mini45 自动重连失败：{exc}")

    def bias_mini45(self) -> None:
        if self.mini45_precomp_active or self.auto_force_active or self.k_ident_active or self.zero_drift_active or self.training_active:
            QMessageBox.warning(self, "Mini45", "预补偿、自动标定、K 辨识、零点漂移或训练采集过程中不能清零/偏置")
            return
        if self.mini45 and hasattr(self.mini45, "bias"):
            self.mini45.bias()
            if self.mini45_precomp_enabled:
                self._invalidate_mini45_precomp("Mini45 硬件清零后原预补偿已失效")
            self.reset_force_filter(log=False)
            self.buffer.clear()
            self._clear_force_plot()
            self._log("Mini45 清零/偏置命令已发送，已重置上位机滤波、稳定窗口和力曲线")

    def start_mini45_precomp(self) -> None:
        if self.mini45_precomp_active:
            return
        if (
            self.workflow.active
            or (
                self._calibration_active()
                and self.static_full_setup_stage not in {"precomp", "retest_precomp"}
            )
            or self.k_ident_active
        ):
            QMessageBox.warning(self, "Mini45 预补偿", "当前有实验、力控或 K 辨识正在运行")
            return
        now = time.monotonic()
        if not self.mini45 or self.last_force_time <= 0.0 or now - self.last_force_time > 1.0:
            QMessageBox.warning(self, "Mini45 预补偿", "请先连接 Mini45，并确认最近 1 秒内有实时数据")
            return
        try:
            self.current_force_frame_mapping().validate()
        except ValueError as exc:
            QMessageBox.warning(self, "Mini45 预补偿", f"请先修正传感器坐标映射：{exc}")
            return

        self.mini45_precomp_samples.clear()
        self.mini45_precomp_active = True
        self.mini45_precomp_enabled = False
        self.mini45_precomp_bias = dict(ZERO_BIAS)
        self.mini45_precomp_start_monotonic_s = now
        self.mini45_precomp_quality = "none"
        self.reset_force_filter(log=False)
        self.buffer.clear()
        self._clear_force_plot()
        self.clear_force_control_k()
        self._update_force_frame_mapping_lock()
        self.mini45_precomp_btn.setEnabled(False)
        self.mini45_precomp_status.setText(f"预补偿：测量中 0.0/{self.mini45_precomp_duration_s:.0f} s")
        self._update_calibration_buttons()
        self._log(
            "开始 Mini45 预补偿 60s：请确保 Mini45 空载、加载头不接触传感器、机械完全静止。"
            "本流程不会补偿 ESP32 电容数据。"
        )

    def _mini45_precomp_summary_path(self) -> Path:
        if self.recorder:
            output_dir = self.recorder.output_dir
        else:
            output_dir = Path(self.output_dir.text()) / "precomp"
        return output_dir / f"mini45_precomp_summary_{time.strftime('%Y%m%d_%H%M%S')}.csv"

    def _mini45_precomp_bias_text(self, bias: dict[str, float]) -> str:
        return ", ".join(f"{field.upper()}={bias[field]:+.6g}" for field in ("fx", "fy", "fz", "mx", "my", "mz"))

    def finish_mini45_precomp(self, forced_failure_reason: str = "") -> None:
        if not self.mini45_precomp_active or self.mini45_precomp_start_monotonic_s is None:
            return
        end_s = time.monotonic()
        summary = compute_precomp_summary(
            self.mini45_precomp_samples,
            measurement_start_s=self.mini45_precomp_start_monotonic_s,
            measurement_end_s=end_s,
            timestamp=utc_timestamp(),
        )
        if forced_failure_reason:
            summary["quality"] = "fail"
            existing_reason = str(summary.get("reject_reason", ""))
            summary["reject_reason"] = "；".join(reason for reason in (existing_reason, forced_failure_reason) if reason)

        self.mini45_precomp_active = False
        self.mini45_precomp_start_monotonic_s = None
        self.mini45_precomp_quality = str(summary["quality"])
        self.mini45_precomp_enabled = self.mini45_precomp_quality != "fail"
        if self.mini45_precomp_enabled:
            self.mini45_precomp_bias = {
                field: float(summary[f"bias_{field}"])
                for field in ("fx", "fy", "fz", "mx", "my", "mz")
            }
        else:
            self.mini45_precomp_bias = dict(ZERO_BIAS)

        self.reset_force_filter(log=False)
        self.buffer.clear()
        self._clear_force_plot()
        self.clear_force_control_k()
        self._update_force_frame_mapping_lock()
        self.mini45_precomp_btn.setEnabled(True)
        self._update_calibration_buttons()

        try:
            summary_path = self._mini45_precomp_summary_path()
            save_precomp_summary(summary, summary_path)
            self._log(f"Mini45 预补偿 summary 已保存：{summary_path}")
        except Exception as exc:
            self._log(f"Mini45 预补偿 summary 保存失败（不影响质量判定）：{exc}")

        if self.mini45_precomp_enabled:
            bias_text = self._mini45_precomp_bias_text(self.mini45_precomp_bias)
            quality_text = "通过" if self.mini45_precomp_quality == "pass" else "警告"
            self.mini45_precomp_status.setText(f"预补偿：已启用（{quality_text}）\n{bias_text}")
            self._log(f"Mini45 预补偿已启用（{quality_text}）：{bias_text}")
            if summary.get("reject_reason"):
                self._log(f"Mini45 预补偿质量提示：{summary['reject_reason']}")
        else:
            reason = str(summary.get("reject_reason") or "质量检查未通过")
            self.mini45_precomp_status.setText(f"预补偿：失败，{reason}")
            self._log(f"Mini45 预补偿失败：{reason}。请保持空载静止后重新测量。")

        if self.static_full_setup_stage == "precomp":
            if not self.mini45_precomp_enabled:
                self._abort_static_full_setup("Mini45 零点预补偿质量检查未通过")
                return
            self.static_full_setup_stage = "k_identification"
            self.static_full_status.setText("自动准备 2/3：正在自动辨识 K")
            self._update_static_full_buttons()
            self._log("Mini45 零点预补偿完成，开始自动辨识 K")
            self.start_k_identification()
            if not self.k_ident_active:
                self._abort_static_full_setup("K 自动辨识未能启动")

        if self.static_full_setup_stage == "retest_precomp":
            if not self.mini45_precomp_enabled:
                self.stop_static_full_retest("Mini45 零点预补偿质量检查未通过")
                return
            if self.recorder:
                self.recorder.update_static_full_retest_manifest(
                    mini45_precomp_source="remeasure",
                    mini45_precomp_quality=self.mini45_precomp_quality,
                    mini45_precomp_bias=dict(self.mini45_precomp_bias),
                )
            if self.static_full_retest_remeasure_k:
                self._start_static_full_retest_k_identification()
            else:
                self._start_static_full_retest_profile_wait()

    def _invalidate_mini45_precomp(self, reason: str) -> None:
        self.mini45_precomp_active = False
        self.mini45_precomp_enabled = False
        self.mini45_precomp_bias = dict(ZERO_BIAS)
        self.mini45_precomp_start_monotonic_s = None
        self.mini45_precomp_quality = "fail"
        self.reset_force_filter(log=False)
        self.buffer.clear()
        if hasattr(self, "mini45_precomp_btn"):
            self.mini45_precomp_btn.setEnabled(True)
            self.mini45_precomp_status.setText(f"预补偿：失败，{reason}")
        self._update_force_frame_mapping_lock()
        self._log(reason)

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
        self.force_plot_dirty = False
        self.pending_force_plot_sample = None
        self.last_force_plot_update_s = 0.0

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
        if self.mini45_precomp_enabled:
            self._invalidate_mini45_precomp("传感器坐标映射已修改，Mini45 预补偿已失效，请重新测量 60s")
        if self.force_control_result:
            self.clear_force_control_k()
            self._log("坐标映射已修改，当前 K 已清除，需要重新自动辨识")

    def _set_force_frame_mapping_enabled(self, enabled: bool) -> None:
        for combo in list(self.frame_sign_combos.values()) + list(self.frame_axis_combos.values()):
            combo.setEnabled(enabled)

    def _update_force_frame_mapping_lock(self) -> None:
        locked = bool(self.recorder or self.k_ident_active or self.force_control_result or self.mini45_precomp_active)
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
        self._update_calibration_buttons()

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
        self._update_calibration_buttons()
        if self.workflow.active and self.workflow.stage == "k_identification":
            self.abort_full_workflow(f"K 辨识失败：{reason}")
        elif self.static_full_setup_stage == "k_identification":
            self._abort_static_full_setup(f"K 辨识失败：{reason}")

        if self.static_full_setup_stage == "retest_k_identification":
            self.stop_static_full_retest(f"K 辨识失败：{reason}")

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
            if self.workflow.active and self.workflow.stage == "k_identification":
                self._advance_workflow()
        else:
            self._log(f"K 辨识无效：{result.reject_reason}")
            QMessageBox.warning(self, "K 辨识", f"K 辨识无效：{result.reject_reason}")
            if self.workflow.active and self.workflow.stage == "k_identification":
                self.abort_full_workflow(f"K 辨识无效：{result.reject_reason}")
        if self.static_full_setup_stage == "k_identification":
            if result.valid:
                self._log("K 自动辨识完成，准备切换静态电容采集配置")
                self._save_static_full_checkpoint("setup_complete", "预补偿和 K 辨识完成")
                self._start_static_full_profile_wait()
            else:
                self._abort_static_full_setup(f"K 辨识无效：{result.reject_reason}")
        self._update_calibration_buttons()

        if self.static_full_setup_stage == "retest_k_identification":
            if result.valid:
                if self.recorder:
                    self.recorder.update_static_full_retest_manifest(force_control_source="remeasure")
                self._start_static_full_retest_profile_wait()
            else:
                self.stop_static_full_retest(f"K 辨识无效：{result.reject_reason}")

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

    def _write_workflow_event(
        self,
        event: str,
        status: str = "",
        note: str = "",
        profile_name: str | None = None,
    ) -> None:
        if not self.recorder:
            return
        if self.static_full_retest_active:
            return
        profile_label = profile_name or self.current_cap_profile
        profile = None
        try:
            profile = get_acquisition_profile(profile_label) if profile_label else None
        except ValueError:
            profile = None
        self.recorder.write_workflow_event(
            {
                "event": event,
                "stage": self.workflow.stage,
                "status": status,
                "cap_profile": profile_label,
                "cnt": profile.cnt if profile else "",
                "cavg": profile.cavg if profile else "",
                "requested_hz": profile.nominal_hz if profile else "",
                "effective_hz": self.current_cap_effective_hz or "",
                "target_index": self.sequence_index if self.workflow.stage == "static_sequence" else self.training_target_index,
                "retry_count": self.static_point_collector.retry_count if self.static_point_collector else self.profile_switch_retry,
                "note": note,
            }
        )

    def _request_cap_profile(self, name: str) -> None:
        if not self.esp32:
            raise RuntimeError("ESP32 未连接")
        profile = get_acquisition_profile(name)
        self.profile_switch_target = profile.name
        self.profile_switch_started_s = time.monotonic()
        self.current_cap_effective_hz = 0.0
        self.esp32.set_profile(profile.name)
        self._write_workflow_event("profile_switch_start", "running", profile.name, profile.name)
        self._log(f"正在切换 MC1081 配置：{profile.name}")

    def _profile_is_ready(self, name: str) -> bool:
        if self.current_cap_profile != name or self.current_cap_effective_hz <= 0.0:
            return False
        nominal = get_acquisition_profile(name).nominal_hz
        return abs(self.current_cap_effective_hz - nominal) / nominal <= 0.20

    def start_full_workflow(self) -> None:
        if self.workflow.active or self._calibration_active() or self.k_ident_active:
            QMessageBox.warning(self, "完整自动实验", "当前已有实验流程正在运行")
            return
        if not self.esp32 or not self.mini45 or not self.motion:
            QMessageBox.warning(self, "完整自动实验", "请先连接 ESP32、Mini45 和 Arduino")
            return
        if self._combo_value(self.esp_mode) != "stream":
            QMessageBox.warning(self, "完整自动实验", "完整自动实验要求 ESP32 使用流式采集模式，以获得唯一 CAP 序号和采集配置字段")
            return
        if not self.latest_force_sample or not self._current_force_safe():
            QMessageBox.warning(self, "完整自动实验", "当前没有安全有效的 Mini45 传感器坐标力数据")
            return
        try:
            self.current_force_frame_mapping().validate()
        except ValueError as exc:
            QMessageBox.warning(self, "完整自动实验", f"传感器坐标映射无效：{exc}")
            return
        if self.recorder:
            QMessageBox.warning(self, "完整自动实验", "请先结束当前实验批次，完整流程会自动创建新批次")
            return
        self.tol_fx.setValue(0.05)
        self.tol_fy.setValue(0.05)
        self.tol_fz.setValue(0.08)
        self.stable_window.setValue(5.0)
        self.toggle_recording()
        if not self.recorder:
            return
        seed = random.SystemRandom().randint(1, 2_147_483_647)
        self.workflow.start(seed)
        self.workflow_stage_started = False
        self.workflow_training_trajectory_index = 0
        self.workflow_random_targets = {}
        self._write_workflow_event("workflow_start", "running", f"random_seed={seed}")
        self._update_workflow_ui()
        self._start_workflow_stage()

    def pause_full_workflow(self) -> None:
        if not self.workflow.active:
            return
        if self.k_ident_active:
            QMessageBox.warning(self, "完整自动实验", "K 辨识扰动过程中不能暂停；如需中断请点击停止/急停")
            return
        self.workflow.paused = True
        self.workflow_pause_started_s = time.monotonic()
        self.pause_calibration()
        self._write_workflow_event("workflow_pause", "paused")
        self._update_workflow_ui()

    def resume_full_workflow(self) -> None:
        if not self.workflow.active:
            return
        if self.force_zero_active:
            return
        now = time.monotonic()
        paused_s = max(0.0, now - self.workflow_pause_started_s)
        if self.zero_drift_active:
            self.zero_drift_start_s += paused_s
        if self.static_point_collector:
            self.static_point_collector.begin(now)
        self.workflow_pause_started_s = 0.0
        self.workflow.paused = False
        self.resume_calibration()
        self._write_workflow_event("workflow_resume", "running")
        self._update_workflow_ui()

    def abort_full_workflow(self, reason: str) -> None:
        if not self.workflow.active:
            return
        self._write_workflow_event("workflow_abort", "failed", reason)
        self.workflow.fail(reason)
        self.stop_calibration(reason)
        try:
            if self.motion:
                self.motion.stop_all()
        except Exception:
            pass
        if self.recorder:
            self.toggle_recording()
        self._update_workflow_ui()

    def _advance_workflow(self, event: str = "stage_complete") -> None:
        if not self.workflow.active:
            return
        self._write_workflow_event(event, "complete")
        self.workflow.advance()
        self.workflow_stage_started = False
        self._update_workflow_ui()
        self._start_workflow_stage()

    def _start_workflow_stage(self) -> None:
        if not self.workflow.active or self.workflow.paused or self.workflow_stage_started:
            return
        self.workflow_stage_started = True
        stage = self.workflow.stage
        self._write_workflow_event("stage_start", "running")
        if stage == "profile_static":
            self.profile_switch_retry = 0
            self._request_cap_profile(STATIC_PRECISION.name)
        elif stage == "zero_drift":
            self.calibration_mode = "zero"
            # 硬件 CMD_BIAS：发送 ATI 清零命令，NETBA 内部归零六轴输出
            if self.mini45 and hasattr(self.mini45, "bias"):
                self.mini45.bias()
                self._log("已发送 Mini45 硬件清零命令（CMD_BIAS），零漂采集将以归零后数据记录")
            self.start_zero_drift()
        elif stage == "k_identification":
            self.start_k_identification()
            if not self.k_ident_active:
                self.abort_full_workflow("K 辨识未能启动")
        elif stage == "static_sequence":
            self.calibration_mode = "sequence"
            self.sequence_targets = generate_three_axis_sequence(
                fz_max_force=self.seq_fz_max.value(),
                fz_step=self.seq_fz_step.value(),
                shear_max_force=self.seq_shear_max.value(),
                shear_step=self.seq_shear_step.value(),
                target_fz=0.0,
                shear_direction_mode="both",
                cycles=self.seq_cycles.value(),
            )
            self.sequence_index = 0
            self.start_next_sequence_target()
        elif stage == "profile_balanced":
            if not self.workflow_balanced_enabled.isChecked():
                self._advance_workflow("stage_skipped")
            else:
                self.profile_switch_retry = 0
                self._request_cap_profile(TRAINING_BALANCED.name)
        elif stage == "training_balanced":
            if not self.workflow_balanced_enabled.isChecked():
                self._advance_workflow("stage_skipped")
            else:
                self.training_profile = TRAINING_BALANCED.name
                self.workflow_training_trajectory_index = 0
                self._start_workflow_training_trajectory()
        elif stage == "profile_fast":
            if not self.workflow_fast_enabled.isChecked():
                self._advance_workflow("stage_skipped")
            else:
                self.profile_switch_retry = 0
                self._request_cap_profile(TRAINING_FAST.name)
        elif stage == "training_fast":
            if not self.workflow_fast_enabled.isChecked():
                self._advance_workflow("stage_skipped")
            else:
                self.training_profile = TRAINING_FAST.name
                self.workflow_training_trajectory_index = 0
                self._start_workflow_training_trajectory()
        elif stage == "return_zero":
            self.calibration_mode = "workflow_return"
            self.active_target = CalibrationTarget("combined", "none", "unloading", 0.0, 0.0, 0.0)
            if not self.start_auto_force():
                self.abort_full_workflow("自动回零启动失败")
        elif stage == "finish":
            self._finish_full_workflow()

    def _start_workflow_training_trajectory(self) -> None:
        if self.workflow_training_trajectory_index >= len(self.workflow_training_trajectories):
            self._advance_workflow()
            return
        trajectory_type = self.workflow_training_trajectories[self.workflow_training_trajectory_index]
        rng = random.Random(self.workflow.random_seed)
        targets = generate_training_trajectory(
            fz_levels=list(range(1, 10)),
            shear_max=self.training_shear_max.value(),
            trajectory_type=trajectory_type,
            target_step_n=self.training_target_step.value(),
            random_points=self.training_random_points.value(),
            rng=rng,
        )
        self.start_training_collection(targets=targets, trajectory_type=trajectory_type, profile=self.training_profile)

    def _finish_full_workflow(self) -> None:
        self._write_workflow_event("workflow_complete", "complete")
        self.workflow.active = False
        self.workflow.stage = "已完成"
        if self.recorder:
            self.toggle_recording()
        self._update_workflow_ui()
        QMessageBox.information(self, "完整自动实验", "完整自动实验已完成，数据文件已关闭")

    def _update_full_workflow(self) -> None:
        if not self.workflow.active or self.workflow.paused:
            return
        stage = self.workflow.stage
        if stage.startswith("profile_") and self.profile_switch_target:
            if self._profile_is_ready(self.profile_switch_target):
                self._write_workflow_event("profile_switch_complete", "complete", self.profile_switch_target)
                self.profile_switch_target = ""
                self._advance_workflow()
                return
            timeout = 15.0 if self.profile_switch_target == STATIC_PRECISION.name else 7.0
            if time.monotonic() - self.profile_switch_started_s >= timeout:
                if self.profile_switch_retry < 1:
                    self.profile_switch_retry += 1
                    self._request_cap_profile(self.profile_switch_target)
                else:
                    self.abort_full_workflow(f"MC1081 配置切换或频率验证失败：{self.profile_switch_target}")
        self._update_workflow_ui()

    def _update_workflow_ui(self) -> None:
        if not hasattr(self, "workflow_start_btn"):
            return
        active = self.workflow.active
        self.workflow_start_btn.setEnabled(not active)
        self.workflow_pause_btn.setEnabled(active and not self.workflow.paused and not self.force_zero_active)
        self.workflow_resume_btn.setEnabled(active and self.workflow.paused and not self.force_zero_active)
        self.workflow_stop_btn.setEnabled(active)
        self.workflow_balanced_enabled.setEnabled(not active)
        self.workflow_fast_enabled.setEnabled(not active)
        for widget in (
            getattr(self, "experiment_mode", None),
            getattr(self, "load_axis", None),
            getattr(self, "experiment_id", None),
            getattr(self, "note", None),
            getattr(self, "seq_fz_max", None),
            getattr(self, "seq_fz_step", None),
            getattr(self, "seq_shear_max", None),
            getattr(self, "seq_shear_step", None),
            getattr(self, "seq_cycles", None),
            getattr(self, "seq_shear_direction", None),
            getattr(self, "zero_duration_s", None),
            getattr(self, "target_fx", None),
            getattr(self, "target_fy", None),
            getattr(self, "target_fz", None),
            getattr(self, "tol_fx", None),
            getattr(self, "tol_fy", None),
            getattr(self, "tol_fz", None),
            getattr(self, "stable_window", None),
            getattr(self, "training_fz_levels", None),
            getattr(self, "training_trajectory_type", None),
            getattr(self, "training_shear_max", None),
            getattr(self, "training_target_step", None),
            getattr(self, "training_arrival_window", None),
            getattr(self, "training_max_wait_s", None),
            getattr(self, "training_random_points", None),
            getattr(self, "k_delta_x", None),
            getattr(self, "k_delta_y", None),
            getattr(self, "k_delta_z", None),
            getattr(self, "k_wait_s", None),
            getattr(self, "k_sample_s", None),
            getattr(self, "k_condition_limit", None),
            getattr(self, "auto_step_mm", None),
            getattr(self, "auto_interval_s", None),
            getattr(self, "auto_speed_mm_s", None),
            getattr(self, "auto_min_effective_step_mm", None),
            getattr(self, "control_style", None),
            getattr(self, "k_ident_btn", None),
            getattr(self, "k_clear_btn", None),
            getattr(self, "cal_start_btn", None),
            getattr(self, "record_btn", None),
            getattr(self, "marker_btn", None),
        ):
            if widget is not None:
                widget.setEnabled(not active)
        if self.workflow.failure_reason and not self.workflow.active:
            stage = f"失败：{self.workflow.failure_reason}"
        else:
            stage = self.workflow.stage or "未运行"
        self.workflow_status_label.setText(f"完整流程：{self.workflow.progress_text}，阶段 {stage}")
        profile_text = self.current_cap_profile or "未确认"
        hz_text = f"{self.current_cap_effective_hz:.2f} Hz" if self.current_cap_effective_hz > 0 else "-- Hz"
        try:
            profile = get_acquisition_profile(self.current_cap_profile)
            profile_text = f"{profile.name}，CNT={profile.cnt}，CAVG={profile.cavg}"
        except ValueError:
            pass
        self.workflow_profile_label.setText(f"MC1081 配置：{profile_text}，实际 {hz_text}")
        collector = self.static_point_collector
        if collector:
            if collector.preserving_progress:
                point_state = f"保留进度等待力恢复，已保留 {len(collector.cap_samples)} 帧"
            elif collector.collection_paused:
                point_state = f"越界暂停 {collector.outside_elapsed_s:.1f}/{collector.out_of_window_grace_s:.1f} s"
            elif collector.collecting:
                point_state = "采集中"
            else:
                point_state = f"稳定保持 {collector.stable_elapsed_s:.1f}/{collector.stable_hold_s:.1f} s"
            self.workflow_point_label.setText(
                f"静态点 {self.sequence_index + 1}/{len(self.sequence_targets)}，重试 {collector.retry_count}/2，"
                f"{point_state}，电容样本 {len(collector.cap_samples)}/{collector.required_cap_samples}"
            )
        else:
            self.workflow_point_label.setText("静态点：--，稳定保持 0.0/5.0 s，电容样本 0/45")
        self.workflow_count_label.setText(
            f"完成 {self.workflow.completed_static_points}，无效 {self.workflow.invalid_static_points}，"
            f"训练跳过 {self.workflow.skipped_training_targets}"
        )

    def start_calibration(self) -> None:
        mode = self._combo_value(self.experiment_mode)
        if self._calibration_active() or self.k_ident_active:
            QMessageBox.information(self, "标定状态", "当前已有子实验正在运行，请先停止或等待完成")
            return
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
        self._update_calibration_buttons()
        if mode == "zero":
            self.start_zero_drift()
            return
        if mode == "single":
            if self._combo_value(self.load_axis) == "all":
                QMessageBox.warning(self, "单目标点标定", "三轴自动只用于静态正反程标定，请选择 Fx、Fy 或 Fz")
                self.calibration_mode = ""
                self._update_calibration_buttons()
                return
            self.current_cycle_id = "cycle_001"
            self.active_target = CalibrationTarget(
                axis=self._combo_value(self.load_axis),
                direction=self._combo_value(self.direction),
                branch=self._combo_value(self.branch),
                target_fx=self.target_fx.value(),
                target_fy=self.target_fy.value(),
                target_fz=self.target_fz.value(),
            )
            if not self.start_auto_force():
                self.stop_calibration("启动失败")
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

    # ── 全静态标定 ────────────────────────────────────────────

    def _static_full_busy(self) -> bool:
        return bool(
            self.static_full_active
            or self.static_full_retest_active
            or self._static_full_profile_wait
            or self.static_full_setup_stage
        )

    def _update_static_full_buttons(self) -> None:
        active = self._static_full_busy()
        paused = self.static_full_paused
        self.static_full_start_btn.setEnabled(not active)
        self.static_full_resume_file_btn.setEnabled(not active)
        self.static_full_pause_btn.setEnabled(
            self.static_full_active
            and not paused
            and not self.static_full_returning_zero
            and not self.static_full_recovering_mini45
            and not self.static_full_recovering_esp32
            and not self.force_zero_active
        )
        self.static_full_resume_btn.setEnabled(
            self.static_full_active
            and paused
            and not self.static_full_recovering_mini45
            and not self.static_full_recovering_esp32
            and not self.force_zero_active
        )
        self.static_full_stop_btn.setEnabled(active)
        for w in (self.static_full_fz_max, self.static_full_fz_step,
                  self.static_full_preload_levels,
                  self.static_full_shear_max, self.static_full_shear_step,
                  self.static_full_angles, self.static_full_cap_samples):
            w.setEnabled(not active)
        if hasattr(self, "static_full_flow_checks"):
            for check in self.static_full_flow_checks.values():
                check.setEnabled(not active)
            for flow, cycles in self.static_full_flow_cycles.items():
                cycles.setEnabled(not active and self.static_full_flow_checks[flow].isChecked())
        if hasattr(self, "static_full_retest_start_btn"):
            loaded = bool(self.static_full_retest_source_targets)
            self.static_full_retest_load_btn.setEnabled(not active)
            self.static_full_retest_start_btn.setEnabled(not active and loaded)
            self.static_full_retest_pause_btn.setEnabled(
                self.static_full_retest_active
                and self.static_full_active
                and not paused
                and not self.static_full_returning_zero
                and not self.force_zero_active
            )
            self.static_full_retest_resume_btn.setEnabled(
                self.static_full_retest_active
                and self.static_full_active
                and paused
                and not self.force_zero_active
            )
            self.static_full_retest_stop_btn.setEnabled(self.static_full_retest_active)
            for widget in (
                *self.static_full_retest_axis_checks.values(),
                *self.static_full_retest_branch_checks.values(),
                *self.static_full_retest_direction_checks.values(),
                self.static_full_retest_cycles,
                self.static_full_retest_preloads,
                self.static_full_retest_shear_levels,
                self.static_full_retest_angles,
                self.static_full_retest_reprecomp,
                self.static_full_retest_rek,
            ):
                widget.setEnabled(not active)

    def _devices_ready(self, *, require_k: bool = True) -> bool:
        if not self.esp32:
            QMessageBox.warning(self, "全静态标定", "请先连接 ESP32 电容采集串口")
            return False
        if not self.mini45:
            QMessageBox.warning(self, "全静态标定", "请先连接 Mini45")
            return False
        if not self.motion:
            QMessageBox.warning(self, "全静态标定", "请先连接 Arduino 电机控制串口")
            return False
        if require_k and (not self.force_control_result or not self.force_control_result.valid):
            QMessageBox.warning(self, "全静态标定", "请先在「实验配置」页完成 K 自动辨识")
            return False
        if self._combo_value(self.esp_mode) != "stream":
            QMessageBox.warning(self, "全静态标定", "全静态标定要求 ESP32 使用流式采集模式")
            return False
        now = time.monotonic()
        if not self.latest_force_sample or self.last_force_time <= 0.0 or now - self.last_force_time > 1.0:
            QMessageBox.warning(self, "全静态标定", "Mini45 最近 1 秒内没有有效力数据")
            return False
        if not self._current_force_safe():
            QMessageBox.warning(self, "全静态标定", "当前力或力矩超过安全限值")
            return False
        try:
            self.current_force_frame_mapping().validate()
        except ValueError as exc:
            QMessageBox.warning(self, "全静态标定", f"传感器坐标映射无效：{exc}")
            return False
        return True

    def _checked_values_or_none(self, checks: dict[str, QCheckBox]) -> set[str] | None:
        selected = {value for value, check in checks.items() if check.isChecked()}
        if not selected or len(selected) == len(checks):
            return None
        return selected

    def _optional_force_levels(self, text: str) -> set[float] | None:
        text = text.strip()
        return set(parse_force_levels(text)) if text else None

    def _optional_angles(self, text: str) -> set[float] | None:
        text = text.strip()
        return set(parse_angles_deg(text)) if text else None

    def _optional_cycles(self, text: str) -> set[int] | None:
        text = text.strip()
        if not text:
            return None
        values = {int(value) for value in parse_force_levels(text)}
        if any(value < 1 for value in values):
            raise ValueError("cycle index must be >= 1")
        return values

    def _static_full_retest_filters(self) -> dict:
        return {
            "axes": self._checked_values_or_none(self.static_full_retest_axis_checks),
            "branches": self._checked_values_or_none(self.static_full_retest_branch_checks),
            "directions": self._checked_values_or_none(self.static_full_retest_direction_checks),
            "cycles": self._optional_cycles(self.static_full_retest_cycles.text()),
            "preload_levels": self._optional_force_levels(self.static_full_retest_preloads.text()),
            "shear_levels": self._optional_force_levels(self.static_full_retest_shear_levels.text()),
            "diagonal_angles_deg": self._optional_angles(self.static_full_retest_angles.text()),
        }

    def _static_full_retest_filter_manifest(self, filters: dict) -> dict:
        return {
            key: sorted(value) if isinstance(value, set) else value
            for key, value in filters.items()
        }

    def _filtered_static_full_retest_targets(self) -> list[CalibrationTarget]:
        filters = self._static_full_retest_filters()
        self.static_full_retest_filter_spec = self._static_full_retest_filter_manifest(filters)
        existing = filter_static_full_targets(self.static_full_retest_source_targets, **filters)
        generated = generate_missing_static_full_diagonal_targets(
            self.static_full_retest_source_targets,
            **filters,
        )
        return existing + generated

    def _update_static_full_retest_preview(self, *args: object) -> None:
        if not hasattr(self, "static_full_retest_status"):
            return
        if not self.static_full_retest_source_targets:
            self.static_full_retest_status.setText("未加载补测批次")
            self._update_static_full_buttons()
            return
        try:
            targets = self._filtered_static_full_retest_targets()
        except Exception as exc:
            self.static_full_retest_status.setText(f"筛选条件无效：{exc}")
            self.static_full_retest_status.setStyleSheet("color: #C62828;")
            self._update_static_full_buttons()
            return
        self.static_full_retest_status.setStyleSheet("")
        fz_n = sum(1 for target in targets if target.axis == "Fz")
        fx_n = sum(1 for target in targets if target.axis == "Fx")
        fy_n = sum(1 for target in targets if target.axis == "Fy")
        diag_n = sum(1 for target in targets if target.axis == "combined")
        self.static_full_retest_status.setText(
            f"已加载 {len(self.static_full_retest_source_targets)} 个原始目标；当前筛选 {len(targets)} 个补测点 "
            f"(Fz {fz_n} / Fx {fx_n} / Fy {fy_n} / 斜向 {diag_n})"
        )
        self._update_static_full_buttons()

    def _apply_force_frame_mapping_row(self, row: dict | None) -> None:
        if not row:
            return
        for sensor_axis in ("Fx", "Fy", "Fz"):
            source = row.get(f"sensor_{sensor_axis}_from")
            sign = row.get(f"sensor_{sensor_axis}_sign")
            if source not in (None, ""):
                self._set_combo_by_data(self.frame_axis_combos[sensor_axis], str(source))
            if sign not in (None, ""):
                self._set_combo_by_data(self.frame_sign_combos[sensor_axis], str(sign))
        self.on_force_frame_mapping_changed()

    def _apply_static_full_parameters(self, saved_parameters: dict | None) -> None:
        if not isinstance(saved_parameters, dict):
            return
        self.static_full_fz_max.setValue(float(saved_parameters["fz_max"]))
        self.static_full_fz_step.setValue(float(saved_parameters["fz_step"]))
        self.static_full_preload_levels.setText(
            ",".join(f"{float(value):g}" for value in saved_parameters["preload_levels"])
        )
        self.static_full_shear_max.setValue(float(saved_parameters["shear_max"]))
        self.static_full_shear_step.setValue(float(saved_parameters["shear_step"]))
        self.static_full_angles.setText(
            ",".join(f"{float(value):g}" for value in saved_parameters["angles_deg"])
        )
        self.static_full_cap_samples.setValue(int(saved_parameters.get("required_cap_samples", 45)))
        legacy_cycles = int(saved_parameters.get("cycles", 3))
        enabled_flows = set(saved_parameters.get("enabled_flows") or STATIC_FULL_FLOWS)
        flow_cycles = saved_parameters.get("flow_cycles") or {}
        if hasattr(self, "static_full_flow_checks"):
            for flow in STATIC_FULL_FLOWS:
                self.static_full_flow_checks[flow].setChecked(flow in enabled_flows)
                self.static_full_flow_cycles[flow].setValue(int(flow_cycles.get(flow, legacy_cycles)))
            self._update_static_full_flow_controls()

    def _load_static_full_batch_document(self, folder: Path) -> tuple[dict, list[CalibrationTarget]]:
        if checkpoint_path(folder).exists():
            document = load_checkpoint(folder)
            targets = [CalibrationTarget(**row) for row in document["targets"]]
            return document, targets
        parameters = self._static_full_parameters()
        targets = self._static_full_targets_from_parameters(parameters)
        completed = legacy_completed_point_count(folder)
        return {
            "status": "legacy",
            "parameters": parameters,
            "targets": [asdict(target) for target in targets],
            "sequence_index": completed,
            "completed_points": completed,
            "invalid_points": 0,
            "marker_id": legacy_last_marker_id(folder),
            "force_frame_mapping": load_last_force_mapping(folder),
            "force_control_result": None,
            "mini45_precomp": None,
        }, targets

    def load_static_full_retest_folder(self) -> None:
        if self._static_full_busy() or self.workflow.active or self.k_ident_active or self._calibration_active():
            QMessageBox.warning(self, "全静态补测", "当前已有实验或标定流程正在运行")
            return
        folder_text = QFileDialog.getExistingDirectory(self, "选择已有全静态实验目录", self.output_dir.text())
        if not folder_text:
            return
        folder = Path(folder_text)
        try:
            document, targets = self._load_static_full_batch_document(folder)
            safety = SafetySettings()
            validate_force_targets(targets, (safety.fx_abs_max_n, safety.fy_abs_max_n, safety.fz_abs_max_n))
            self._apply_static_full_parameters(document.get("parameters"))
            self._apply_force_frame_mapping_row(document.get("force_frame_mapping"))
        except Exception as exc:
            QMessageBox.warning(self, "全静态补测", f"无法读取已有实验：{exc}")
            return

        k_payload = document.get("force_control_result")
        try:
            restored_k = KIdentificationResult(**k_payload) if isinstance(k_payload, dict) else load_last_valid_k_result(folder)
        except Exception:
            restored_k = None
        if restored_k and restored_k.valid:
            self.force_control_result = restored_k
            self.force_control_state = DecoupledControlState()
            self.update_k_display(restored_k)
            self.static_full_retest_k_source = "checkpoint" if isinstance(k_payload, dict) else "force_control_k.csv"
        elif not self.static_full_retest_rek.isChecked():
            QMessageBox.warning(self, "全静态补测", "已有实验中没有可复用的有效 K；请勾选重新辨识 K 后再加载")
            return
        else:
            self.clear_force_control_k()
            self.static_full_retest_k_source = "remeasure"

        precomp_payload = document.get("mini45_precomp")
        if isinstance(precomp_payload, dict) and precomp_payload.get("enabled"):
            try:
                restored_bias = {field: float(precomp_payload["bias"][field]) for field in ZERO_BIAS}
                restored_quality = str(precomp_payload.get("quality") or "warning")
                self.static_full_retest_precomp_source = "checkpoint"
            except Exception as exc:
                QMessageBox.warning(self, "全静态补测", f"Mini45 预补偿记录无效：{exc}")
                return
        else:
            legacy_precomp = load_latest_precomp(folder)
            if legacy_precomp:
                restored_bias, restored_quality = legacy_precomp
                self.static_full_retest_precomp_source = "mini45_precomp_summary"
            elif not self.static_full_retest_reprecomp.isChecked():
                QMessageBox.warning(self, "全静态补测", "已有实验中没有可复用的 Mini45 预补偿；请勾选重新做预补偿后再加载")
                return
            else:
                restored_bias, restored_quality = dict(ZERO_BIAS), "remeasure"
                self.static_full_retest_precomp_source = "remeasure"

        if self.static_full_retest_precomp_source != "remeasure":
            self.mini45_precomp_enabled = True
            self.mini45_precomp_active = False
            self.mini45_precomp_bias = dict(restored_bias)
            self.mini45_precomp_quality = restored_quality
            self.mini45_precomp_status.setText(
                f"预补偿：已从补测源批次恢复（{restored_quality}）\n"
                f"{self._mini45_precomp_bias_text(self.mini45_precomp_bias)}"
            )

        self.static_full_retest_folder = folder
        self.static_full_retest_document = document
        self.static_full_retest_source_targets = targets
        self.static_full_retest_source_experiment_id = str(document.get("experiment_id") or self.experiment_id.text().strip() or "exp001")
        self.static_full_retest_folder_text.setText(str(folder))
        self.output_dir.setText(str(folder.parent))
        self.experiment_id.setText(self.static_full_retest_source_experiment_id)
        self._update_force_frame_mapping_lock()
        self._update_static_full_retest_preview()
        self._log(f"已加载全静态补测源批次：{folder}，目标 {len(targets)} 个")

    def start_static_full(self) -> None:
        if self._static_full_busy():
            return
        if self.workflow.active or self._calibration_active() or self.k_ident_active or self.calibration_mode:
            QMessageBox.warning(self, "全静态标定", "当前已有实验、标定或 K 辨识流程正在运行")
            return
        if not self._devices_ready(require_k=False):
            return
        if self.recorder:
            QMessageBox.warning(self, "全静态标定", "请先结束当前实验批次；全静态流程会自动创建并管理独立批次")
            return
        try:
            parameters = self._static_full_parameters()
            targets = self._static_full_targets_from_parameters(parameters)
            safety = SafetySettings()
            validate_force_targets(
                targets,
                (safety.fx_abs_max_n, safety.fy_abs_max_n, safety.fz_abs_max_n),
            )
        except Exception as exc:
            QMessageBox.warning(self, "全静态标定", f"参数错误：{exc}")
            return
        if not targets:
            QMessageBox.warning(self, "全静态标定", "请至少勾选一个测量流程")
            return

        self.toggle_recording()
        if not self.recorder:
            return
        self.static_full_auto_recording = True
        self._static_full_pending_targets = targets
        self.static_full_setup_stage = "precomp"
        self.static_full_status.setText("自动准备 1/3：Mini45 零点预补偿测量中（60 s）")
        self._update_static_full_buttons()
        self._update_calibration_buttons()
        self._log("全静态标定自动准备：已创建实验批次，开始 Mini45 零点预补偿")
        self.start_mini45_precomp()
        if not self.mini45_precomp_active:
            self._abort_static_full_setup("Mini45 零点预补偿未能启动")

    def _static_full_retest_target_rows(self, targets: list[CalibrationTarget]) -> list[dict]:
        rows = []
        for target in targets:
            row = asdict(target)
            row["shear_N"] = static_full_target_shear_n(target)
            angle = static_full_target_angle_deg(target)
            row["angle_deg"] = "" if angle is None else angle
            rows.append(row)
        return rows

    def start_static_full_retest(self) -> None:
        if self._static_full_busy() or self.workflow.active or self.k_ident_active or self._calibration_active():
            QMessageBox.warning(self, "全静态补测", "当前已有实验或标定流程正在运行")
            return
        if not self.static_full_retest_folder or not self.static_full_retest_source_targets:
            QMessageBox.warning(self, "全静态补测", "请先选择已有全静态批次")
            return
        try:
            targets = self._filtered_static_full_retest_targets()
            safety = SafetySettings()
            validate_force_targets(targets, (safety.fx_abs_max_n, safety.fy_abs_max_n, safety.fz_abs_max_n))
        except Exception as exc:
            QMessageBox.warning(self, "全静态补测", f"筛选条件无效：{exc}")
            return
        if not targets:
            QMessageBox.warning(self, "全静态补测", "当前筛选条件没有匹配任何补测点")
            return
        self.static_full_retest_remeasure_precomp = self.static_full_retest_reprecomp.isChecked()
        self.static_full_retest_remeasure_k = self.static_full_retest_rek.isChecked()
        if self.static_full_retest_remeasure_precomp and not self.static_full_retest_remeasure_k:
            self.static_full_retest_remeasure_k = True
            self.static_full_retest_rek.setChecked(True)
        if not self._devices_ready(require_k=not self.static_full_retest_remeasure_k):
            return
        if self.recorder:
            QMessageBox.warning(self, "全静态补测", "请先结束当前实验批次；补测会自动打开原批次目录")
            return

        folder = self.static_full_retest_folder
        retest_id = time.strftime("%Y%m%d_%H%M%S")
        manifest = {
            "source_batch_dir": str(folder),
            "source_experiment_id": self.static_full_retest_source_experiment_id,
            "source_checkpoint_status": self.static_full_retest_document.get("status", ""),
            "filters": dict(self.static_full_retest_filter_spec),
            "target_count": len(targets),
            "targets": self._static_full_retest_target_rows(targets),
            "force_frame_mapping": self.current_force_frame_mapping().as_row("", self.static_full_retest_source_experiment_id),
            "force_control_source": "remeasure" if self.static_full_retest_remeasure_k else self.static_full_retest_k_source,
            "mini45_precomp_source": "remeasure" if self.static_full_retest_remeasure_precomp else self.static_full_retest_precomp_source,
            "required_cap_samples": self._static_full_required_cap_samples(),
            "completed_points": 0,
            "invalid_points": 0,
        }
        try:
            recorder = CsvRecorder(folder)
            recorder.start(resume=True)
            recorder.start_static_full_retest(retest_id=retest_id, manifest=manifest)
        except Exception as exc:
            QMessageBox.warning(self, "全静态补测", f"无法打开补测输出文件：{exc}")
            return

        self.recorder = recorder
        self.static_full_auto_recording = True
        self.static_full_retest_active = True
        self.static_full_retest_id = retest_id
        self.static_full_retest_targets = targets
        self._static_full_pending_targets = targets
        self._static_full_pending_resume = None
        self.static_full_points_completed = 0
        self.static_full_points_invalid = 0
        self.marker_id = 0
        self.buffer.clear()
        self.reset_force_filter(log=False)
        self.record_btn.setText("结束实验批次")
        self.record_status.setText(str(folder))
        self._update_force_frame_mapping_lock()
        self._update_static_full_buttons()
        self._update_calibration_buttons()
        self._log(f"全静态补测开始准备：{folder}，补测点 {len(targets)} 个，retest_id={retest_id}")

        if self.static_full_retest_remeasure_precomp:
            self.static_full_setup_stage = "retest_precomp"
            self.static_full_retest_status.setText("补测准备 1/3：Mini45 零点预补偿测量中（60 s）")
            self.static_full_status.setText("补测准备：Mini45 零点预补偿测量中")
            self.start_mini45_precomp()
            if not self.mini45_precomp_active:
                self.stop_static_full_retest("Mini45 零点预补偿未能启动")
            return
        if self.static_full_retest_remeasure_k:
            self._start_static_full_retest_k_identification()
            return
        self._start_static_full_retest_profile_wait()

    def _start_static_full_retest_k_identification(self) -> None:
        self.static_full_setup_stage = "retest_k_identification"
        self.static_full_status.setText("补测准备：正在自动辨识 K")
        self.static_full_retest_status.setText("补测准备：正在自动辨识 K")
        self._update_static_full_buttons()
        self._update_calibration_buttons()
        self.start_k_identification()
        if not self.k_ident_active:
            self.stop_static_full_retest("K 自动辨识未能启动")

    def _start_static_full_retest_profile_wait(self) -> None:
        self.static_full_setup_stage = "retest_profile"
        self._static_full_profile_wait = True
        self._static_full_profile_start_s = time.monotonic()
        self.static_full_status.setText(f"补测准备：等待 {STATIC_PRECISION.name} 配置生效")
        self.static_full_retest_status.setText(f"补测准备：等待 {STATIC_PRECISION.name} 配置生效")
        self._update_static_full_buttons()
        self._update_calibration_buttons()
        try:
            self._request_cap_profile(STATIC_PRECISION.name)
        except Exception as exc:
            self._static_full_profile_wait = False
            self.stop_static_full_retest(f"切换 MC1081 配置失败：{exc}")

    def _static_full_retest_begin(self) -> None:
        targets = list(self._static_full_pending_targets or self.static_full_retest_targets)
        self._static_full_pending_targets = []
        self._static_full_pending_resume = None
        self.static_full_setup_stage = ""
        self.static_full_active = True
        self.static_full_paused = False
        self.static_full_returning_zero = False
        self.static_full_recovering_mini45 = False
        self.static_full_recovering_esp32 = False
        self.static_full_points_completed = 0
        self.static_full_points_invalid = 0
        self.sequence_targets = targets
        self.sequence_index = 0
        self.calibration_mode = "static_full_retest"
        self.calibration_paused = False
        self.static_full_status.setText(f"补测运行中 — 共 {len(targets)} 个补测点")
        self.static_full_retest_status.setText(f"补测运行中 — 共 {len(targets)} 个补测点")
        if self.recorder:
            self.recorder.update_static_full_retest_manifest(status="running", run_started_at=utc_timestamp())
        self._update_static_full_buttons()
        self._update_calibration_buttons()
        self._log(f"全静态补测正式开始：{len(targets)} 个补测点")
        self.start_next_sequence_target()

    def stop_static_full_retest(self, reason: str = "补测停止") -> None:
        if not self.static_full_retest_active:
            return
        self.static_full_setup_stage = ""
        self._static_full_profile_wait = False
        if self.mini45_precomp_active:
            self.finish_mini45_precomp(reason)
        if self.k_ident_active:
            self.abort_k_identification(reason)
        self.stop_auto_force(reason)
        try:
            if self.motion:
                self.motion.stop_all()
        except Exception:
            pass
        if self.recorder:
            self.recorder.finish_static_full_retest(
                status="stopped",
                reason=reason,
                completed_points=self.static_full_points_completed,
                invalid_points=self.static_full_points_invalid,
            )
        self._reset_static_full_retest_runtime()
        self.static_full_status.setText(f"补测已停止：{reason}")
        if hasattr(self, "static_full_retest_status"):
            self.static_full_retest_status.setText(f"补测已停止：{reason}")
        self._close_static_full_recording()
        self._update_static_full_buttons()
        self._update_calibration_buttons()

    def _finish_static_full_retest(self) -> None:
        total = len(self.sequence_targets)
        self.stop_auto_force("")
        if self.recorder:
            self.recorder.finish_static_full_retest(
                status="completed",
                reason="补测点完成并已回零",
                completed_points=self.static_full_points_completed,
                invalid_points=self.static_full_points_invalid,
            )
        self._reset_static_full_retest_runtime()
        text = (
            f"补测完成 — {self.static_full_points_completed} 有效 / "
            f"{self.static_full_points_invalid} 无效 / 共 {total} 点，已自动卸载回零"
        )
        self.static_full_status.setText(text)
        if hasattr(self, "static_full_retest_status"):
            self.static_full_retest_status.setText(text)
        self.cal_status.setText("标定状态：全静态补测完成，已卸载回零")
        self._log(text)
        self._close_static_full_recording()
        self._update_static_full_buttons()
        self._update_calibration_buttons()

    def _reset_static_full_retest_runtime(self) -> None:
        self.static_full_retest_active = False
        self.static_full_retest_targets = []
        self.static_full_retest_id = ""
        self.static_full_active = False
        self.static_full_paused = False
        self.static_full_returning_zero = False
        self.static_full_recovering_mini45 = False
        self.static_full_recovering_esp32 = False
        self.static_full_setup_stage = ""
        self._static_full_profile_wait = False
        self._static_full_pending_targets = []
        self._static_full_pending_resume = None
        self.calibration_mode = ""
        self.calibration_paused = False
        self.static_point_collector = None
        self.sequence_targets = []
        self.sequence_index = 0
        self.active_target = None
        self.static_full_retest_remeasure_precomp = False
        self.static_full_retest_remeasure_k = False

    def _start_static_full_profile_wait(self) -> None:
        self.static_full_setup_stage = "profile"
        self._static_full_profile_wait = True
        self._static_full_profile_start_s = time.monotonic()
        self.static_full_status.setText(f"自动准备 3/3：等待 {STATIC_PRECISION.name} 配置生效")
        self._update_static_full_buttons()
        self._update_calibration_buttons()
        try:
            self._request_cap_profile(STATIC_PRECISION.name)
        except Exception as exc:
            self._static_full_profile_wait = False
            self._abort_static_full_setup(f"切换 MC1081 配置失败：{exc}")
            return
        self._log(f"全静态标定：等待 MC1081 切换到 {STATIC_PRECISION.name} 配置（超时 15s）")

    def _close_static_full_recording(self) -> None:
        if self.static_full_auto_recording and self.recorder:
            self.recorder.stop()
            self.recorder = None
            self.record_btn.setText("开始实验批次")
            self.record_status.setText("未开始实验批次")
            self.clear_force_control_k()
            self._update_force_frame_mapping_lock()
        self.static_full_auto_recording = False

    def _abort_static_full_setup(self, reason: str) -> None:
        self.static_full_setup_stage = ""
        self._static_full_profile_wait = False
        self._static_full_pending_resume = None
        self._static_full_pending_targets = []
        if self.mini45_precomp_active:
            self.finish_mini45_precomp(reason)
        if self.k_ident_active:
            self.abort_k_identification(reason)
        try:
            if self.motion:
                self.motion.stop_all()
        except Exception:
            pass
        self._close_static_full_recording()
        self.static_full_status.setText(f"自动准备失败：{reason}")
        self._log(f"全静态标定自动准备失败：{reason}")
        self._update_static_full_buttons()
        self._update_calibration_buttons()

    def _static_full_parameters(self) -> dict:
        enabled_flows = self._static_full_enabled_flows()
        flow_cycles = self._static_full_flow_cycle_values()
        legacy_cycles = max((flow_cycles[flow] for flow in enabled_flows), default=3)
        shear_flows_enabled = bool(set(enabled_flows) & {"fx", "fy", "diagonal"})
        return {
            "fz_max": self.static_full_fz_max.value(),
            "fz_step": self.static_full_fz_step.value(),
            "preload_levels": parse_force_levels(self.static_full_preload_levels.text()) if shear_flows_enabled else [0.0],
            "shear_max": self.static_full_shear_max.value(),
            "shear_step": self.static_full_shear_step.value(),
            "angles_deg": parse_angles_deg(self.static_full_angles.text()) if "diagonal" in enabled_flows else [],
            "cycles": legacy_cycles,
            "enabled_flows": enabled_flows,
            "flow_cycles": flow_cycles,
            "required_cap_samples": self._static_full_required_cap_samples(),
        }

    def _static_full_targets_from_parameters(self, parameters: dict) -> list[CalibrationTarget]:
        cycles = int(parameters.get("cycles", 3))
        enabled_flows = parameters.get("enabled_flows")
        flow_cycles = parameters.get("flow_cycles")
        return generate_static_full_sequence(
            fz_max=float(parameters["fz_max"]),
            fz_step=float(parameters["fz_step"]),
            preload_levels=[float(value) for value in parameters["preload_levels"]],
            shear_max=float(parameters["shear_max"]),
            shear_step=float(parameters["shear_step"]),
            diagonal_angles_deg=[float(value) for value in parameters["angles_deg"]],
            cycles=cycles,
            enabled_flows=enabled_flows,
            flow_cycles=flow_cycles,
        )

    def _save_static_full_checkpoint(self, status: str, note: str = "") -> None:
        if not self.recorder:
            return
        if self.static_full_retest_active:
            self.recorder.update_static_full_retest_manifest(
                status=status,
                note=note,
                sequence_index=self.sequence_index,
                completed_points=self.static_full_points_completed,
                invalid_points=self.static_full_points_invalid,
            )
            return
        targets = self.sequence_targets or self._static_full_pending_targets
        if not targets:
            return
        mapping = self.current_force_frame_mapping().as_row("", self.experiment_id.text().strip() or "exp001")
        payload = {
            "status": status,
            "note": note,
            "experiment_id": self.experiment_id.text().strip() or "exp001",
            "parameters": self._static_full_parameters(),
            "targets": [asdict(target) for target in targets],
            "sequence_index": self.sequence_index,
            "completed_points": self.static_full_points_completed,
            "invalid_points": self.static_full_points_invalid,
            "marker_id": self.marker_id,
            "force_frame_mapping": mapping,
            "force_control_result": asdict(self.force_control_result) if self.force_control_result else None,
            "mini45_precomp": {
                "enabled": self.mini45_precomp_enabled,
                "quality": self.mini45_precomp_quality,
                "bias": dict(self.mini45_precomp_bias),
            },
        }
        try:
            self.recorder.flush()
            save_checkpoint(self.recorder.output_dir, payload)
        except Exception as exc:
            self._log(f"全静态检查点保存失败：{exc}")

    def _checkpoint_mapping_matches(self, saved: dict | None) -> bool:
        if not saved:
            return True
        current = self.current_force_frame_mapping().as_row("", "")
        keys = (
            "sensor_Fx_from", "sensor_Fx_sign",
            "sensor_Fy_from", "sensor_Fy_sign",
            "sensor_Fz_from", "sensor_Fz_sign",
        )
        return all(str(current.get(key)) == str(saved.get(key)) for key in keys)

    def resume_static_full_from_folder(self) -> None:
        if self._static_full_busy() or self.workflow.active or self.k_ident_active or self._calibration_active():
            QMessageBox.warning(self, "继续全静态标定", "当前已有实验或标定流程正在运行")
            return
        folder_text = QFileDialog.getExistingDirectory(self, "选择已有全静态实验目录", self.output_dir.text())
        if not folder_text:
            return
        if self.recorder:
            try:
                self.recorder.stop()
            except Exception:
                pass
            self.recorder = None
            self.static_full_auto_recording = False
            self.record_btn.setText("开始实验批次")
            self.record_status.setText("未开始实验批次")
            self.clear_force_control_k()
            self._update_force_frame_mapping_lock()
            self._log("继续已有全静态实验前已关闭当前空闲实验批次")
        folder = Path(folder_text)
        try:
            if checkpoint_path(folder).exists():
                document = load_checkpoint(folder)
                targets = [CalibrationTarget(**row) for row in document["targets"]]
            else:
                parameters = self._static_full_parameters()
                targets = self._static_full_targets_from_parameters(parameters)
                completed = legacy_completed_point_count(folder)
                document = {
                    "status": "legacy",
                    "parameters": parameters,
                    "targets": [asdict(target) for target in targets],
                    "sequence_index": completed,
                    "completed_points": completed,
                    "invalid_points": 0,
                    "marker_id": legacy_last_marker_id(folder),
                    "force_frame_mapping": load_last_force_mapping(folder),
                    "force_control_result": None,
                    "mini45_precomp": None,
                }
                QMessageBox.information(
                    self,
                    "继续全静态标定",
                    f"目录中没有新版检查点，将按当前页面参数和已有记录 {completed} 点，"
                    f"从第 {completed + 1} 点继续。请确认页面参数与原实验一致。",
                )
            safety = SafetySettings()
            validate_force_targets(targets, (safety.fx_abs_max_n, safety.fy_abs_max_n, safety.fz_abs_max_n))
            sequence_index = int(document.get("sequence_index", 0))
            if sequence_index < 0 or sequence_index >= len(targets):
                raise ValueError(f"检查点进度 {sequence_index} 不在目标序列范围内（共 {len(targets)} 点）")
            if not self._checkpoint_mapping_matches(document.get("force_frame_mapping")):
                raise ValueError("当前传感器坐标映射与原实验不一致")
            saved_parameters = document.get("parameters")
            if isinstance(saved_parameters, dict):
                self._apply_static_full_parameters(saved_parameters)
        except Exception as exc:
            QMessageBox.warning(self, "继续全静态标定", f"无法读取已有实验：{exc}")
            return
        if not self._devices_ready(require_k=False):
            return

        k_payload = document.get("force_control_result")
        try:
            restored_k = KIdentificationResult(**k_payload) if isinstance(k_payload, dict) else load_last_valid_k_result(folder)
        except Exception:
            restored_k = None
        if not restored_k or not restored_k.valid:
            QMessageBox.warning(self, "继续全静态标定", "已有实验中没有可恢复的有效 K 辨识结果")
            return
        precomp_payload = document.get("mini45_precomp")
        if isinstance(precomp_payload, dict) and precomp_payload.get("enabled"):
            restored_bias = {field: float(precomp_payload["bias"][field]) for field in ZERO_BIAS}
            restored_quality = str(precomp_payload.get("quality") or "warning")
        else:
            legacy_precomp = load_latest_precomp(folder)
            if not legacy_precomp:
                QMessageBox.warning(self, "继续全静态标定", "已有实验中没有可恢复的 Mini45 预补偿结果")
                return
            restored_bias, restored_quality = legacy_precomp

        try:
            recorder = CsvRecorder(folder)
            recorder.start(resume=True)
        except Exception as exc:
            QMessageBox.warning(self, "继续全静态标定", f"无法追加打开已有实验文件：{exc}")
            return
        self.recorder = recorder
        self.static_full_auto_recording = True
        self.record_btn.setText("结束实验批次")
        self.record_status.setText(str(folder))
        self.output_dir.setText(str(folder.parent))
        self.force_control_result = restored_k
        self.force_control_state = DecoupledControlState()
        self.update_k_display(restored_k)
        self.mini45_precomp_enabled = True
        self.mini45_precomp_active = False
        self.mini45_precomp_bias = dict(restored_bias)
        self.mini45_precomp_quality = restored_quality
        self.mini45_precomp_status.setText(
            f"预补偿：已从已有实验恢复（{restored_quality}）\n"
            f"{self._mini45_precomp_bias_text(self.mini45_precomp_bias)}"
        )
        self.marker_id = int(document.get("marker_id", 0))
        self._static_full_pending_targets = targets
        self._static_full_pending_resume = {
            "sequence_index": sequence_index,
            "completed_points": int(document.get("completed_points", sequence_index)),
            "invalid_points": int(document.get("invalid_points", 0)),
        }
        self.buffer.clear()
        self.reset_force_filter(log=False)
        self._update_force_frame_mapping_lock()
        self._log(
            f"已加载全静态检查点：{folder}，将从第 {sequence_index + 1}/{len(targets)} 点继续；"
            "K 与 Mini45 预补偿已恢复"
        )
        self._start_static_full_profile_wait()

    def _static_full_begin(self) -> None:
        """配置就绪后启动全静态序列。"""
        targets = list(self._static_full_pending_targets)
        resume = self._static_full_pending_resume
        self._static_full_pending_targets = []
        self._static_full_pending_resume = None
        self.static_full_setup_stage = ""
        self.static_full_active = True
        self.static_full_paused = False
        self.static_full_returning_zero = False
        self.static_full_recovering_mini45 = False
        self.static_full_recovering_esp32 = False
        self.static_full_points_completed = int(resume["completed_points"]) if resume else 0
        self.static_full_points_invalid = int(resume["invalid_points"]) if resume else 0
        self.sequence_targets = targets
        self.sequence_index = int(resume["sequence_index"]) if resume else 0
        self.calibration_mode = "static_full"
        self.calibration_paused = False
        self._update_static_full_buttons()
        self._update_calibration_buttons()
        if resume:
            self.static_full_status.setText(
                f"已恢复 — 从第 {self.sequence_index + 1}/{len(targets)} 个标定点继续"
            )
            self._log(f"全静态标定恢复运行：从第 {self.sequence_index + 1}/{len(targets)} 点继续")
        else:
            self.static_full_status.setText(f"运行中 — 共 {len(targets)} 个标定点")
            self._log(f"全静态标定开始：共 {len(targets)} 个标定点")
        self._save_static_full_checkpoint("running", "恢复运行" if resume else "开始运行")
        self.start_next_sequence_target()

    def pause_static_full(self) -> None:
        if not self.static_full_active or self.static_full_paused or self.static_full_returning_zero:
            return
        self.static_full_paused = True
        self.calibration_paused = True
        self.stop_auto_force("暂停")
        self._save_static_full_checkpoint("paused", "用户暂停")
        self._update_static_full_buttons()
        self._update_calibration_buttons()
        self.static_full_status.setText("已暂停")
        self._log("全静态标定已暂停")

    def resume_static_full(self) -> None:
        if not self.static_full_active or not self.static_full_paused:
            return
        if self.force_zero_active:
            return
        self.static_full_paused = False
        self.calibration_paused = False
        self._update_static_full_buttons()
        self._update_calibration_buttons()
        self.static_full_status.setText(
            f"运行中 — {self.sequence_index}/{len(self.sequence_targets)}"
        )
        self._log("全静态标定继续")
        self.start_next_sequence_target()

    def stop_static_full(self, reason: str = "人工停止") -> None:
        if self.static_full_retest_active:
            self.stop_static_full_retest(reason)
            return
        if not self._static_full_busy():
            return
        self._save_static_full_checkpoint("stopped", reason)
        self.static_full_setup_stage = ""
        self._static_full_profile_wait = False
        self._static_full_pending_resume = None
        self._static_full_pending_targets = []
        if self.mini45_precomp_active:
            self.finish_mini45_precomp(reason)
        if self.k_ident_active:
            self.abort_k_identification(reason)
        self.stop_auto_force(reason)
        self.static_full_active = False
        self.static_full_paused = False
        self.static_full_returning_zero = False
        self.static_full_recovering_mini45 = False
        self.static_full_recovering_esp32 = False
        self.mini_btn.setEnabled(True)
        self.mini_btn.setText("断开 Mini45" if self.mini45 else "连接 Mini45")
        self.esp_btn.setEnabled(True)
        self.esp_btn.setText("断开 ESP32" if self.esp32 else "连接 ESP32")
        self.calibration_mode = ""
        self.calibration_paused = False
        self.static_point_collector = None
        self.sequence_targets = []
        self.sequence_index = 0
        self.active_target = None
        self._update_static_full_buttons()
        self._update_calibration_buttons()
        self.static_full_status.setText(f"已停止：{reason}")
        self._log(f"全静态标定停止：{reason}，完成 {self.static_full_points_completed} 点")
        self._close_static_full_recording()

    def _finish_static_full(self) -> None:
        if self.static_full_retest_active:
            self._finish_static_full_retest()
            return
        total = len(self.sequence_targets)
        self._save_static_full_checkpoint("completed", "全部标定点完成并已回零")
        self.stop_auto_force("")
        self.static_full_active = False
        self.static_full_paused = False
        self.static_full_returning_zero = False
        self.static_full_recovering_mini45 = False
        self.static_full_recovering_esp32 = False
        self.static_full_setup_stage = ""
        self.esp_btn.setEnabled(True)
        self.esp_btn.setText("断开 ESP32" if self.esp32 else "连接 ESP32")
        self.calibration_mode = ""
        self.calibration_paused = False
        self.static_point_collector = None
        self.active_target = None
        self.sequence_targets = []
        self.sequence_index = 0
        self._update_static_full_buttons()
        self._update_calibration_buttons()
        self.static_full_status.setText(
            f"完成 — {self.static_full_points_completed} 有效 / "
            f"{self.static_full_points_invalid} 无效 / "
            f"共 {total} 点，已自动卸载回零"
        )
        self.cal_status.setText("标定状态：全静态标定完成，已卸载回零")
        self._log(
            f"全静态标定完成：{self.static_full_points_completed} 有效，"
            f"{self.static_full_points_invalid} 无效"
        )
        self._close_static_full_recording()

    def _update_static_full_estimate(self, *args: object) -> None:
        """参数变化时实时更新预估标定时间。"""
        if self.static_full_active:
            return
        try:
            targets = self._static_full_targets_from_parameters(self._static_full_parameters())
        except Exception:
            self.static_full_estimate.setText("参数无效")
            self.static_full_estimate.setStyleSheet("font-weight: bold; color: #C62828;")
            return

        n = len(targets)
        collector = StaticPointCollector(required_cap_samples=self._static_full_required_cap_samples())
        seconds_per_point = collector.stable_hold_s + collector.required_cap_samples / STATIC_PRECISION.nominal_hz
        total_s = n * seconds_per_point
        if total_s < 3600:
            minutes = total_s / 60
            text = f"{n} 点 ≈ {minutes:.0f} 分钟"
        else:
            hours = total_s / 3600
            text = f"{n} 点 ≈ {hours:.1f} 小时"

        fz_n = sum(1 for t in targets if t.axis == "Fz")
        fx_n = sum(1 for t in targets if t.axis == "Fx")
        fy_n = sum(1 for t in targets if t.axis == "Fy")
        diag_n = n - fz_n - fx_n - fy_n
        parts = []
        if fz_n:
            parts.append(f"Fz单轴 {fz_n}")
        if fx_n:
            parts.append(f"Fx预载 {fx_n}")
        if fy_n:
            parts.append(f"Fy预载 {fy_n}")
        if diag_n:
            parts.append(f"斜向 {diag_n}")
        detail = " + ".join(parts) if parts else "未选择流程"
        text += f"（{detail}；不含运动与重试）"

        self.static_full_estimate.setText(text)
        self.static_full_estimate.setStyleSheet("font-weight: bold; color: #1565C0;")

    @staticmethod
    def _static_full_phase_name(target: CalibrationTarget | None) -> str:
        if target is None:
            return "—"
        if target.branch == "return_zero":
            return "结束卸载回零"
        axis = target.axis
        if axis == "Fz":
            return "Fz 单轴标定"
        if axis == "Fx":
            return f"Fx 单轴加载 @ Fz={target.target_fz:.0f}N"
        if axis == "Fy":
            return f"Fy 单轴加载 @ Fz={target.target_fz:.0f}N"
        # combined: diagonal
        return f"斜向加载 {target.direction} @ Fz={target.target_fz:.0f}N"

    def _update_static_full_progress(self) -> None:
        if not self.static_full_active or self.static_full_recovering_mini45:
            return
        target = self.active_target
        collector = self.static_point_collector
        total = len(self.sequence_targets)
        idx = self.sequence_index

        # Build status lines
        lines = []
        lines.append(f"【{self._static_full_phase_name(target)}】")
        if target:
            lines.append(
                f"目标: Fx={target.target_fx:+.3f}  Fy={target.target_fy:+.3f}  "
                f"Fz={target.target_fz:+.3f} N  |  "
                f"{target.branch} / {target.direction}  |  "
                f"第 {target.cycle_index} 组"
            )

        if collector is not None:
            cap_n = len(collector.cap_samples)
            cap_total = collector.required_cap_samples
            if collector.complete:
                cap_text = f"电容: {cap_n}/{cap_total} ✓ 完成，等待质量判定…"
            elif collector.collecting and not collector.collection_paused:
                cap_text = f"电容: {cap_n}/{cap_total}  采集中"
            elif collector.collection_paused:
                cap_text = f"电容: {cap_n}/{cap_total}  暂停（力越界，等待恢复）"
            elif collector.in_window_since_s > 0:
                stable_s = collector.stable_elapsed_s
                cap_text = f"稳定保持中… {stable_s:.1f}s / {collector.stable_hold_s:.0f}s"
            else:
                cap_text = "等待力进入目标窗口…"
            if collector.retry_count > 0:
                cap_text += f"  [重试 {collector.retry_count}/{collector.max_retries}]"
            lines.append(cap_text)
        elif self.auto_force_active:
            lines.append("力控逼近中…")

        if self.static_full_returning_zero:
            lines.append(
                f"进度: 标定点已完成，正在自动卸载回零  |  "
                f"有效 {self.static_full_points_completed}  |  无效 {self.static_full_points_invalid}"
            )
        else:
            lines.append(
                f"进度: 点 {idx + 1}/{total}  |  "
                f"已完成 {self.static_full_points_completed}  |  "
                f"无效 {self.static_full_points_invalid}"
            )
        self.static_full_status.setText("\n".join(lines))

    def _update_static_full_profile_switch(self) -> None:
        """检测全静态标定的 CAP 配置切换是否就绪。"""
        if not getattr(self, "_static_full_profile_wait", False):
            return
        target_profile = STATIC_PRECISION.name
        if self._profile_is_ready(target_profile):
            self._static_full_profile_wait = False
            if self.static_full_setup_stage == "retest_profile":
                self._static_full_retest_begin()
            else:
                self._static_full_begin()
            return
        elapsed = time.monotonic() - self._static_full_profile_start_s
        if elapsed > 15.0:
            self._static_full_profile_wait = False
            QMessageBox.warning(
                self, "全静态标定",
                f"MC1081 配置切换超时（{target_profile}），请检查 ESP32 连接和 CAP 流"
            )
            self._log(f"全静态标定：等待 {target_profile} 配置超时")
            if self.static_full_setup_stage == "retest_profile":
                self.stop_static_full_retest(f"{target_profile} 配置切换超时")
            else:
                self._abort_static_full_setup(f"{target_profile} 配置切换超时")

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

    def _new_static_point_collector(self) -> StaticPointCollector:
        if self.calibration_mode in {"static_full", "static_full_retest"}:
            return StaticPointCollector(required_cap_samples=self._static_full_required_cap_samples())
        return StaticPointCollector()

    def _build_sequence_targets(self) -> list[CalibrationTarget]:
        axis = self._combo_value(self.load_axis)
        if axis == "all":
            return generate_three_axis_sequence(
                fz_max_force=self.seq_fz_max.value(),
                fz_step=self.seq_fz_step.value(),
                shear_max_force=self.seq_shear_max.value(),
                shear_step=self.seq_shear_step.value(),
                target_fz=self.target_fz.value(),
                shear_direction_mode=self._combo_value(self.seq_shear_direction),
                cycles=self.seq_cycles.value(),
            )
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
            if self.calibration_mode in {"static_full", "static_full_retest"}:
                self._start_static_full_return_zero()
                return
            self.stop_auto_force("")
            self.calibration_mode = ""
            self.static_point_collector = None
            if self.workflow.active and self.workflow.stage == "static_sequence":
                self._advance_workflow()
            else:
                self.stop_calibration("静态正反程标定完成")
            return
        self.active_target = self.sequence_targets[self.sequence_index]
        _use_collector = (
            (self.workflow.active and self.workflow.stage == "static_sequence")
            or self.calibration_mode in {"static_full", "static_full_retest"}
        )
        if _use_collector:
            self.static_point_collector = self._new_static_point_collector()
            self.static_point_collector.begin(time.monotonic())
        else:
            self.static_point_collector = None
        self._apply_target_to_ui(self.active_target)
        if self.calibration_mode == "static_full_retest" and self.recorder:
            self.recorder.set_static_full_retest_source(
                source_point_index=self.active_target.point_index,
                source_cycle_id=f"cycle_{self.active_target.cycle_index:03d}",
            )
        if self.calibration_mode == "static_full_retest":
            label = "补测"
        elif self.calibration_mode == "static_full":
            label = "全静态"
        else:
            label = "正反程"
        self.cal_status.setText(f"标定状态：{label}点 {self.sequence_index + 1}/{len(self.sequence_targets)}")
        if not self.start_auto_force():
            self.static_point_collector = None
            if self.workflow.active and self.workflow.stage == "static_sequence":
                self.abort_full_workflow("静态正反程标定力控启动失败")
            elif self.calibration_mode in {"static_full", "static_full_retest"}:
                self.stop_static_full("全静态标定力控启动失败")
            else:
                self.stop_calibration("启动失败")

    def _start_static_full_return_zero(self) -> None:
        """After the final calibration point, unload all three axes to zero."""
        self.stop_auto_force("")
        self.static_point_collector = None
        self.static_full_returning_zero = True
        if self.calibration_mode == "static_full_retest" and self.recorder:
            self.recorder.set_static_full_retest_source()
        self.active_target = CalibrationTarget(
            "combined",
            "none",
            "return_zero",
            0.0,
            0.0,
            0.0,
            max((int(target.cycle_index) for target in self.sequence_targets), default=1),
            len(self.sequence_targets) + 1,
        )
        self._apply_target_to_ui(self.active_target)
        self.static_full_status.setText("标定点已完成，正在自动卸载回零…")
        self._log("全静态标定点已完成，开始自动卸载回零")
        self._save_static_full_checkpoint("returning_zero", "标定点完成，正在卸载回零")
        self._update_static_full_buttons()
        if not self.start_auto_force():
            self.stop_static_full("自动卸载回零启动失败")

    def _apply_target_to_ui(self, target: CalibrationTarget) -> None:
        self._set_combo_by_data(self.load_axis, target.axis)
        self._set_combo_by_data(self.branch, target.branch)
        self._set_combo_by_data(self.direction, target.direction)
        self.target_fx.setValue(target.target_fx)
        self.target_fy.setValue(target.target_fy)
        self.target_fz.setValue(target.target_fz)

    def start_training_collection(
        self,
        targets: list[TrainingTarget] | None = None,
        trajectory_type: str | None = None,
        profile: str | None = None,
    ) -> None:
        try:
            if targets is None:
                fz_levels = parse_force_levels(self.training_fz_levels.text())
                targets = generate_training_trajectory(
                    fz_levels=fz_levels,
                    shear_max=self.training_shear_max.value(),
                    trajectory_type=trajectory_type or self._combo_value(self.training_trajectory_type),
                    target_step_n=self.training_target_step.value(),
                    random_points=self.training_random_points.value(),
                )
            self.training_targets = list(targets)
        except Exception as exc:
            QMessageBox.warning(self, "训练数据采集", str(exc))
            return
        if not self.training_targets:
            QMessageBox.warning(self, "训练数据采集", "当前参数没有生成训练轨迹")
            return

        self.training_count += 1
        self.current_cycle_id = f"training_{self.training_count:03d}"
        self.training_profile = profile or self.current_cap_profile or TRAINING_BALANCED.name
        self.training_active = True
        self.training_target_index = 0
        self.training_current_target = None
        self.active_target = CalibrationTarget("combined", "none", "loading", 0.0, 0.0, 0.0)
        if self.recorder:
            self.recorder.start_training_files(self.training_profile)
        self._write_training_marker("training_start")
        self._enter_training_target(0)
        self.start_auto_force()
        if not self.auto_force_active:
            self.finish_training_collection("启动失败")
            self._update_calibration_buttons()
            return
        profile_label = "平衡频率" if self.training_profile == TRAINING_BALANCED.name else "高速补充"
        self._log(f"训练数据采集开始：写入{profile_label}训练专用原始时序和 marker 文件")
        self._update_calibration_buttons()

    def _enter_training_target(self, index: int) -> None:
        self.training_target_index = index
        self.training_current_target = self.training_targets[index]
        self.training_target_start_s = time.monotonic()
        self.training_last_ramp_update_s = self.training_target_start_s
        if self.latest_force_sample:
            self.training_ramp_target = (
                self.latest_force_sample.fx,
                self.latest_force_sample.fy,
                self.latest_force_sample.fz,
            )
        self._set_training_target(self.training_current_target)
        self._write_training_marker(self._training_start_marker_note(index))

    def _set_training_target(self, target: TrainingTarget) -> None:
        self.active_target = CalibrationTarget(
            axis="combined",
            direction=target.direction,
            branch=target.branch,
            target_fx=self.training_ramp_target[0],
            target_fy=self.training_ramp_target[1],
            target_fz=self.training_ramp_target[2],
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
        if self.latest_force_sample and self.active_target:
            dt = max(0.0, now - self.training_last_ramp_update_s)
            self.training_last_ramp_update_s = now
            self.training_ramp_target = advance_ramped_force_target(
                self.training_ramp_target,
                (target.target_fx, target.target_fy, target.target_fz),
                (self.latest_force_sample.fx, self.latest_force_sample.fy, self.latest_force_sample.fz),
                rate_n_s=0.20,
                elapsed_s=dt,
                max_lag_n=0.20,
            )
            self.active_target.target_fx, self.active_target.target_fy, self.active_target.target_fz = self.training_ramp_target
        if self.latest_force_sample and training_target_reached(self.latest_force_sample, target, self.training_arrival_window.value()):
            self._write_training_marker("target_reached")
            self._advance_training_target()
            return
        if training_target_timed_out(elapsed, self.training_max_wait_s.value()):
            self._write_training_marker("target_timeout")
            self._write_training_marker("target_skipped")
            if self.workflow.active:
                self.workflow.skipped_training_targets += 1
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
            if self.workflow.active and self.workflow.stage in {"training_balanced", "training_fast"}:
                self.finish_training_collection("完成")
            else:
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
        if self.workflow.active and self.workflow.stage in {"training_balanced", "training_fast"}:
            if "完成" in reason:
                self.workflow_training_trajectory_index += 1
                self._start_workflow_training_trajectory()
            elif reason:
                self.abort_full_workflow(f"训练数据采集异常结束：{reason}")

    def _write_training_marker(self, note: str) -> None:
        if not self.recorder:
            return
        self.marker_id += 1
        meta = self._meta()
        meta.note = f"{meta.note}; {note}" if meta.note else note
        target = self.training_current_target
        if target:
            meta.target_fx = target.target_fx
            meta.target_fy = target.target_fy
            meta.target_fz = target.target_fz
            meta.preload_n = target.target_fz
            meta.direction = target.direction
            meta.branch = target.branch
        self.recorder.write_training_marker(
            self.marker_id,
            meta,
            trajectory_type=target.trajectory_type if target else self._combo_value(self.training_trajectory_type),
            phase=target.phase if target else note,
            target_shear_n=target.target_shear_n if target else "",
            target_angle_deg=target.target_angle_deg if target else "",
            profile=self.training_profile,
        )

    def pause_calibration(self) -> None:
        if self.static_full_active:
            self.pause_static_full()
            return
        if not self._calibration_active():
            return
        self.calibration_paused = True
        if self.training_active:
            self.training_pause_started_s = time.monotonic()
        if self.static_point_collector:
            self.static_point_collector.begin(time.monotonic())
        try:
            if self.motion:
                self.motion.stop_all()
        except Exception:
            pass
        self.cal_status.setText("标定状态：已暂停")
        self._update_calibration_buttons()

    def resume_calibration(self) -> None:
        if self.force_zero_active:
            return
        if self.static_full_active and self.static_full_paused:
            self.resume_static_full()
            return
        if not self.calibration_mode:
            return
        if self.training_active and self.training_pause_started_s > 0.0:
            self.training_target_start_s += time.monotonic() - self.training_pause_started_s
            self.training_pause_started_s = 0.0
            self.training_last_ramp_update_s = time.monotonic()
        self.calibration_paused = False
        self.cal_status.setText("标定状态：继续")
        self._update_calibration_buttons()

    def skip_calibration_point(self) -> None:
        if self.training_active:
            self._write_training_marker("target_skipped")
            self._advance_training_target()
            self._update_calibration_buttons()
            return
        if self.calibration_mode == "sequence":
            self.sequence_index += 1
            self.start_next_sequence_target()
        else:
            self.stop_calibration("已跳过当前点")

    def stop_calibration(self, reason: str = "") -> None:
        if self._static_full_busy():
            self.stop_static_full(reason or "停止")
            return
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
        self._update_calibration_buttons()

    def start_zero_drift(self) -> None:
        self.stop_auto_force("零点漂移采集中")
        self.zero_drift_count += 1
        self.current_cycle_id = f"zero_{self.zero_drift_count:03d}"
        self.zero_drift_active = True
        self.zero_drift_start_s = time.monotonic()
        self.zero_drift_sample_count = 0
        path = self.recorder.start_zero_drift_timeseries() if self.recorder else None
        self.zero_drift_file = path.name if path else ""
        self._write_marker_with_note("zero_start")
        self.cal_status.setText(f"标定状态：空载零点漂移采集中，文件 {self.zero_drift_file}")
        self._log(f"空载零点漂移开始：{self.zero_drift_file}")
        self._update_calibration_buttons()

    def finish_zero_drift(self, reason: str = "完成") -> None:
        if not self.zero_drift_active:
            return
        self.zero_drift_active = False
        self._write_marker_with_note("zero_end")
        if self.recorder:
            self.recorder.stop_zero_drift_timeseries()
        self.cal_status.setText(f"标定状态：空载零点漂移{reason}，共 {self.zero_drift_sample_count} 行")
        self._log(f"空载零点漂移{reason}：{self.zero_drift_sample_count} 行")
        if self.calibration_mode == "zero":
            self.calibration_mode = ""
        self._update_calibration_buttons()
        if self.workflow.active and self.workflow.stage == "zero_drift":
            if reason == "完成":
                self._advance_workflow()
            else:
                self.abort_full_workflow(f"零点漂移异常结束：{reason}")

    def _write_marker_with_note(self, note: str) -> None:
        self.marker_id += 1
        meta = self._meta()
        meta.note = f"{meta.note}; {note}" if meta.note else note
        if self.recorder:
            self.recorder.write_marker(self.marker_id, meta)

    def start_auto_force(self) -> bool:
        if not self.motion:
            QMessageBox.warning(self, "自动标定", "请先连接 Arduino 电机控制串口")
            return False
        if not self.mini45:
            QMessageBox.warning(self, "自动标定", "请先连接 Mini45 并确认有实时力数据")
            return False
        if self.k_ident_active:
            QMessageBox.warning(self, "自动标定", "K 正在自动辨识，请等待辨识结束")
            return False
        if not self.force_control_result or not self.force_control_result.valid:
            QMessageBox.warning(self, "自动标定", "请先完成有效的 K 自动辨识")
            return False
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
            return False
        meta = self._meta()
        self.motion_status.setText(f"电机状态：自动逼近 {meta.axis}")
        self.cal_status.setText(f"标定状态：自动逼近目标 Fx={meta.target_fx:.3f}, Fy={meta.target_fy:.3f}, Fz={meta.target_fz:.3f}")
        self._log(f"开始解耦自动力控，目标 Fx={meta.target_fx:.3f}, Fy={meta.target_fy:.3f}, Fz={meta.target_fz:.3f}")
        self._update_calibration_buttons()
        return True

    def stop_auto_force(self, reason: str = "") -> None:
        if not self.auto_force_active and not self.auto_force_holding:
            force_zero_was_active = self.force_zero_active
            self.force_zero_active = False
            if force_zero_was_active:
                self._update_calibration_buttons()
            return
        force_zero_was_active = self.force_zero_active
        self.force_zero_active = False
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
            if force_zero_was_active:
                self.cal_status.setText(f"标定状态：力归0/卸载已停止：{reason}")
        self._update_calibration_buttons()

    def _fail_auto_force(self, reason: str) -> None:
        """自动流程中的力控故障必须终止整套实验，手动模式只停止当前力控。"""
        if self.force_zero_active:
            self.force_zero_active = False
            self.stop_auto_force(reason)
            self.calibration_paused = bool(
                self.training_active
                or self.static_full_active
                or self.calibration_mode
                or self.workflow.active
            )
            self.cal_status.setText(f"标定状态：力归0/卸载失败：{reason}，保持暂停")
            self._log(f"力归0/卸载失败：{reason}，流程保持暂停")
            self._update_static_full_buttons()
            self._update_workflow_ui()
            self._update_calibration_buttons()
            return
        if self.workflow.active:
            self.abort_full_workflow(reason)
        elif self._static_full_busy():
            if self.static_full_active and "Mini45" in reason and ("未更新" in reason or "断开" in reason):
                self._begin_static_full_mini45_recovery(reason)
            else:
                self.stop_static_full(reason)
        else:
            self.stop_auto_force(reason)

    def toggle_recording(self) -> None:
        if self.recorder:
            if self._static_full_busy():
                self.stop_static_full("实验批次结束")
                return
            elif self.training_active or self.zero_drift_active or self.auto_force_active:
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

    @staticmethod
    def _force_within_tolerances(sample, meta: ExperimentMeta, tolerances: tuple[float, float, float]) -> bool:
        return bool(
            sample
            and abs(sample.fx - meta.target_fx) <= tolerances[0]
            and abs(sample.fy - meta.target_fy) <= tolerances[1]
            and abs(sample.fz - meta.target_fz) <= tolerances[2]
        )

    def _static_collection_force_in_window(self) -> bool:
        if not self.static_point_collector or not self.active_target or not self.latest_force_sample:
            return False
        return self._force_within_tolerances(
            self.latest_force_sample,
            self._meta(),
            collection_tolerances(),
        )

    def _update_static_point_collection(self) -> None:
        collector = self.static_point_collector
        if not is_static_collection_mode(self.calibration_mode) or not collector or not self.active_target or self.calibration_paused:
            return
        now = time.monotonic()
        meta = self._meta()
        strict_tolerances = (0.05, 0.05, 0.08)
        hold_tolerances = collection_tolerances(strict_tolerances)
        settings = StabilitySettings(
            stable_window_s=5.0,
            tolerance_fx=strict_tolerances[0],
            tolerance_fy=strict_tolerances[1],
            tolerance_fz=strict_tolerances[2],
        )
        hold_settings = StabilitySettings(
            stable_window_s=5.0,
            tolerance_fx=hold_tolerances[0],
            tolerance_fy=hold_tolerances[1],
            tolerance_fz=hold_tolerances[2],
        )
        # 使用短窗口判断当前力是否稳定，再由 StaticPointCollector 连续计满 5 s，
        # 避免先等待一个完整 5 s 窗口后又重复计时 5 s。
        force_samples = [
            sample
            for sample in self.buffer.window(now, 0.5)
            if sample.fx is not None and sample.fy is not None and sample.fz is not None
        ]
        result = evaluate_three_axis_stability(force_samples, meta, settings, SafetySettings())
        hold_result = evaluate_three_axis_stability(force_samples, meta, hold_settings, SafetySettings())
        current = self.latest_force_sample
        in_window = self._force_within_tolerances(current, meta, strict_tolerances)
        collection_in_window = self._force_within_tolerances(current, meta, hold_tolerances)
        retained_samples = len(collector.cap_samples)
        state_event = collector.update_force_state(
            now,
            in_window=in_window,
            stable=result.stable,
            collection_in_window=collection_in_window,
            collection_stable=hold_result.stable,
        )
        if state_event == "collection_reset":
            self._write_workflow_event(
                "static_collection_reset",
                "retry",
                f"持续超出采集保持窗口，丢弃 {retained_samples} 个电容样本",
            )
            self._log(f"持续超出采集保持窗口，已丢弃当前静态点的 {retained_samples} 个电容样本并重新稳定")
        elif state_event == "collection_preserved":
            self._write_workflow_event(
                "static_collection_preserved",
                "paused",
                f"已采集 {retained_samples}/{collector.required_cap_samples}，保留进度等待力恢复",
            )
            self._log(
                f"静态点已采集 {retained_samples}/{collector.required_cap_samples}，"
                "持续越界后保留进度并等待力恢复"
            )

        if collector.complete:
            bounds = collector.time_bounds
            if not bounds:
                return
            start_s, end_s = bounds
            selected_force = [
                sample
                for sample in self.buffer.window(end_s, max(0.0, end_s - start_s) + 1e-6)
                if start_s <= sample.monotonic_s <= end_s
                and sample.fx is not None
                and sample.fy is not None
                and sample.fz is not None
                and self._force_within_tolerances(sample, meta, hold_tolerances)
            ]
            samples = selected_force + collector.selected_cap_samples()
            final_result = evaluate_three_axis_stability(samples, meta, settings, SafetySettings())
            if not final_result.stable and collector.retry(now):
                self.auto_force_holding = False
                self._write_workflow_event("static_point_retry", "retry", final_result.reject_reason)
                self._log(
                    f"静态点质量判定未通过，正在执行第 {collector.retry_count} 次重试："
                    f"{self._display_reason(final_result.reject_reason)}"
                )
                return
            self.marker_id += 1
            if self.recorder:
                self.recorder.write_marker(self.marker_id, meta)
                point = build_calibration_point(samples, meta, self.marker_id, final_result.stable, final_result.reject_reason)
                if point:
                    selected_caps = collector.selected_cap_samples()
                    point.timestamp_start = selected_caps[0].timestamp
                    point.timestamp_end = selected_caps[-1].timestamp
                    self.recorder.write_calibration_point(point)
            if final_result.stable:
                if self.workflow.active:
                    self.workflow.completed_static_points += 1
                if self.calibration_mode in {"static_full", "static_full_retest"}:
                    self.static_full_points_completed += 1
                self._write_workflow_event("static_point_complete", "complete")
            else:
                if self.workflow.active:
                    self.workflow.invalid_static_points += 1
                if self.calibration_mode in {"static_full", "static_full_retest"}:
                    self.static_full_points_invalid += 1
                self._write_workflow_event("static_point_complete", "invalid", final_result.reject_reason)
            if self.static_full_active:
                self._update_static_full_progress()
            self.sequence_index += 1
            if self.static_full_active:
                self._save_static_full_checkpoint("running", "静态点完成")
            self.stop_auto_force("")
            self.static_point_collector = None
            self.start_next_sequence_target()
            return

        if collector.timed_out(now):
            if collector.retry(now):
                self.auto_force_holding = False
                self._write_workflow_event("static_point_retry", "retry", "静态点超时")
                self._log(f"静态点超时，正在执行第 {collector.retry_count} 次重试")
            else:
                self.marker_id += 1
                invalid_meta = self._meta()
                invalid_meta.note = f"{invalid_meta.note}; static_point_timeout" if invalid_meta.note else "static_point_timeout"
                if self.recorder:
                    self.recorder.write_marker(self.marker_id, invalid_meta)
                if self.workflow.active:
                    self.workflow.invalid_static_points += 1
                if self.calibration_mode in {"static_full", "static_full_retest"}:
                    self.static_full_points_invalid += 1
                self._write_workflow_event("static_point_skipped", "invalid", "重试两次后仍超时")
                if self.static_full_active:
                    self._update_static_full_progress()
                self.sequence_index += 1
                if self.static_full_active:
                    self._save_static_full_checkpoint("running", "静态点超时后跳过")
                self.stop_auto_force("")
                self.static_point_collector = None
                self.start_next_sequence_target()

    def _tick(self) -> None:
        tick_started_s = time.perf_counter()
        self._drain_esp32()
        self._drain_mini45()
        self._drain_motion()
        self._update_static_full_mini45_recovery()
        self._update_static_full_esp32_recovery()
        if self.mini45_precomp_active and self.mini45_precomp_start_monotonic_s is not None:
            elapsed = max(0.0, time.monotonic() - self.mini45_precomp_start_monotonic_s)
            self.mini45_precomp_status.setText(
                f"预补偿：测量中 {min(elapsed, self.mini45_precomp_duration_s):.1f}/"
                f"{self.mini45_precomp_duration_s:.0f} s，样本 {len(self.mini45_precomp_samples)}"
            )
            if elapsed >= self.mini45_precomp_duration_s:
                self.finish_mini45_precomp()
        self._update_k_identification()
        self._update_static_full_profile_switch()
        if self._static_full_profile_wait and not self.esp32:
            self.stop_static_full("ESP32 已断开")
        elif (
            self.static_full_active
            and not self.esp32
            and not self.static_full_recovering_mini45
            and not self.static_full_recovering_esp32
        ):
            self._begin_static_full_esp32_recovery("ESP32 已断开")
        elif (
            self.static_full_active
            and not self.static_full_recovering_mini45
            and not self.static_full_recovering_esp32
            and self.last_cap_time > 0.0
            and time.monotonic() - self.last_cap_time > 5.0
        ):
            self._begin_static_full_esp32_recovery("ESP32 电容数据超过 5 秒未更新")
        elif self.static_full_active and not self.static_full_recovering_esp32:
            self._update_static_full_progress()
        if self.zero_drift_active and not self.calibration_paused and time.monotonic() - self.zero_drift_start_s >= self.zero_duration_s.value():
            self.finish_zero_drift("完成")
        self._update_calibration_progress_status()
        self._update_training_collection()
        self._update_auto_force()
        self._update_static_point_collection()
        self._update_full_workflow()
        self._update_status()
        self._flush_plot_updates(tick_started_s)

    def _update_calibration_progress_status(self) -> None:
        now = time.monotonic()
        if now - self.last_cal_progress_update_s < 0.5:
            return
        self.last_cal_progress_update_s = now
        if self.zero_drift_active:
            elapsed = max(0.0, now - self.zero_drift_start_s)
            total = self.zero_duration_s.value()
            remaining = max(0.0, total - elapsed)
            self.cal_status.setText(
                f"标定状态：空载零点漂移进行中，已采集 {elapsed:.1f}/{total:.1f}s，"
                f"剩余 {remaining:.1f}s，已保存 {self.zero_drift_sample_count} 行"
            )

    def _drain_esp32(self) -> None:
        if not self.esp32:
            return
        processed = 0
        deadline = time.perf_counter() + self.esp32_drain_budget_s
        while processed < self.max_esp32_items_per_tick and time.perf_counter() < deadline:
            try:
                item = self.esp32.out_queue.get_nowait()
            except queue.Empty:
                break
            processed += 1
            if isinstance(item, Esp32ProfileStatus):
                self.current_cap_profile = item.name
                self.workflow_profile_label.setText(
                    f"MC1081 配置：{item.name}，CNT={item.cnt}，CAVG={item.cavg}，实际 -- Hz"
                )
                self._log(f"ESP32 配置确认：{item.name}，CNT={item.cnt}，CAVG={item.cavg}")
            elif isinstance(item, Esp32Log):
                self._log(f"ESP32 {self._display_level(item.level)}：{item.message}")
                if item.level == "error" and "PROFILE" in item.message.upper() and self.workflow.active:
                    self.abort_full_workflow(f"ESP32 配置切换失败：{item.message}")
            elif isinstance(item, CapSample):
                self.last_cap_time = item.monotonic_s
                if item.cap_profile:
                    self.current_cap_profile = item.cap_profile
                if item.cap_effective_hz:
                    self.current_cap_effective_hz = item.cap_effective_hz
                snapshot = CombinedSnapshot.from_cap(item)
                self.buffer.append(snapshot)
                if self.recorder:
                    if self.training_active and not self.calibration_paused:
                        self.recorder.write_training_raw(snapshot, self.training_profile)
                    elif self._static_raw_recording_active():
                        self.recorder.write_raw(snapshot)
                    if self.zero_drift_active and not self.calibration_paused:
                        self.recorder.write_zero_drift_raw(snapshot)
                if self.zero_drift_active and not self.calibration_paused:
                    self.zero_drift_sample_count += 1
                if (
                    self.static_point_collector
                    and not self.calibration_paused
                    and self._static_collection_force_in_window()
                ):
                    self.static_point_collector.add_cap_sample(item)
                self._add_cap_plot(item)

    def _drain_mini45(self) -> None:
        if not self.mini45:
            return
        processed = 0
        deadline = time.perf_counter() + self.mini45_drain_budget_s
        while processed < self.max_mini45_items_per_tick and time.perf_counter() < deadline:
            try:
                item = self.mini45.out_queue.get_nowait()
            except queue.Empty:
                break
            processed += 1
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
                if self.mini45_precomp_active:
                    # 预补偿统计固定使用坐标映射后的六轴未滤波数据。
                    self.mini45_precomp_samples.append(mapped_item)
                compensated_item = mapped_item
                if self.mini45_precomp_enabled and not self.mini45_precomp_active:
                    compensated_item = subtract_precomp_bias(mapped_item, self.mini45_precomp_bias)
                filtered_item = self.force_filter.update(compensated_item, self._force_filter_settings())
                self.last_force_time = filtered_item.monotonic_s
                self.latest_force_sample = filtered_item
                if first_sample:
                    self.mini_status.setText("Mini45 状态：数据正常")
                    self._log("Mini45 已收到第一帧可解析数据")
                # 控制和稳定判定使用补偿后再滤波的数据；CSV 的 mini45_raw_* 始终保留原始轴值。
                control_snapshot = CombinedSnapshot.from_force(filtered_item, raw_sample=item)
                raw_snapshot = CombinedSnapshot.from_force(compensated_item, raw_sample=item)
                self.buffer.append(control_snapshot)
                if self.recorder:
                    if self.training_active and not self.calibration_paused:
                        self.recorder.write_training_raw(raw_snapshot, self.training_profile)
                    elif self._static_raw_recording_active():
                        self.recorder.write_raw(raw_snapshot)
                    if self.zero_drift_active and not self.calibration_paused:
                        self.recorder.write_zero_drift_raw(raw_snapshot)
                if self.zero_drift_active and not self.calibration_paused:
                    self.zero_drift_sample_count += 1
                self._add_force_plot(filtered_item)

    def _static_raw_recording_active(self) -> bool:
        if self.calibration_paused:
            return False
        return bool(
            self.zero_drift_active
            or self.auto_force_active
            or self.auto_force_holding
            or self.k_ident_active
        )

    def _drain_motion(self) -> None:
        if not self.motion:
            return
        processed = 0
        deadline = time.perf_counter() + self.motion_drain_budget_s
        while processed < self.max_motion_items_per_tick and time.perf_counter() < deadline:
            try:
                item = self.motion.out_queue.get_nowait()
            except queue.Empty:
                break
            processed += 1
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
        prefix = "力归0/卸载中" if self.force_zero_active else ("自动逼近中" if self.auto_force_active else "已连接")
        self.motion_status.setText(f"电机状态：{prefix}，" + "，".join(parts))

    def _update_auto_force(self) -> None:
        if not self.auto_force_active:
            return
        if not self.motion:
            self._fail_auto_force("Arduino 未连接")
            return
        if self.calibration_paused and not self.force_zero_active:
            return
        if not self.latest_force_sample or self.last_force_time <= 0.0:
            return

        now = time.monotonic()
        if now - self.last_force_time > 1.0:
            self._fail_auto_force("Mini45 数据超过 1 秒未更新")
            return

        sample = self.latest_force_sample
        if not self._current_force_safe():
            self._fail_auto_force("力或力矩超过安全限值")
            return
        if not self.force_control_result or not self.force_control_result.valid:
            self._fail_auto_force("K 未辨识或无效")
            return

        meta = self._meta()
        settings = self._stability_settings()
        target_force = [meta.target_fx, meta.target_fy, meta.target_fz]
        current_force = force_vector_from_sample(sample)
        error = [target_force[index] - current_force[index] for index in range(3)]
        tolerances = [settings.tolerance_fx, settings.tolerance_fy, settings.tolerance_fz]
        all_in_window = all(abs(error[index]) <= tolerances[index] for index in range(3))

        if all_in_window:
            if self.force_zero_active:
                self._finish_force_zero_unload()
                return
            if self.training_active:
                # 训练采集由 _update_training_collection 负责到达后立即切换目标，
                # 这里不停车等待，避免连续加载数据出现人为停顿。
                return
            if self.workflow.active and self.workflow.stage == "return_zero":
                self.stop_auto_force("")
                self.calibration_mode = ""
                self._advance_workflow()
                return
            if self.static_full_active and self.static_full_returning_zero:
                self.stop_auto_force("")
                self._finish_static_full()
                return
            if is_static_collection_mode(self.calibration_mode) and self.static_point_collector:
                if not self.auto_force_holding:
                    self.auto_force_holding = True
                    try:
                        self.motion.stop_all()
                    except Exception:
                        pass
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
                    min_effective_step_mm=self.auto_min_effective_step_mm.value(),
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
            self._fail_auto_force(str(exc))

    def _update_status(self) -> None:
        now = time.monotonic()
        active_interval = 0.2
        idle_interval = 0.5
        interval = active_interval if (self.auto_force_active or self.auto_force_holding or self.training_active or self.k_ident_active) else idle_interval
        if now - self.last_status_update_s < interval:
            return
        self.last_status_update_s = now

        if self.zero_drift_active:
            safe = True
            if self.latest_force_sample:
                sample = self.latest_force_sample
                safe = abs(sample.fz) <= 10.0 and abs(sample.fx) <= 4.0 and abs(sample.fy) <= 4.0
            self.window_label.setText("目标窗口：零点采集中")
            self.stable_label.setText("稳定状态：--")
            self.safe_label.setText(f"安全状态：{self._yes_no(safe)}")
            self.window_label.setStyleSheet("color: #666")
            self.stable_label.setStyleSheet("color: #666")
            self.safe_label.setStyleSheet("color: green" if safe else "color: red")
            if self.mini45 and self.last_force_time > 0.0 and now - self.last_force_time > 1.0:
                self.mini_status.setText("Mini45 状态：数据超过 1 秒未更新")
            return

        samples = self.buffer.window(now, self.stable_window.value())
        meta = self._meta()
        result = evaluate_three_axis_stability(samples, meta, self._stability_settings(), SafetySettings())
        self.window_label.setText(f"目标窗口：{self._yes_no(result.in_window)}")
        self.stable_label.setText(f"稳定状态：{self._yes_no(result.stable)}")
        self.safe_label.setText(f"安全状态：{self._yes_no(result.safe)}")
        self.window_label.setStyleSheet("color: green" if result.in_window else "color: #a66")
        self.stable_label.setStyleSheet("color: green" if result.stable else "color: #a66")
        self.safe_label.setStyleSheet("color: green" if result.safe else "color: red")
        if self.mini45 and self.last_force_time > 0.0 and now - self.last_force_time > 1.0:
            self.mini_status.setText("Mini45 状态：数据超过 1 秒未更新")

    def _meta(self) -> ExperimentMeta:
        if self.force_zero_active:
            note = self.note.text().strip()
            return ExperimentMeta(
                experiment_id=self.experiment_id.text().strip() or "exp001",
                cycle_id=self.current_cycle_id,
                branch="force_zero",
                axis="combined",
                direction="unload",
                preload_n=0.0,
                target_fx=0.0,
                target_fy=0.0,
                target_fz=0.0,
                note=f"{note}; force_zero_unload" if note else "force_zero_unload",
            )
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
        self.force_plot_dirty = True
        self.pending_force_plot_sample = sample

    def _update_force_value_labels(self, sample: ForceSample) -> None:
        for label, attr in (("Fx", "fx"), ("Fy", "fy"), ("Fz", "fz"), ("Mx", "mx"), ("My", "my"), ("Mz", "mz")):
            self.value_labels[label].setText(f"{getattr(sample, attr):.4f}")

    def _add_cap_plot(self, sample: CapSample) -> None:
        x = sample.monotonic_s
        self.cap_x.append(x)
        for key in self.cap_y:
            self.cap_y[key].append(getattr(sample, key))
        self._trim_plot(self.cap_x, self.cap_y)
        self.cap_plot_dirty = True
        self.pending_cap_plot_sample = sample

    def _flush_plot_updates(self, tick_started_s: float | None = None) -> None:
        now_wall_s = time.perf_counter()
        if tick_started_s is not None and now_wall_s - tick_started_s >= self.plot_tick_budget_s:
            return
        if now_wall_s - self.last_plot_flush_wall_s < self.plot_flush_interval_s:
            return

        flushed = False
        if self.force_plot_dirty:
            for key, curve in self.force_curves.items():
                curve.setData(self.force_x, self.force_y[key])
            if self.pending_force_plot_sample is not None:
                self._update_force_value_labels(self.pending_force_plot_sample)
            self.force_plot_dirty = False
            self.pending_force_plot_sample = None
            flushed = True

        if self.cap_plot_dirty:
            for key, curve in self.cap_curves.items():
                curve.setData(self.cap_x, self.cap_y[key])
            if self.pending_cap_plot_sample is not None:
                self._update_cap_value_labels(self.pending_cap_plot_sample)
            self.cap_plot_dirty = False
            self.pending_cap_plot_sample = None
            flushed = True

        if flushed:
            self.last_plot_flush_wall_s = now_wall_s

    def _update_cap_value_labels(self, sample: CapSample) -> None:
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
            elif item.endswith(" capacitance p95-p5 too high"):
                translated.append(item.replace(" capacitance p95-p5 too high", " 电容P95-P5波动过大"))
            elif item.endswith(" capacitance std too high"):
                translated.append(item.replace(" capacitance std too high", " 电容标准差过大"))
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
