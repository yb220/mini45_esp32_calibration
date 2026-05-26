from __future__ import annotations

import csv
from dataclasses import asdict
from pathlib import Path
from typing import Optional

from .models import CalibrationPoint, CombinedSnapshot, ExperimentMeta, utc_timestamp


RAW_FIELDS = [
    "timestamp",
    "monotonic_s",
    "source",
    "fx",
    "fy",
    "fz",
    "mx",
    "my",
    "mz",
    "mini45_raw_fx",
    "mini45_raw_fy",
    "mini45_raw_fz",
    "mini45_raw_mx",
    "mini45_raw_my",
    "mini45_raw_mz",
    "c0",
    "c1",
    "c2",
    "c3",
    "c4",
    "mini45_sequence",
    "mini45_status",
    "esp_ms",
    "esp_sequence",
]

MARKER_FIELDS = [
    "timestamp",
    "marker_id",
    "experiment_id",
    "cycle_id",
    "branch",
    "axis",
    "direction",
    "preload_N",
    "target_Fx",
    "target_Fy",
    "target_Fz",
    "note",
]

TRAINING_MARKER_FIELDS = [
    "timestamp",
    "marker_id",
    "experiment_id",
    "cycle_id",
    "trajectory_type",
    "phase",
    "axis",
    "direction",
    "branch",
    "target_Fx",
    "target_Fy",
    "target_Fz",
    "target_shear_N",
    "target_angle_deg",
    "note",
]

CALIBRATION_FIELDS = [
    "timestamp_start",
    "timestamp_end",
    "experiment_id",
    "cycle_id",
    "branch",
    "axis",
    "direction",
    "preload_N",
    "target_Fx",
    "target_Fy",
    "target_Fz",
    "Fx_mean",
    "Fy_mean",
    "Fz_mean",
    "Mx_mean",
    "My_mean",
    "Mz_mean",
    "Fx_std",
    "Fy_std",
    "Fz_std",
    "C0_mean",
    "C1_mean",
    "C2_mean",
    "C3_mean",
    "C4_mean",
    "C0_std",
    "C1_std",
    "C2_std",
    "C3_std",
    "C4_std",
    "marker_id",
    "valid",
    "reject_reason",
    "note",
]

FORCE_CONTROL_K_FIELDS = [
    "timestamp",
    "experiment_id",
    "valid",
    "reject_reason",
    "debug",
    "delta_X_mm",
    "delta_Y_mm",
    "delta_Z_mm",
    "wait_s",
    "sample_window_s",
    "noise_norm",
    "condition",
    "singular_1",
    "singular_2",
    "singular_3",
    "K_Fx_X",
    "K_Fx_Y",
    "K_Fx_Z",
    "K_Fy_X",
    "K_Fy_Y",
    "K_Fy_Z",
    "K_Fz_X",
    "K_Fz_Y",
    "K_Fz_Z",
    "before_X_Fx",
    "before_X_Fy",
    "before_X_Fz",
    "after_X_Fx",
    "after_X_Fy",
    "after_X_Fz",
    "before_Y_Fx",
    "before_Y_Fy",
    "before_Y_Fz",
    "after_Y_Fx",
    "after_Y_Fy",
    "after_Y_Fz",
    "before_Z_Fx",
    "before_Z_Fy",
    "before_Z_Fz",
    "after_Z_Fx",
    "after_Z_Fy",
    "after_Z_Fz",
]

FORCE_CONTROL_LOG_FIELDS = [
    "timestamp",
    "experiment_id",
    "cycle_id",
    "target_Fx",
    "target_Fy",
    "target_Fz",
    "current_Fx",
    "current_Fy",
    "current_Fz",
    "error_Fx",
    "error_Fy",
    "error_Fz",
    "delta_X_mm",
    "delta_Y_mm",
    "delta_Z_mm",
    "pulses_X",
    "pulses_Y",
    "pulses_Z",
    "damping_eta",
    "trust_scale",
    "condition",
    "predicted_dFx",
    "predicted_dFy",
    "predicted_dFz",
    "note",
]

FORCE_FRAME_MAPPING_FIELDS = [
    "timestamp",
    "experiment_id",
    "sensor_Fx_from",
    "sensor_Fx_sign",
    "sensor_Fy_from",
    "sensor_Fy_sign",
    "sensor_Fz_from",
    "sensor_Fz_sign",
]

class CsvRecorder:
    def __init__(self, output_dir: Path):
        self.output_dir = output_dir
        self.raw_file = None
        self.marker_file = None
        self.cal_file = None
        self.zero_file = None
        self.training_raw_file = None
        self.training_marker_file = None
        self.force_control_k_file = None
        self.force_control_log_file = None
        self.force_frame_mapping_file = None
        self.raw_writer: Optional[csv.DictWriter] = None
        self.marker_writer: Optional[csv.DictWriter] = None
        self.cal_writer: Optional[csv.DictWriter] = None
        self.zero_writer: Optional[csv.DictWriter] = None
        self.training_raw_writer: Optional[csv.DictWriter] = None
        self.training_marker_writer: Optional[csv.DictWriter] = None
        self.force_control_k_writer: Optional[csv.DictWriter] = None
        self.force_control_log_writer: Optional[csv.DictWriter] = None
        self.force_frame_mapping_writer: Optional[csv.DictWriter] = None
        self.zero_drift_index = 0
        self.active_zero_path: Optional[Path] = None

    def start(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.raw_file = (self.output_dir / "raw_timeseries.csv").open("w", newline="", encoding="utf-8-sig")
        self.marker_file = (self.output_dir / "markers.csv").open("w", newline="", encoding="utf-8-sig")
        self.cal_file = (self.output_dir / "calibration_points.csv").open("w", newline="", encoding="utf-8-sig")
        self.training_raw_file = (self.output_dir / "training_raw_timeseries.csv").open("w", newline="", encoding="utf-8-sig")
        self.training_marker_file = (self.output_dir / "training_markers.csv").open("w", newline="", encoding="utf-8-sig")
        self.force_control_k_file = (self.output_dir / "force_control_k.csv").open("w", newline="", encoding="utf-8-sig")
        self.force_control_log_file = (self.output_dir / "force_control_log.csv").open("w", newline="", encoding="utf-8-sig")
        self.force_frame_mapping_file = (self.output_dir / "force_frame_mapping.csv").open("w", newline="", encoding="utf-8-sig")
        self.raw_writer = csv.DictWriter(self.raw_file, fieldnames=RAW_FIELDS)
        self.marker_writer = csv.DictWriter(self.marker_file, fieldnames=MARKER_FIELDS)
        self.cal_writer = csv.DictWriter(self.cal_file, fieldnames=CALIBRATION_FIELDS)
        self.training_raw_writer = csv.DictWriter(self.training_raw_file, fieldnames=RAW_FIELDS)
        self.training_marker_writer = csv.DictWriter(self.training_marker_file, fieldnames=TRAINING_MARKER_FIELDS)
        self.force_control_k_writer = csv.DictWriter(self.force_control_k_file, fieldnames=FORCE_CONTROL_K_FIELDS)
        self.force_control_log_writer = csv.DictWriter(self.force_control_log_file, fieldnames=FORCE_CONTROL_LOG_FIELDS)
        self.force_frame_mapping_writer = csv.DictWriter(self.force_frame_mapping_file, fieldnames=FORCE_FRAME_MAPPING_FIELDS)
        self.raw_writer.writeheader()
        self.marker_writer.writeheader()
        self.cal_writer.writeheader()
        self.training_raw_writer.writeheader()
        self.training_marker_writer.writeheader()
        self.force_control_k_writer.writeheader()
        self.force_control_log_writer.writeheader()
        self.force_frame_mapping_writer.writeheader()

    def stop(self) -> None:
        self.stop_zero_drift_timeseries()
        for file_obj in (
            self.raw_file,
            self.marker_file,
            self.cal_file,
            self.training_raw_file,
            self.training_marker_file,
            self.force_control_k_file,
            self.force_control_log_file,
            self.force_frame_mapping_file,
        ):
            if file_obj:
                file_obj.flush()
                file_obj.close()
        self.raw_file = self.marker_file = self.cal_file = self.training_raw_file = self.training_marker_file = None
        self.force_control_k_file = self.force_control_log_file = None
        self.force_frame_mapping_file = None
        self.raw_writer = self.marker_writer = self.cal_writer = self.training_raw_writer = self.training_marker_writer = None
        self.force_control_k_writer = self.force_control_log_writer = self.force_frame_mapping_writer = None

    def write_raw(self, snapshot: CombinedSnapshot) -> None:
        if not self.raw_writer:
            return
        row = {field: snapshot.to_row().get(field, "") for field in RAW_FIELDS}
        self.raw_writer.writerow(row)
        if self.raw_file:
            self.raw_file.flush()

    def write_marker(self, marker_id: int, meta: ExperimentMeta) -> None:
        if not self.marker_writer:
            return
        self.marker_writer.writerow(
            {
                "timestamp": utc_timestamp(),
                "marker_id": marker_id,
                "experiment_id": meta.experiment_id,
                "cycle_id": meta.cycle_id,
                "branch": meta.branch,
                "axis": meta.axis,
                "direction": meta.direction,
                "preload_N": meta.preload_n,
                "target_Fx": meta.target_fx,
                "target_Fy": meta.target_fy,
                "target_Fz": meta.target_fz,
                "note": meta.note,
            }
        )
        if self.marker_file:
            self.marker_file.flush()

    def write_calibration_point(self, point: CalibrationPoint) -> None:
        if not self.cal_writer:
            return
        row = {field: point.to_row().get(field, "") for field in CALIBRATION_FIELDS}
        self.cal_writer.writerow(row)
        if self.cal_file:
            self.cal_file.flush()

    def start_zero_drift_timeseries(self) -> Path:
        if self.zero_writer:
            self.stop_zero_drift_timeseries()
        self.zero_drift_index += 1
        path = self.output_dir / f"zero_drift_timeseries_{self.zero_drift_index:03d}.csv"
        self.zero_file = path.open("w", newline="", encoding="utf-8-sig")
        self.zero_writer = csv.DictWriter(self.zero_file, fieldnames=RAW_FIELDS)
        self.zero_writer.writeheader()
        self.active_zero_path = path
        return path

    def write_zero_drift_raw(self, snapshot: CombinedSnapshot) -> None:
        if not self.zero_writer:
            return
        row = {field: snapshot.to_row().get(field, "") for field in RAW_FIELDS}
        self.zero_writer.writerow(row)
        if self.zero_file:
            self.zero_file.flush()

    def stop_zero_drift_timeseries(self) -> None:
        if self.zero_file:
            self.zero_file.flush()
            self.zero_file.close()
        self.zero_file = None
        self.zero_writer = None
        self.active_zero_path = None

    def start_training_files(self) -> None:
        if self.training_raw_writer and self.training_marker_writer:
            return
        self.output_dir.mkdir(parents=True, exist_ok=True)
        raw_path = self.output_dir / "training_raw_timeseries.csv"
        marker_path = self.output_dir / "training_markers.csv"
        raw_exists = raw_path.exists() and raw_path.stat().st_size > 0
        marker_exists = marker_path.exists() and marker_path.stat().st_size > 0
        self.training_raw_file = raw_path.open("a", newline="", encoding="utf-8-sig")
        self.training_marker_file = marker_path.open("a", newline="", encoding="utf-8-sig")
        self.training_raw_writer = csv.DictWriter(self.training_raw_file, fieldnames=RAW_FIELDS)
        self.training_marker_writer = csv.DictWriter(self.training_marker_file, fieldnames=TRAINING_MARKER_FIELDS)
        if not raw_exists:
            self.training_raw_writer.writeheader()
        if not marker_exists:
            self.training_marker_writer.writeheader()

    def write_training_raw(self, snapshot: CombinedSnapshot) -> None:
        if not self.training_raw_writer:
            return
        row = {field: snapshot.to_row().get(field, "") for field in RAW_FIELDS}
        self.training_raw_writer.writerow(row)
        if self.training_raw_file:
            self.training_raw_file.flush()

    def write_training_marker(
        self,
        marker_id: int,
        meta: ExperimentMeta,
        trajectory_type: str,
        phase: str,
        target_shear_n: float | str = "",
        target_angle_deg: float | str = "",
    ) -> None:
        if not self.training_marker_writer:
            return
        self.training_marker_writer.writerow(
            {
                "timestamp": utc_timestamp(),
                "marker_id": marker_id,
                "experiment_id": meta.experiment_id,
                "cycle_id": meta.cycle_id,
                "trajectory_type": trajectory_type,
                "phase": phase,
                "axis": meta.axis,
                "direction": meta.direction,
                "branch": meta.branch,
                "target_Fx": meta.target_fx,
                "target_Fy": meta.target_fy,
                "target_Fz": meta.target_fz,
                "target_shear_N": target_shear_n,
                "target_angle_deg": target_angle_deg,
                "note": meta.note,
            }
        )
        if self.training_marker_file:
            self.training_marker_file.flush()

    def stop_training_files(self) -> None:
        for file_obj in (self.training_raw_file, self.training_marker_file):
            if file_obj:
                file_obj.flush()

    def write_force_control_k(self, row: dict) -> None:
        if not self.force_control_k_writer:
            return
        out = {field: row.get(field, "") for field in FORCE_CONTROL_K_FIELDS}
        out["timestamp"] = out["timestamp"] or utc_timestamp()
        self.force_control_k_writer.writerow(out)
        if self.force_control_k_file:
            self.force_control_k_file.flush()

    def write_force_control_log(self, row: dict) -> None:
        if not self.force_control_log_writer:
            return
        out = {field: row.get(field, "") for field in FORCE_CONTROL_LOG_FIELDS}
        out["timestamp"] = out["timestamp"] or utc_timestamp()
        self.force_control_log_writer.writerow(out)
        if self.force_control_log_file:
            self.force_control_log_file.flush()

    def write_force_frame_mapping(self, row: dict) -> None:
        if not self.force_frame_mapping_writer:
            return
        out = {field: row.get(field, "") for field in FORCE_FRAME_MAPPING_FIELDS}
        out["timestamp"] = out["timestamp"] or utc_timestamp()
        self.force_frame_mapping_writer.writerow(out)
        if self.force_frame_mapping_file:
            self.force_frame_mapping_file.flush()

    def __enter__(self) -> "CsvRecorder":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()
