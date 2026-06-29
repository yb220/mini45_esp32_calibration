from __future__ import annotations

import csv
import json
import queue
import threading
import time
from pathlib import Path
from typing import Any, Optional

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
    "cap_profile",
    "mc1081_cnt",
    "mc1081_cavg",
    "cap_nominal_hz",
    "cap_effective_hz",
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
    "Fx_trimmed_mean",
    "Fy_trimmed_mean",
    "Fz_trimmed_mean",
    "Mx_trimmed_mean",
    "My_trimmed_mean",
    "Mz_trimmed_mean",
    "C0_trimmed_mean",
    "C1_trimmed_mean",
    "C2_trimmed_mean",
    "C3_trimmed_mean",
    "C4_trimmed_mean",
    "cap_sample_count",
    "force_sample_count",
    "marker_id",
    "valid",
    "reject_reason",
    "note",
]

RETEST_PREFIX_FIELDS = [
    "retest_id",
    "source_point_index",
    "source_experiment_id",
    "source_batch_dir",
    "source_cycle_id",
]
RETEST_MARKER_FIELDS = RETEST_PREFIX_FIELDS + MARKER_FIELDS
RETEST_CALIBRATION_FIELDS = RETEST_PREFIX_FIELDS + CALIBRATION_FIELDS

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
RETEST_FORCE_CONTROL_K_FIELDS = ["retest_id"] + FORCE_CONTROL_K_FIELDS
RETEST_FORCE_CONTROL_LOG_FIELDS = ["retest_id", "source_point_index"] + FORCE_CONTROL_LOG_FIELDS

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

WORKFLOW_EVENT_FIELDS = [
    "timestamp",
    "event",
    "stage",
    "status",
    "cap_profile",
    "cnt",
    "cavg",
    "requested_hz",
    "effective_hz",
    "target_index",
    "retry_count",
    "note",
]

class CsvRecorder:
    FLUSH_INTERVAL_S = 1.0
    FLUSH_ROW_INTERVAL = 1000

    def __init__(self, output_dir: Path):
        self.output_dir = output_dir
        self.raw_file = None
        self.marker_file = None
        self.cal_file = None
        self.zero_file = None
        self.training_raw_file = None
        self.training_marker_file = None
        self.training_fast_raw_file = None
        self.training_fast_marker_file = None
        self.force_control_k_file = None
        self.force_control_log_file = None
        self.force_frame_mapping_file = None
        self.workflow_event_file = None
        self.retest_raw_file = None
        self.retest_marker_file = None
        self.retest_cal_file = None
        self.retest_force_control_k_file = None
        self.retest_force_control_log_file = None
        self.raw_writer: Optional[csv.DictWriter] = None
        self.marker_writer: Optional[csv.DictWriter] = None
        self.cal_writer: Optional[csv.DictWriter] = None
        self.zero_writer: Optional[csv.DictWriter] = None
        self.training_raw_writer: Optional[csv.DictWriter] = None
        self.training_marker_writer: Optional[csv.DictWriter] = None
        self.training_fast_raw_writer: Optional[csv.DictWriter] = None
        self.training_fast_marker_writer: Optional[csv.DictWriter] = None
        self.force_control_k_writer: Optional[csv.DictWriter] = None
        self.force_control_log_writer: Optional[csv.DictWriter] = None
        self.force_frame_mapping_writer: Optional[csv.DictWriter] = None
        self.workflow_event_writer: Optional[csv.DictWriter] = None
        self.retest_raw_writer: Optional[csv.DictWriter] = None
        self.retest_marker_writer: Optional[csv.DictWriter] = None
        self.retest_cal_writer: Optional[csv.DictWriter] = None
        self.retest_force_control_k_writer: Optional[csv.DictWriter] = None
        self.retest_force_control_log_writer: Optional[csv.DictWriter] = None
        self.active_training_profile = "TRAINING_BALANCED"
        self.zero_drift_index = 0
        self.active_zero_path: Optional[Path] = None
        self.static_full_retest_active = False
        self.static_full_retest_id = ""
        self.static_full_retest_manifest: dict[str, Any] = {}
        self.static_full_retest_manifest_path: Optional[Path] = None
        self.static_full_retest_source_point_index: int | str = ""
        self.static_full_retest_source_cycle_id = ""
        self._paths: dict[str, Path] = {}
        self._retest_paths: dict[str, Path] = {}
        self._resume = False
        self._pending_flush_rows = 0
        self._last_flush_s = time.monotonic()
        self._write_queue: queue.Queue[tuple[str, Any] | None] = queue.Queue()
        self._worker: threading.Thread | None = None
        self._worker_error: Exception | None = None

    def start(self, resume: bool = False) -> None:
        self._write_queue = queue.Queue()
        self._worker_error = None
        self._resume = bool(resume)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._paths = {
            "raw": self.output_dir / "raw_timeseries.csv",
            "marker": self.output_dir / "markers.csv",
            "cal": self.output_dir / "calibration_points.csv",
            "training_raw": self.output_dir / "training_balanced_raw_timeseries.csv",
            "training_marker": self.output_dir / "training_balanced_markers.csv",
            "training_fast_raw": self.output_dir / "training_fast_raw_timeseries.csv",
            "training_fast_marker": self.output_dir / "training_fast_markers.csv",
            "force_control_k": self.output_dir / "force_control_k.csv",
            "force_control_log": self.output_dir / "force_control_log.csv",
            "force_frame_mapping": self.output_dir / "force_frame_mapping.csv",
            "workflow_event": self.output_dir / "workflow_events.csv",
        }
        self._worker = threading.Thread(target=self._writer_loop, name="csv-recorder-writer", daemon=True)
        self._worker.start()

    def _ensure_writer(
        self,
        paths: dict[str, Path],
        key: str,
        file_attr: str,
        writer_attr: str,
        fieldnames: list[str],
        *,
        resume: bool,
    ) -> Optional[csv.DictWriter]:
        writer = getattr(self, writer_attr)
        if writer:
            return writer
        path = paths.get(key)
        if not path:
            return None
        path.parent.mkdir(parents=True, exist_ok=True)
        has_content = bool(resume and path.exists() and path.stat().st_size > 0)
        file_obj = path.open("a" if has_content else "w", newline="", encoding="utf-8-sig")
        writer = csv.DictWriter(file_obj, fieldnames=fieldnames)
        if not has_content:
            writer.writeheader()
        setattr(self, file_attr, file_obj)
        setattr(self, writer_attr, writer)
        return writer

    def _ensure_main_writer(self, key: str) -> Optional[csv.DictWriter]:
        specs = {
            "raw": ("raw_file", "raw_writer", RAW_FIELDS),
            "marker": ("marker_file", "marker_writer", MARKER_FIELDS),
            "cal": ("cal_file", "cal_writer", CALIBRATION_FIELDS),
            "training_raw": ("training_raw_file", "training_raw_writer", RAW_FIELDS),
            "training_marker": ("training_marker_file", "training_marker_writer", TRAINING_MARKER_FIELDS),
            "training_fast_raw": ("training_fast_raw_file", "training_fast_raw_writer", RAW_FIELDS),
            "training_fast_marker": ("training_fast_marker_file", "training_fast_marker_writer", TRAINING_MARKER_FIELDS),
            "force_control_k": ("force_control_k_file", "force_control_k_writer", FORCE_CONTROL_K_FIELDS),
            "force_control_log": ("force_control_log_file", "force_control_log_writer", FORCE_CONTROL_LOG_FIELDS),
            "force_frame_mapping": ("force_frame_mapping_file", "force_frame_mapping_writer", FORCE_FRAME_MAPPING_FIELDS),
            "workflow_event": ("workflow_event_file", "workflow_event_writer", WORKFLOW_EVENT_FIELDS),
        }
        file_attr, writer_attr, fieldnames = specs[key]
        return self._ensure_writer(
            self._paths,
            key,
            file_attr,
            writer_attr,
            fieldnames,
            resume=self._resume,
        )

    def _ensure_retest_writer(self, key: str) -> Optional[csv.DictWriter]:
        specs = {
            "raw_timeseries": ("retest_raw_file", "retest_raw_writer", RAW_FIELDS),
            "markers": ("retest_marker_file", "retest_marker_writer", RETEST_MARKER_FIELDS),
            "calibration_points": ("retest_cal_file", "retest_cal_writer", RETEST_CALIBRATION_FIELDS),
            "force_control_k": ("retest_force_control_k_file", "retest_force_control_k_writer", RETEST_FORCE_CONTROL_K_FIELDS),
            "force_control_log": ("retest_force_control_log_file", "retest_force_control_log_writer", RETEST_FORCE_CONTROL_LOG_FIELDS),
        }
        file_attr, writer_attr, fieldnames = specs[key]
        return self._ensure_writer(
            self._retest_paths,
            key,
            file_attr,
            writer_attr,
            fieldnames,
            resume=False,
        )

    def start_static_full_retest(
        self,
        *,
        retest_id: str | None = None,
        manifest: dict[str, Any] | None = None,
    ) -> dict[str, Path]:
        if self.static_full_retest_active:
            raise RuntimeError("static full retest is already active")
        self._wait_for_writes()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        retest_id = retest_id or time.strftime("%Y%m%d_%H%M%S")
        stem = f"static_full_retest_{retest_id}"
        paths = {
            "raw_timeseries": self.output_dir / f"{stem}_raw_timeseries.csv",
            "markers": self.output_dir / f"{stem}_markers.csv",
            "calibration_points": self.output_dir / f"{stem}_calibration_points.csv",
            "force_control_k": self.output_dir / f"{stem}_force_control_k.csv",
            "force_control_log": self.output_dir / f"{stem}_force_control_log.csv",
            "manifest": self.output_dir / f"{stem}_manifest.json",
        }
        self._retest_paths = {key: value for key, value in paths.items() if key != "manifest"}
        self.static_full_retest_active = True
        self.static_full_retest_id = retest_id
        self.static_full_retest_source_point_index = ""
        self.static_full_retest_source_cycle_id = ""
        self.static_full_retest_manifest_path = paths["manifest"]
        self.static_full_retest_manifest = dict(manifest or {})
        self.static_full_retest_manifest.update(
            {
                "retest_id": retest_id,
                "status": "running",
                "started_at": utc_timestamp(),
                "output_files": {name: path.name for name, path in paths.items()},
            }
        )
        self._write_static_full_retest_manifest()
        self._flush_files()
        return paths

    def set_static_full_retest_source(
        self,
        *,
        source_point_index: int | str = "",
        source_cycle_id: str = "",
    ) -> None:
        self.static_full_retest_source_point_index = source_point_index
        self.static_full_retest_source_cycle_id = source_cycle_id

    def update_static_full_retest_manifest(self, **values: Any) -> None:
        if not self.static_full_retest_manifest_path:
            return
        self.static_full_retest_manifest.update(values)
        self._write_static_full_retest_manifest()

    def finish_static_full_retest(
        self,
        *,
        status: str,
        reason: str = "",
        completed_points: int | None = None,
        invalid_points: int | None = None,
    ) -> None:
        if not self.static_full_retest_active:
            return
        self._wait_for_writes()
        self.static_full_retest_manifest.update(
            {
                "status": status,
                "finished_at": utc_timestamp(),
                "reason": reason,
            }
        )
        if completed_points is not None:
            self.static_full_retest_manifest["completed_points"] = completed_points
        if invalid_points is not None:
            self.static_full_retest_manifest["invalid_points"] = invalid_points
        self._write_static_full_retest_manifest()
        for file_obj in (
            self.retest_raw_file,
            self.retest_marker_file,
            self.retest_cal_file,
            self.retest_force_control_k_file,
            self.retest_force_control_log_file,
        ):
            if file_obj:
                file_obj.flush()
                file_obj.close()
        self.retest_raw_file = self.retest_marker_file = self.retest_cal_file = None
        self.retest_force_control_k_file = self.retest_force_control_log_file = None
        self.retest_raw_writer = self.retest_marker_writer = self.retest_cal_writer = None
        self.retest_force_control_k_writer = self.retest_force_control_log_writer = None
        self.static_full_retest_active = False
        self.static_full_retest_id = ""
        self.static_full_retest_source_point_index = ""
        self.static_full_retest_source_cycle_id = ""
        self._retest_paths = {}

    def _write_static_full_retest_manifest(self) -> None:
        if not self.static_full_retest_manifest_path:
            return
        self.static_full_retest_manifest_path.write_text(
            json.dumps(self.static_full_retest_manifest, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    def stop(self) -> None:
        if self.static_full_retest_active:
            self.finish_static_full_retest(status="closed", reason="recorder stopped")
        self.stop_zero_drift_timeseries()
        self._wait_for_writes()
        if self._worker:
            self._write_queue.put(None)
            self._worker.join(timeout=5.0)
            self._worker = None
        self.flush()
        for file_obj in (
            self.raw_file,
            self.marker_file,
            self.cal_file,
            self.training_raw_file,
            self.training_marker_file,
            self.training_fast_raw_file,
            self.training_fast_marker_file,
            self.force_control_k_file,
            self.force_control_log_file,
            self.force_frame_mapping_file,
            self.workflow_event_file,
            self.retest_raw_file,
            self.retest_marker_file,
            self.retest_cal_file,
            self.retest_force_control_k_file,
            self.retest_force_control_log_file,
        ):
            if file_obj:
                file_obj.flush()
                file_obj.close()
        self.raw_file = self.marker_file = self.cal_file = self.training_raw_file = self.training_marker_file = None
        self.training_fast_raw_file = self.training_fast_marker_file = None
        self.force_control_k_file = self.force_control_log_file = None
        self.force_frame_mapping_file = None
        self.workflow_event_file = None
        self.raw_writer = self.marker_writer = self.cal_writer = self.training_raw_writer = self.training_marker_writer = None
        self.training_fast_raw_writer = self.training_fast_marker_writer = None
        self.force_control_k_writer = self.force_control_log_writer = self.force_frame_mapping_writer = None
        self.workflow_event_writer = None
        self.retest_raw_file = self.retest_marker_file = self.retest_cal_file = None
        self.retest_force_control_k_file = self.retest_force_control_log_file = None
        self.retest_raw_writer = self.retest_marker_writer = self.retest_cal_writer = None
        self.retest_force_control_k_writer = self.retest_force_control_log_writer = None
        self._paths = {}
        self._retest_paths = {}
        self._resume = False
        self._pending_flush_rows = 0

    def _open_files(self):
        return (
            self.raw_file,
            self.marker_file,
            self.cal_file,
            self.zero_file,
            self.training_raw_file,
            self.training_marker_file,
            self.training_fast_raw_file,
            self.training_fast_marker_file,
            self.force_control_k_file,
            self.force_control_log_file,
            self.force_frame_mapping_file,
            self.workflow_event_file,
            self.retest_raw_file,
            self.retest_marker_file,
            self.retest_cal_file,
            self.retest_force_control_k_file,
            self.retest_force_control_log_file,
        )

    def flush(self) -> None:
        self._wait_for_writes()
        self._flush_files()

    def _flush_files(self) -> None:
        for file_obj in self._open_files():
            if file_obj:
                file_obj.flush()
        self._pending_flush_rows = 0
        self._last_flush_s = time.monotonic()

    def _wait_for_writes(self) -> None:
        if self._worker and threading.current_thread() is not self._worker:
            self._write_queue.join()

    def _mark_dirty(self, *, force: bool = False) -> None:
        if force:
            self._flush_files()
            return
        self._pending_flush_rows += 1
        now = time.monotonic()
        if self._pending_flush_rows >= self.FLUSH_ROW_INTERVAL or now - self._last_flush_s >= self.FLUSH_INTERVAL_S:
            self._flush_files()

    def _enqueue(self, kind: str, payload: Any) -> None:
        if self._worker_error is not None:
            return
        if not self._worker:
            self._write_task(kind, payload)
            return
        self._write_queue.put((kind, payload))

    def _writer_loop(self) -> None:
        while True:
            task = self._write_queue.get()
            try:
                if task is None:
                    return
                kind, payload = task
                self._write_task(kind, payload)
            except Exception as exc:  # pragma: no cover - defensive for runtime I/O errors
                self._worker_error = exc
            finally:
                self._write_queue.task_done()

    def _snapshot_row(self, snapshot: CombinedSnapshot) -> dict:
        source = snapshot.to_row()
        return {field: source.get(field, "") for field in RAW_FIELDS}

    def _retest_prefix_row(self, source_cycle_id: str = "") -> dict:
        manifest = self.static_full_retest_manifest
        return {
            "retest_id": self.static_full_retest_id,
            "source_point_index": self.static_full_retest_source_point_index,
            "source_experiment_id": manifest.get("source_experiment_id", ""),
            "source_batch_dir": manifest.get("source_batch_dir", str(self.output_dir)),
            "source_cycle_id": source_cycle_id or self.static_full_retest_source_cycle_id,
        }

    def _write_task(self, kind: str, payload: Any) -> None:
        if kind == "raw":
            writer = self._ensure_main_writer("raw")
            if not writer:
                return
            writer.writerow(self._snapshot_row(payload))
            self._mark_dirty()
        elif kind == "retest_raw":
            writer = self._ensure_retest_writer("raw_timeseries")
            if not writer:
                return
            writer.writerow(self._snapshot_row(payload))
            self._mark_dirty()
        elif kind == "zero" and self.zero_writer:
            self.zero_writer.writerow(self._snapshot_row(payload))
            self._mark_dirty()
        elif kind == "training_raw":
            writer = self._ensure_main_writer("training_raw")
            if not writer:
                return
            writer.writerow(self._snapshot_row(payload))
            self._mark_dirty()
        elif kind == "marker":
            writer = self._ensure_main_writer("marker")
            if not writer:
                return
            writer.writerow(payload)
            self._mark_dirty(force=True)
        elif kind == "retest_marker":
            writer = self._ensure_retest_writer("markers")
            if not writer:
                return
            out = {field: payload.get(field, "") for field in MARKER_FIELDS}
            row = self._retest_prefix_row(str(out.get("cycle_id") or ""))
            row.update(out)
            writer.writerow({field: row.get(field, "") for field in RETEST_MARKER_FIELDS})
            self._mark_dirty(force=True)
        elif kind == "calibration":
            writer = self._ensure_main_writer("cal")
            if not writer:
                return
            row = {field: payload.to_row().get(field, "") for field in CALIBRATION_FIELDS}
            writer.writerow(row)
            self._mark_dirty(force=True)
        elif kind == "retest_calibration":
            writer = self._ensure_retest_writer("calibration_points")
            if not writer:
                return
            row = {field: payload.to_row().get(field, "") for field in CALIBRATION_FIELDS}
            prefix = self._retest_prefix_row(str(row.get("cycle_id") or ""))
            prefix.update(row)
            writer.writerow({field: prefix.get(field, "") for field in RETEST_CALIBRATION_FIELDS})
            self._mark_dirty(force=True)
        elif kind == "training_marker":
            writer = self._ensure_main_writer("training_marker")
            if not writer:
                return
            writer.writerow(payload)
            self._mark_dirty(force=True)
        elif kind == "training_fast_raw":
            writer = self._ensure_main_writer("training_fast_raw")
            if not writer:
                return
            writer.writerow(self._snapshot_row(payload))
            self._mark_dirty()
        elif kind == "training_fast_marker":
            writer = self._ensure_main_writer("training_fast_marker")
            if not writer:
                return
            writer.writerow(payload)
            self._mark_dirty(force=True)
        elif kind == "force_control_k":
            writer = self._ensure_main_writer("force_control_k")
            if not writer:
                return
            out = {field: payload.get(field, "") for field in FORCE_CONTROL_K_FIELDS}
            out["timestamp"] = out["timestamp"] or utc_timestamp()
            writer.writerow(out)
            self._mark_dirty(force=True)
        elif kind == "retest_force_control_k":
            writer = self._ensure_retest_writer("force_control_k")
            if not writer:
                return
            out = {field: payload.get(field, "") for field in FORCE_CONTROL_K_FIELDS}
            out["timestamp"] = out["timestamp"] or utc_timestamp()
            row = {"retest_id": self.static_full_retest_id, **out}
            writer.writerow({field: row.get(field, "") for field in RETEST_FORCE_CONTROL_K_FIELDS})
            self._mark_dirty(force=True)
        elif kind == "force_control_log":
            writer = self._ensure_main_writer("force_control_log")
            if not writer:
                return
            out = {field: payload.get(field, "") for field in FORCE_CONTROL_LOG_FIELDS}
            out["timestamp"] = out["timestamp"] or utc_timestamp()
            writer.writerow(out)
            self._mark_dirty()
        elif kind == "retest_force_control_log":
            writer = self._ensure_retest_writer("force_control_log")
            if not writer:
                return
            out = {field: payload.get(field, "") for field in FORCE_CONTROL_LOG_FIELDS}
            out["timestamp"] = out["timestamp"] or utc_timestamp()
            row = {
                "retest_id": self.static_full_retest_id,
                "source_point_index": self.static_full_retest_source_point_index,
                **out,
            }
            writer.writerow({field: row.get(field, "") for field in RETEST_FORCE_CONTROL_LOG_FIELDS})
            self._mark_dirty()
        elif kind == "force_frame_mapping":
            writer = self._ensure_main_writer("force_frame_mapping")
            if not writer:
                return
            out = {field: payload.get(field, "") for field in FORCE_FRAME_MAPPING_FIELDS}
            out["timestamp"] = out["timestamp"] or utc_timestamp()
            writer.writerow(out)
            self._mark_dirty(force=True)
        elif kind == "workflow_event":
            writer = self._ensure_main_writer("workflow_event")
            if not writer:
                return
            out = {field: payload.get(field, "") for field in WORKFLOW_EVENT_FIELDS}
            out["timestamp"] = out["timestamp"] or utc_timestamp()
            writer.writerow(out)
            self._mark_dirty(force=True)

    def write_raw(self, snapshot: CombinedSnapshot) -> None:
        if self.static_full_retest_active:
            self._enqueue("retest_raw", snapshot)
            return
        if not self._paths:
            return
        self._enqueue("raw", snapshot)

    def write_marker(self, marker_id: int, meta: ExperimentMeta) -> None:
        if self.static_full_retest_active:
            kind = "retest_marker"
        elif self._paths:
            kind = "marker"
        else:
            return
        self._enqueue(
            kind,
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
            },
        )

    def write_calibration_point(self, point: CalibrationPoint) -> None:
        if self.static_full_retest_active:
            self._enqueue("retest_calibration", point)
            return
        if not self._paths:
            return
        self._enqueue("calibration", point)

    def start_zero_drift_timeseries(self) -> Path:
        if self.zero_writer:
            self.stop_zero_drift_timeseries()
        self._wait_for_writes()
        self.zero_drift_index += 1
        path = self.output_dir / f"zero_drift_timeseries_{self.zero_drift_index:03d}.csv"
        self.zero_file = path.open("w", newline="", encoding="utf-8-sig")
        self.zero_writer = csv.DictWriter(self.zero_file, fieldnames=RAW_FIELDS)
        self.zero_writer.writeheader()
        self.active_zero_path = path
        self._flush_files()
        return path

    def write_zero_drift_raw(self, snapshot: CombinedSnapshot) -> None:
        if not self.zero_writer:
            return
        self._enqueue("zero", snapshot)

    def stop_zero_drift_timeseries(self) -> None:
        self._wait_for_writes()
        if self.zero_file:
            self.zero_file.flush()
            self.zero_file.close()
        self.zero_file = None
        self.zero_writer = None
        self.active_zero_path = None

    def start_training_files(self, profile: str = "TRAINING_BALANCED") -> None:
        normalized = str(profile).strip().upper()
        if normalized not in {"TRAINING_BALANCED", "TRAINING_FAST"}:
            raise ValueError(f"不支持的训练采集配置：{profile}")
        self.active_training_profile = normalized
        self._wait_for_writes()
        self._flush_files()

    def write_training_raw(self, snapshot: CombinedSnapshot, profile: str | None = None) -> None:
        if not self._paths:
            return
        normalized = str(profile or self.active_training_profile).strip().upper()
        kind = "training_fast_raw" if normalized == "TRAINING_FAST" else "training_raw"
        self._enqueue(kind, snapshot)

    def write_training_marker(
        self,
        marker_id: int,
        meta: ExperimentMeta,
        trajectory_type: str,
        phase: str,
        target_shear_n: float | str = "",
        target_angle_deg: float | str = "",
        profile: str | None = None,
    ) -> None:
        if not self._paths:
            return
        normalized = str(profile or self.active_training_profile).strip().upper()
        self._enqueue(
            "training_fast_marker" if normalized == "TRAINING_FAST" else "training_marker",
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
            },
        )

    def stop_training_files(self) -> None:
        self._wait_for_writes()
        for file_obj in (
            self.training_raw_file,
            self.training_marker_file,
            self.training_fast_raw_file,
            self.training_fast_marker_file,
        ):
            if file_obj:
                file_obj.flush()
        self._pending_flush_rows = 0
        self._last_flush_s = time.monotonic()

    def write_force_control_k(self, row: dict) -> None:
        if self.static_full_retest_active:
            self._enqueue("retest_force_control_k", dict(row))
            return
        if not self._paths:
            return
        self._enqueue("force_control_k", dict(row))

    def write_force_control_log(self, row: dict) -> None:
        if self.static_full_retest_active:
            self._enqueue("retest_force_control_log", dict(row))
            return
        if not self._paths:
            return
        self._enqueue("force_control_log", dict(row))

    def write_force_frame_mapping(self, row: dict) -> None:
        if not self._paths:
            return
        self._enqueue("force_frame_mapping", dict(row))

    def write_workflow_event(self, row: dict) -> None:
        if not self._paths:
            return
        self._enqueue("workflow_event", dict(row))

    def __enter__(self) -> "CsvRecorder":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()
