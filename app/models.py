from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from typing import Optional


FORCE_FIELDS = ("fx", "fy", "fz", "mx", "my", "mz")
CAP_FIELDS = ("c0", "c1", "c2", "c3", "c4")


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")


@dataclass
class ForceSample:
    timestamp: str
    monotonic_s: float
    fx: float
    fy: float
    fz: float
    mx: float
    my: float
    mz: float
    sequence: Optional[int] = None
    status: Optional[int] = None
    source: str = "mini45"


@dataclass
class CapSample:
    timestamp: str
    monotonic_s: float
    c0: float
    c1: float
    c2: float
    c3: float
    c4: float
    esp_ms: Optional[int] = None
    sequence: Optional[int] = None
    cap_profile: str = ""
    mc1081_cnt: Optional[int] = None
    mc1081_cavg: Optional[int] = None
    cap_nominal_hz: Optional[float] = None
    cap_effective_hz: Optional[float] = None
    source: str = "esp32"


@dataclass
class CombinedSnapshot:
    timestamp: str
    monotonic_s: float
    source: str
    fx: Optional[float] = None
    fy: Optional[float] = None
    fz: Optional[float] = None
    mx: Optional[float] = None
    my: Optional[float] = None
    mz: Optional[float] = None
    mini45_raw_fx: Optional[float] = None
    mini45_raw_fy: Optional[float] = None
    mini45_raw_fz: Optional[float] = None
    mini45_raw_mx: Optional[float] = None
    mini45_raw_my: Optional[float] = None
    mini45_raw_mz: Optional[float] = None
    c0: Optional[float] = None
    c1: Optional[float] = None
    c2: Optional[float] = None
    c3: Optional[float] = None
    c4: Optional[float] = None
    mini45_sequence: Optional[int] = None
    mini45_status: Optional[int] = None
    esp_ms: Optional[int] = None
    esp_sequence: Optional[int] = None
    cap_profile: str = ""
    mc1081_cnt: Optional[int] = None
    mc1081_cavg: Optional[int] = None
    cap_nominal_hz: Optional[float] = None
    cap_effective_hz: Optional[float] = None

    @classmethod
    def from_force(cls, sample: ForceSample, raw_sample: ForceSample | None = None) -> "CombinedSnapshot":
        raw = raw_sample or sample
        return cls(
            timestamp=sample.timestamp,
            monotonic_s=sample.monotonic_s,
            source=sample.source,
            fx=sample.fx,
            fy=sample.fy,
            fz=sample.fz,
            mx=sample.mx,
            my=sample.my,
            mz=sample.mz,
            mini45_raw_fx=raw.fx,
            mini45_raw_fy=raw.fy,
            mini45_raw_fz=raw.fz,
            mini45_raw_mx=raw.mx,
            mini45_raw_my=raw.my,
            mini45_raw_mz=raw.mz,
            mini45_sequence=sample.sequence,
            mini45_status=sample.status,
        )

    @classmethod
    def from_cap(cls, sample: CapSample) -> "CombinedSnapshot":
        return cls(
            timestamp=sample.timestamp,
            monotonic_s=sample.monotonic_s,
            source=sample.source,
            c0=sample.c0,
            c1=sample.c1,
            c2=sample.c2,
            c3=sample.c3,
            c4=sample.c4,
            esp_ms=sample.esp_ms,
            esp_sequence=sample.sequence,
            cap_profile=sample.cap_profile,
            mc1081_cnt=sample.mc1081_cnt,
            mc1081_cavg=sample.mc1081_cavg,
            cap_nominal_hz=sample.cap_nominal_hz,
            cap_effective_hz=sample.cap_effective_hz,
        )

    def to_row(self) -> dict:
        return asdict(self)


@dataclass
class ExperimentMeta:
    experiment_id: str = "exp001"
    cycle_id: str = "cycle001"
    branch: str = "loading"
    axis: str = "Fz"
    direction: str = "none"
    preload_n: float = 0.0
    target_fx: float = 0.0
    target_fy: float = 0.0
    target_fz: float = 0.0
    note: str = ""


@dataclass
class StabilitySettings:
    stable_window_s: float = 2.0
    hold_window_s: float = 5.0
    target_force_std_max_n: float = 0.03
    target_force_std_percent_fs: float = 0.01
    cross_axis_ratio_max: float = 0.10
    capacitance_p95p5_max_pf: float = 0.06
    capacitance_std_max_pf: float = 0.02
    torque_abs_max: float = 1.0
    tolerance_fx: float = 0.10
    tolerance_fy: float = 0.10
    tolerance_fz: float = 0.10
    fs_fx: float = 4.0
    fs_fy: float = 4.0
    fs_fz: float = 10.0


@dataclass
class SafetySettings:
    fx_abs_max_n: float = 4.0
    fy_abs_max_n: float = 4.0
    fz_abs_max_n: float = 10.0
    stale_timeout_s: float = 1.0


@dataclass
class StabilityResult:
    in_window: bool
    stable: bool
    safe: bool
    reject_reason: str
    target_axis: str
    target_value: float
    measured_value: Optional[float]
    force_std: Optional[float]


@dataclass
class CalibrationPoint:
    timestamp_start: str
    timestamp_end: str
    experiment_id: str
    cycle_id: str
    branch: str
    axis: str
    direction: str
    preload_N: float
    target_Fx: float
    target_Fy: float
    target_Fz: float
    Fx_mean: float
    Fy_mean: float
    Fz_mean: float
    Mx_mean: float
    My_mean: float
    Mz_mean: float
    Fx_std: float
    Fy_std: float
    Fz_std: float
    C0_mean: float
    C1_mean: float
    C2_mean: float
    C3_mean: float
    C4_mean: float
    C0_std: float
    C1_std: float
    C2_std: float
    C3_std: float
    C4_std: float
    marker_id: int
    valid: bool
    reject_reason: str
    note: str
    Fx_trimmed_mean: float = float("nan")
    Fy_trimmed_mean: float = float("nan")
    Fz_trimmed_mean: float = float("nan")
    Mx_trimmed_mean: float = float("nan")
    My_trimmed_mean: float = float("nan")
    Mz_trimmed_mean: float = float("nan")
    C0_trimmed_mean: float = float("nan")
    C1_trimmed_mean: float = float("nan")
    C2_trimmed_mean: float = float("nan")
    C3_trimmed_mean: float = float("nan")
    C4_trimmed_mean: float = float("nan")
    cap_sample_count: int = 0
    force_sample_count: int = 0

    def to_row(self) -> dict:
        return asdict(self)
