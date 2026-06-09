from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class AcquisitionProfile:
    name: str
    cnt: int
    cavg: int
    nominal_hz: float


STATIC_PRECISION = AcquisitionProfile("STATIC_PRECISION", 255, 32, 2.262325)
TRAINING_BALANCED = AcquisitionProfile("TRAINING_BALANCED", 191, 8, 11.363636)
TRAINING_FAST = AcquisitionProfile("TRAINING_FAST", 255, 1, 50.0)

ACQUISITION_PROFILES = {
    profile.name: profile
    for profile in (STATIC_PRECISION, TRAINING_BALANCED, TRAINING_FAST)
}


def get_acquisition_profile(name: str) -> AcquisitionProfile:
    normalized = str(name).strip().upper()
    if normalized not in ACQUISITION_PROFILES:
        raise ValueError(f"未知 MC1081 采集配置：{name}")
    return ACQUISITION_PROFILES[normalized]
