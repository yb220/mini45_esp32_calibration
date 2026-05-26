from __future__ import annotations

from dataclasses import dataclass

from .models import ForceSample


FORCE_AXES = ("Fx", "Fy", "Fz")
FORCE_FIELDS = {"Fx": "fx", "Fy": "fy", "Fz": "fz"}
TORQUE_FIELDS = {"Fx": "mx", "Fy": "my", "Fz": "mz"}


@dataclass(frozen=True)
class AxisFrameMap:
    source_axis: str
    sign: int = 1


@dataclass(frozen=True)
class ForceFrameMapping:
    sensor_fx: AxisFrameMap
    sensor_fy: AxisFrameMap
    sensor_fz: AxisFrameMap

    @classmethod
    def identity(cls) -> "ForceFrameMapping":
        return cls(
            sensor_fx=AxisFrameMap("Fx", 1),
            sensor_fy=AxisFrameMap("Fy", 1),
            sensor_fz=AxisFrameMap("Fz", 1),
        )

    def validate(self) -> None:
        rows = (self.sensor_fx, self.sensor_fy, self.sensor_fz)
        sources = [row.source_axis for row in rows]
        invalid = [axis for axis in sources if axis not in FORCE_AXES]
        if invalid:
            raise ValueError(f"坐标映射包含无效 Mini45 轴：{', '.join(invalid)}")
        if len(set(sources)) != 3:
            raise ValueError("Mini45 力轴必须一一映射到传感器 Fx/Fy/Fz，不能重复选择")
        for row in rows:
            if row.sign not in (-1, 1):
                raise ValueError("坐标映射符号只能为 + 或 -")

    def as_row(self, timestamp: str, experiment_id: str) -> dict:
        self.validate()
        return {
            "timestamp": timestamp,
            "experiment_id": experiment_id,
            "sensor_Fx_from": self.sensor_fx.source_axis,
            "sensor_Fx_sign": self.sensor_fx.sign,
            "sensor_Fy_from": self.sensor_fy.source_axis,
            "sensor_Fy_sign": self.sensor_fy.sign,
            "sensor_Fz_from": self.sensor_fz.source_axis,
            "sensor_Fz_sign": self.sensor_fz.sign,
        }


def _mapped_value(sample: ForceSample, axis_map: AxisFrameMap, field_map: dict[str, str]) -> float:
    return float(axis_map.sign) * float(getattr(sample, field_map[axis_map.source_axis]))


def transform_force_sample(raw_sample: ForceSample, mapping: ForceFrameMapping) -> ForceSample:
    """将 Mini45 原始力/力矩转换到待标定传感器坐标系。"""
    mapping.validate()
    return ForceSample(
        timestamp=raw_sample.timestamp,
        monotonic_s=raw_sample.monotonic_s,
        fx=_mapped_value(raw_sample, mapping.sensor_fx, FORCE_FIELDS),
        fy=_mapped_value(raw_sample, mapping.sensor_fy, FORCE_FIELDS),
        fz=_mapped_value(raw_sample, mapping.sensor_fz, FORCE_FIELDS),
        mx=_mapped_value(raw_sample, mapping.sensor_fx, TORQUE_FIELDS),
        my=_mapped_value(raw_sample, mapping.sensor_fy, TORQUE_FIELDS),
        mz=_mapped_value(raw_sample, mapping.sensor_fz, TORQUE_FIELDS),
        sequence=raw_sample.sequence,
        status=raw_sample.status,
        source=raw_sample.source,
    )
