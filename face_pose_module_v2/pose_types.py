from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any

import numpy as np


@dataclass
class FacePose:
    timestamp: float
    valid: bool
    center: np.ndarray | None = None
    normal: np.ndarray | None = None
    x_axis: np.ndarray | None = None
    imu: dict[str, tuple[float, float, float]] | None = None
    center_px: tuple[int, int] | None = None
    bbox: tuple[int, int, int, int] | None = None
    landmarks_px: list[tuple[int, int]] | None = None
    angles_deg: dict[str, float] | None = None
    status: str = "invalid"

    def copy_with(self, **changes: Any) -> "FacePose":
        return replace(self, **changes)

    def to_udp_dict(self) -> dict[str, Any]:
        payload = {
            "t": float(self.timestamp),
            "valid": bool(self.valid),
            "center": _array_to_list(self.center),
            "normal": _array_to_list(self.normal),
            "x_axis": _array_to_list(self.x_axis),
            "imu": _imu_to_dict(self.imu),
        }
        if self.angles_deg is not None:
            payload["angles_deg"] = {key: float(value) for key, value in self.angles_deg.items()}
        if self.status:
            payload["status"] = self.status
        return payload


def _array_to_list(value: np.ndarray | None) -> list[float] | None:
    if value is None:
        return None
    return [float(x) for x in np.asarray(value, dtype=np.float64).reshape(-1).tolist()]


def _imu_to_dict(value: dict[str, tuple[float, float, float]] | None) -> dict[str, list[float]] | None:
    if not value:
        return None
    return {key: [float(x) for x in sample] for key, sample in value.items()}
