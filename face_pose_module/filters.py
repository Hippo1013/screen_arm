from __future__ import annotations

from collections import deque
from typing import Any

import numpy as np

from face_geometry import FacePose


class PoseStabilizer:
    def __init__(self, config: dict[str, Any]) -> None:
        self.enabled = bool(config.get("enabled", True))
        self.ema_alpha_center = float(config.get("ema_alpha_center", 0.35))
        self.ema_alpha_axis = float(config.get("ema_alpha_axis", 0.35))
        self.window = max(1, int(config.get("moving_average_window", 1)))
        self.center_deadband_m = float(config.get("center_deadband_m", 0.003))
        self.normal_deadband_deg = float(config.get("normal_deadband_deg", 1.0))
        self.x_axis_deadband_deg = float(config.get("x_axis_deadband_deg", 1.0))
        self.hold_seconds = float(config.get("hold_last_valid_seconds", 0.25))
        self.hold_marks_valid = bool(config.get("hold_marks_valid", True))

        self.last_output: FacePose | None = None
        self.center_history: deque[np.ndarray] = deque(maxlen=self.window)
        self.normal_history: deque[np.ndarray] = deque(maxlen=self.window)
        self.x_axis_history: deque[np.ndarray] = deque(maxlen=self.window)

    def update(self, pose: FacePose) -> FacePose:
        if not self.enabled:
            return pose

        if not pose.valid or pose.center is None or pose.normal is None or pose.x_axis is None:
            return self._hold_or_invalid(pose)

        center_input, normal_input, x_axis_input, held_parts = self._apply_component_deadbands(pose)

        center = self._ema_vector(center_input, self.last_output.center if self.last_output else None, self.ema_alpha_center)
        normal = self._ema_unit(normal_input, self.last_output.normal if self.last_output else None, self.ema_alpha_axis)
        x_axis = self._ema_unit(x_axis_input, self.last_output.x_axis if self.last_output else None, self.ema_alpha_axis)

        self.center_history.append(center)
        self.normal_history.append(normal)
        self.x_axis_history.append(x_axis)

        center = np.mean(np.asarray(self.center_history), axis=0)
        normal = _normalize(np.mean(np.asarray(self.normal_history), axis=0))
        x_axis = _normalize(np.mean(np.asarray(self.x_axis_history), axis=0))

        status = "valid_filtered"
        if held_parts:
            status = "filtered_deadband_" + "_".join(held_parts)
        output = pose.copy_with(center=center, normal=normal, x_axis=x_axis, status=status)
        self.last_output = output
        return output

    def _hold_or_invalid(self, pose: FacePose) -> FacePose:
        if self.last_output is None:
            return pose

        age = pose.timestamp - self.last_output.timestamp
        if 0.0 <= age <= self.hold_seconds:
            return self.last_output.copy_with(
                timestamp=pose.timestamp,
                valid=self.hold_marks_valid,
                imu=pose.imu or self.last_output.imu,
                selected_points_px=pose.selected_points_px,
                plane_points_px=pose.plane_points_px,
                status="lost_hold",
            )
        return pose

    def _apply_component_deadbands(self, pose: FacePose) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[str]]:
        center = pose.center
        normal = pose.normal
        x_axis = pose.x_axis
        held_parts: list[str] = []

        if self.last_output is None:
            return center, normal, x_axis, held_parts
        if self.last_output.center is None or self.last_output.normal is None or self.last_output.x_axis is None:
            return center, normal, x_axis, held_parts

        normal = _align_to_reference(normal, self.last_output.normal)
        x_axis = _align_to_reference(x_axis, self.last_output.x_axis)

        center_delta = float(np.linalg.norm(center - self.last_output.center))
        if center_delta < self.center_deadband_m:
            center = self.last_output.center
            held_parts.append("center")

        normal_delta = _angle_deg(normal, self.last_output.normal)
        if normal_delta < self.normal_deadband_deg:
            normal = self.last_output.normal
            held_parts.append("normal")

        x_delta = _angle_deg(x_axis, self.last_output.x_axis)
        if x_delta < self.x_axis_deadband_deg:
            x_axis = self.last_output.x_axis
            held_parts.append("x_axis")

        return center, normal, x_axis, held_parts

    @staticmethod
    def _ema_vector(current: np.ndarray, previous: np.ndarray | None, alpha: float) -> np.ndarray:
        if previous is None:
            return current
        return alpha * current + (1.0 - alpha) * previous

    @staticmethod
    def _ema_unit(current: np.ndarray, previous: np.ndarray | None, alpha: float) -> np.ndarray:
        if previous is not None and float(np.dot(current, previous)) < 0:
            current = -current
        value = PoseStabilizer._ema_vector(current, previous, alpha)
        return _normalize(value)


def _normalize(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        return vector
    return vector / norm


def _align_to_reference(current: np.ndarray, reference: np.ndarray) -> np.ndarray:
    if float(np.dot(current, reference)) < 0:
        return -current
    return current


def _angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
    return float(np.degrees(np.arccos(dot)))
