from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from pose_types import FacePose


class PoseVisualizer:
    def __init__(self, config: dict[str, Any], arrow_length_m: float) -> None:
        self.enabled = bool(config.get("enabled", True))
        self.window_name = str(config.get("window_name", "3DDFA_V2 Face Pose Debug"))
        self.window_position = _optional_int_pair(config.get("window_position"))
        self.window_size = _optional_int_pair(config.get("window_size"))
        self.window_initialized = False
        self.draw_bbox = bool(config.get("draw_bbox", True))
        self.draw_landmarks = bool(config.get("draw_landmarks", True))
        self.draw_axes = bool(config.get("draw_axes", True))
        self.draw_auxiliary_x_axis = bool(config.get("draw_auxiliary_x_axis", False))
        self.font_scale = float(config.get("font_scale", 0.55))
        self.line_thickness = int(config.get("line_thickness", 2))
        self.arrow_length_m = float(arrow_length_m)

    def draw(self, frame: np.ndarray, pose: FacePose, fps: float, intrinsics: Any) -> np.ndarray:
        if self.draw_bbox and pose.bbox is not None:
            x1, y1, x2, y2 = pose.bbox
            color = (0, 220, 0) if pose.valid else (0, 0, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, self.line_thickness, lineType=cv2.LINE_AA)

        if self.draw_landmarks and pose.landmarks_px:
            for point in pose.landmarks_px:
                cv2.circle(frame, point, 2, (0, 255, 255), -1, lineType=cv2.LINE_AA)

        if pose.center_px is not None:
            color = (0, 255, 0) if pose.valid else (0, 0, 255)
            cv2.circle(frame, pose.center_px, 5, color, -1, lineType=cv2.LINE_AA)

        if self.draw_axes and pose.valid and pose.center is not None and pose.normal is not None:
            self._draw_vector(frame, intrinsics, pose.center, pose.normal, self.arrow_length_m, (0, 0, 255))
            if self.draw_auxiliary_x_axis and pose.x_axis is not None:
                self._draw_vector(frame, intrinsics, pose.center, pose.x_axis, self.arrow_length_m * 0.7, (255, 0, 0))

        self._draw_text(frame, pose, fps)
        return frame

    def show(self, frame: np.ndarray) -> bool:
        if not self.window_initialized:
            flags = cv2.WINDOW_NORMAL if self.window_size is not None else cv2.WINDOW_AUTOSIZE
            cv2.namedWindow(self.window_name, flags)
            if self.window_size is not None:
                cv2.resizeWindow(self.window_name, self.window_size[0], self.window_size[1])
            if self.window_position is not None:
                cv2.moveWindow(self.window_name, self.window_position[0], self.window_position[1])
            self.window_initialized = True
        cv2.imshow(self.window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        return key == ord("q")

    def close(self) -> None:
        if self.window_initialized:
            cv2.destroyWindow(self.window_name)
            self.window_initialized = False

    def _draw_vector(
        self,
        frame: np.ndarray,
        intrinsics: Any,
        origin: np.ndarray,
        direction: np.ndarray,
        length_m: float,
        color: tuple[int, int, int],
    ) -> None:
        start = _project_point(origin, intrinsics)
        end = _project_point(origin + direction * length_m, intrinsics)
        if start is None or end is None:
            return
        cv2.arrowedLine(
            frame,
            start,
            end,
            color,
            self.line_thickness,
            line_type=cv2.LINE_AA,
            tipLength=0.25,
        )

    def _draw_text(self, frame: np.ndarray, pose: FacePose, fps: float) -> None:
        lines = [
            f"valid: {pose.valid}  status: {pose.status}",
            f"FPS: {fps:.1f}",
        ]
        if pose.angles_deg:
            lines.append(
                "yaw/pitch/roll: "
                f"{pose.angles_deg.get('yaw', 0.0):+.1f}, "
                f"{pose.angles_deg.get('pitch', 0.0):+.1f}, "
                f"{pose.angles_deg.get('roll', 0.0):+.1f} deg"
            )
        if pose.normal is not None:
            lines.append(f"face_dir/red normal: {pose.normal[0]:+.3f}, {pose.normal[1]:+.3f}, {pose.normal[2]:+.3f}")
        if pose.center is not None:
            lines.append(f"center proxy[m]: {pose.center[0]:+.3f}, {pose.center[1]:+.3f}, {pose.center[2]:+.3f}")
        if pose.imu:
            if "accel" in pose.imu:
                accel = pose.imu["accel"]
                lines.append(f"accel: {accel[0]:+.2f}, {accel[1]:+.2f}, {accel[2]:+.2f}")
            if "gyro" in pose.imu:
                gyro = pose.imu["gyro"]
                lines.append(f"gyro: {gyro[0]:+.3f}, {gyro[1]:+.3f}, {gyro[2]:+.3f}")

        x = 12
        y = 24
        for line in lines:
            cv2.putText(
                frame,
                line,
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                self.font_scale,
                (0, 0, 0),
                self.line_thickness + 2,
                lineType=cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                line,
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                self.font_scale,
                (255, 255, 255),
                self.line_thickness,
                lineType=cv2.LINE_AA,
            )
            y += 24


def _project_point(point: np.ndarray, intrinsics: Any) -> tuple[int, int] | None:
    if point[2] <= 1e-6:
        return None
    fx = float(getattr(intrinsics, "fx", 0.0) or 0.0)
    fy = float(getattr(intrinsics, "fy", 0.0) or 0.0)
    ppx = float(getattr(intrinsics, "ppx", 0.0) or 0.0)
    ppy = float(getattr(intrinsics, "ppy", 0.0) or 0.0)
    if fx <= 1e-6 or fy <= 1e-6:
        return None
    u = point[0] / point[2] * fx + ppx
    v = point[1] / point[2] * fy + ppy
    return int(round(float(u))), int(round(float(v)))


def _optional_int_pair(value: Any) -> tuple[int, int] | None:
    if value is None:
        return None
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        return None
    return int(value[0]), int(value[1])
