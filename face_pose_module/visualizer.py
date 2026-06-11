from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from face_geometry import FacePose


class PoseVisualizer:
    def __init__(self, config: dict[str, Any], normal_arrow_length_m: float) -> None:
        self.enabled = bool(config.get("enabled", True))
        self.window_name = str(config.get("window_name", "Face Pose Debug"))
        self.draw_all_selected_points = bool(config.get("draw_all_selected_points", True))
        self.draw_plane_hull = bool(config.get("draw_plane_hull", True))
        self.draw_axes = bool(config.get("draw_axes", True))
        self.font_scale = float(config.get("font_scale", 0.55))
        self.line_thickness = int(config.get("line_thickness", 2))
        self.normal_arrow_length_m = float(normal_arrow_length_m)

    def draw(self, frame: np.ndarray, pose: FacePose, fps: float, intrinsics: Any) -> np.ndarray:
        if self.draw_all_selected_points and pose.selected_points_px:
            for point in pose.selected_points_px:
                cv2.circle(frame, point, 2, (0, 255, 255), -1, lineType=cv2.LINE_AA)

        if self.draw_plane_hull and pose.plane_points_px and len(pose.plane_points_px) >= 3:
            hull = cv2.convexHull(np.asarray(pose.plane_points_px, dtype=np.int32))
            cv2.polylines(frame, [hull], True, (255, 180, 0), 1, lineType=cv2.LINE_AA)

        if pose.center_px is not None:
            color = (0, 255, 0) if pose.valid else (0, 0, 255)
            cv2.circle(frame, pose.center_px, 6, color, -1, lineType=cv2.LINE_AA)

        if self.draw_axes and pose.valid and pose.center is not None and pose.normal is not None:
            self._draw_vector(
                frame,
                intrinsics,
                pose.center,
                pose.normal,
                self.normal_arrow_length_m,
                (0, 0, 255),
            )
            if pose.x_axis is not None:
                self._draw_vector(
                    frame,
                    intrinsics,
                    pose.center,
                    pose.x_axis,
                    self.normal_arrow_length_m * 0.7,
                    (255, 0, 0),
                )

        self._draw_text(frame, pose, fps)
        return frame

    def show(self, frame: np.ndarray) -> bool:
        cv2.imshow(self.window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        return key == ord("q")

    def close(self) -> None:
        cv2.destroyWindow(self.window_name)

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
        if pose.center is not None:
            lines.append(f"center[m]: {pose.center[0]:+.3f}, {pose.center[1]:+.3f}, {pose.center[2]:+.3f}")
        if pose.normal is not None:
            lines.append(f"normal: {pose.normal[0]:+.3f}, {pose.normal[1]:+.3f}, {pose.normal[2]:+.3f}")
        if pose.x_axis is not None:
            lines.append(f"x_axis: {pose.x_axis[0]:+.3f}, {pose.x_axis[1]:+.3f}, {pose.x_axis[2]:+.3f}")
        if pose.imu:
            if "accel" in pose.imu:
                accel = pose.imu["accel"]
                lines.append(f"accel: {accel[0]:+.2f}, {accel[1]:+.2f}, {accel[2]:+.2f}")
            if "gyro" in pose.imu:
                gyro = pose.imu["gyro"]
                lines.append(f"gyro: {gyro[0]:+.3f}, {gyro[1]:+.3f}, {gyro[2]:+.3f}")
        if pose.rmse_m is not None:
            lines.append(f"plane rmse[m]: {pose.rmse_m:.4f}")

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
    u = point[0] / point[2] * intrinsics.fx + intrinsics.ppx
    v = point[1] / point[2] * intrinsics.fy + intrinsics.ppy
    return int(round(float(u))), int(round(float(v)))
