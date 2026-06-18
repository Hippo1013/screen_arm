from __future__ import annotations

from dataclasses import dataclass
from threading import Lock
from typing import Any

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:  # pragma: no cover - hardware/runtime dependency
    rs = None


@dataclass
class RealSenseRgbFrame:
    color_bgr: np.ndarray
    intrinsics: Any
    timestamp_ms: float
    imu: dict[str, tuple[float, float, float]]


class RealSenseRgbCamera:
    def __init__(self, config: dict[str, Any]) -> None:
        if rs is None:
            raise RuntimeError("pyrealsense2 is not installed; cannot start D435i RGB stream.")

        self.config_data = config
        self.pipeline = rs.pipeline()
        self.rs_config = rs.config()
        self.profile = None
        self.frame_queue = None
        self.imu_lock = Lock()
        self.latest_imu: dict[str, tuple[float, float, float]] = {}
        self.started = False

    def start(self) -> None:
        color_format = self._resolve_format(str(self.config_data.get("color_format", "bgr8")))
        self.rs_config.enable_stream(
            rs.stream.color,
            int(self.config_data["color_width"]),
            int(self.config_data["color_height"]),
            color_format,
            int(self.config_data["color_fps"]),
        )

        if self.config_data.get("enable_imu", True):
            self._enable_imu_streams()

        try:
            if self.config_data.get("enable_imu", True):
                self.frame_queue = rs.frame_queue(5, True)
                self.profile = self.pipeline.start(self.rs_config, self._frame_callback)
            else:
                self.profile = self.pipeline.start(self.rs_config)
        except RuntimeError as exc:
            raise RuntimeError(
                "Could not start RealSense color/IMU streams. Check D435i connection, USB bandwidth, "
                "and supported color resolution/fps."
            ) from exc

        self.started = True
        self._warmup()

    def read(self) -> RealSenseRgbFrame | None:
        frames = self._wait_for_video_frames(int(self.config_data.get("frame_timeout_ms", 5000)))
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None

        color_image = np.asanyarray(color_frame.get_data())
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        return RealSenseRgbFrame(
            color_bgr=color_image,
            intrinsics=intrinsics,
            timestamp_ms=float(frames.get_timestamp()),
            imu=self._latest_imu_snapshot(),
        )

    def stop(self) -> None:
        if self.started:
            self.pipeline.stop()
            self.started = False

    def _warmup(self) -> None:
        warmup_frames = int(self.config_data.get("warmup_frames", 0))
        timeout_ms = int(self.config_data.get("frame_timeout_ms", 5000))
        for _ in range(max(0, warmup_frames)):
            self._wait_for_video_frames(timeout_ms)

    def _wait_for_video_frames(self, timeout_ms: int) -> Any:
        if self.frame_queue is None:
            return self.pipeline.wait_for_frames(timeout_ms)

        frame = self.frame_queue.wait_for_frame(timeout_ms)
        if not frame or not frame.is_frameset():
            raise RuntimeError("RealSense did not return a color frameset.")
        return frame.as_frameset()

    def _frame_callback(self, frame: Any) -> None:
        try:
            if frame.is_motion_frame():
                self._store_motion_frame(frame.as_motion_frame())
                return
            if frame.is_frameset() and self.frame_queue is not None:
                self.frame_queue.enqueue(frame)
        except RuntimeError:
            return

    def _enable_imu_streams(self) -> None:
        accel_fps = int(self.config_data.get("imu_accel_fps", 63))
        gyro_fps = int(self.config_data.get("imu_gyro_fps", 200))
        try:
            self.rs_config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, accel_fps)
            self.rs_config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, gyro_fps)
        except TypeError:
            self.rs_config.enable_stream(rs.stream.accel)
            self.rs_config.enable_stream(rs.stream.gyro)

    def _store_motion_frame(self, motion_frame: Any) -> None:
        if not motion_frame:
            return

        data = motion_frame.get_motion_data()
        sample = (float(data.x), float(data.y), float(data.z))
        stream_type = motion_frame.profile.stream_type()
        with self.imu_lock:
            if stream_type == rs.stream.accel:
                self.latest_imu["accel"] = sample
            elif stream_type == rs.stream.gyro:
                self.latest_imu["gyro"] = sample

    def _latest_imu_snapshot(self) -> dict[str, tuple[float, float, float]]:
        with self.imu_lock:
            return dict(self.latest_imu)

    @staticmethod
    def _resolve_format(name: str) -> Any:
        format_map = {
            "bgr8": rs.format.bgr8,
            "rgb8": rs.format.rgb8,
        }
        try:
            return format_map[name.lower()]
        except KeyError as exc:
            raise ValueError(f"Unsupported RealSense color format: {name}") from exc
