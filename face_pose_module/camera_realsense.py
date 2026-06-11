from __future__ import annotations

from dataclasses import dataclass
from threading import Lock
from typing import Any

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:  # pragma: no cover - depends on local hardware environment
    rs = None


@dataclass
class RealSenseFrame:
    color_bgr: np.ndarray
    depth_frame: Any
    intrinsics: Any
    timestamp_ms: float
    imu: dict[str, tuple[float, float, float]]


class RealSenseCamera:
    def __init__(self, config: dict[str, Any]) -> None:
        if rs is None:
            raise RuntimeError(
                "缺少 pyrealsense2，无法启动 D435i。请先安装 requirements.txt，并确认 RealSense SDK 环境可用。"
            )

        self.config_data = config
        self.pipeline = rs.pipeline()
        self.rs_config = rs.config()
        self.profile = None
        self.align = None
        self.frame_queue = None
        self.imu_lock = Lock()
        self.latest_imu: dict[str, tuple[float, float, float]] = {}

    def start(self) -> None:
        depth_format = self._resolve_format(self.config_data["depth_format"])
        color_format = self._resolve_format(self.config_data["color_format"])

        self.rs_config.enable_stream(
            rs.stream.depth,
            int(self.config_data["depth_width"]),
            int(self.config_data["depth_height"]),
            depth_format,
            int(self.config_data["depth_fps"]),
        )
        self.rs_config.enable_stream(
            rs.stream.color,
            int(self.config_data["color_width"]),
            int(self.config_data["color_height"]),
            color_format,
            int(self.config_data["color_fps"]),
        )

        if self.config_data.get("enable_infrared", False):
            self.rs_config.enable_stream(
                rs.stream.infrared,
                int(self.config_data["depth_width"]),
                int(self.config_data["depth_height"]),
                rs.format.y8,
                int(self.config_data["depth_fps"]),
            )

        if self.config_data.get("enable_imu", False):
            self._enable_imu_streams()

        try:
            if self.config_data.get("enable_imu", False):
                self.frame_queue = rs.frame_queue(5, True)
                self.profile = self.pipeline.start(self.rs_config, self._frame_callback)
            else:
                self.profile = self.pipeline.start(self.rs_config)
        except RuntimeError as exc:
            raise RuntimeError(
                "无法启动 RealSense 相机。请检查 D435i 是否连接、USB 端口是否为高速端口、分辨率和帧率是否被设备支持。"
            ) from exc

        if self.config_data.get("align_depth_to_color", True):
            self.align = rs.align(rs.stream.color)

        self._warmup()

    def read(self) -> RealSenseFrame | None:
        timeout_ms = int(self.config_data.get("frame_timeout_ms", 5000))
        frames = self._wait_for_video_frames(timeout_ms)
        imu = self._latest_imu_snapshot()
        if self.align is not None:
            frames = self.align.process(frames)

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return None

        color_image = np.asanyarray(color_frame.get_data())
        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        return RealSenseFrame(
            color_bgr=color_image,
            depth_frame=depth_frame,
            intrinsics=intrinsics,
            timestamp_ms=float(frames.get_timestamp()),
            imu=imu,
        )

    def stop(self) -> None:
        self.pipeline.stop()

    def _warmup(self) -> None:
        warmup_frames = int(self.config_data.get("warmup_frames", 0))
        for _ in range(max(0, warmup_frames)):
            self._wait_for_video_frames(int(self.config_data.get("frame_timeout_ms", 5000)))

    def _wait_for_video_frames(self, timeout_ms: int) -> Any:
        if self.frame_queue is None:
            return self.pipeline.wait_for_frames(timeout_ms)

        frame = self.frame_queue.wait_for_frame(timeout_ms)
        if not frame or not frame.is_frameset():
            raise RuntimeError("RealSense 未返回 color/depth 视频帧集。")
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
            # 兼容旧版 pyrealsense2 的重载形式。
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
            "z16": rs.format.z16,
            "bgr8": rs.format.bgr8,
            "rgb8": rs.format.rgb8,
            "y8": rs.format.y8,
        }
        try:
            return format_map[name.lower()]
        except KeyError as exc:
            raise ValueError(f"不支持的 RealSense stream format: {name}") from exc
