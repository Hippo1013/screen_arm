from __future__ import annotations

from typing import Any

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:  # pragma: no cover - depends on local hardware environment
    rs = None


class DepthProjector:
    def __init__(self, config: dict[str, Any]) -> None:
        if rs is None:
            raise RuntimeError("缺少 pyrealsense2，无法执行深度反投影。")

        self.min_depth_m = float(config["min_depth_m"])
        self.max_depth_m = float(config["max_depth_m"])
        self.window_size = int(config["depth_window_size"])
        self.valid_ratio = float(config["depth_valid_ratio"])

    def deproject_pixel(
        self,
        u: float,
        v: float,
        image_width: int,
        image_height: int,
        depth_frame: Any,
        intrinsics: Any,
    ) -> tuple[np.ndarray, tuple[int, int], float] | None:
        x = int(round(u))
        y = int(round(v))
        if x < 0 or y < 0 or x >= image_width or y >= image_height:
            return None

        depth_m = self._median_depth(depth_frame, x, y, image_width, image_height)
        if depth_m is None:
            return None

        point = rs.rs2_deproject_pixel_to_point(intrinsics, [float(x), float(y)], float(depth_m))
        return np.asarray(point, dtype=np.float64), (x, y), float(depth_m)

    def _median_depth(
        self,
        depth_frame: Any,
        x: int,
        y: int,
        image_width: int,
        image_height: int,
    ) -> float | None:
        radius = max(0, self.window_size // 2)
        values: list[float] = []
        total = 0

        for yy in range(max(0, y - radius), min(image_height, y + radius + 1)):
            for xx in range(max(0, x - radius), min(image_width, x + radius + 1)):
                total += 1
                depth = float(depth_frame.get_distance(xx, yy))
                if self.min_depth_m <= depth <= self.max_depth_m:
                    values.append(depth)

        if total == 0 or len(values) / total < self.valid_ratio:
            return None
        return float(np.median(values))
