from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import yaml

MODULE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MODULE_DIR))

from camera_realsense import RealSenseCamera  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="D435i 彩色/深度/IMU 冒烟测试")
    parser.add_argument("--config", default=str(MODULE_DIR / "config.yaml"), help="配置文件路径")
    parser.add_argument("--frames", type=int, default=60, help="读取帧数")
    return parser.parse_args()


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def main() -> int:
    args = parse_args()
    config = load_config(Path(args.config))["camera"]
    camera = RealSenseCamera(config)

    print("启动 D435i...")
    camera.start()
    count = 0
    first_time = time.perf_counter()
    last_imu = {}
    try:
        while count < args.frames:
            frame = camera.read()
            if frame is None:
                continue
            count += 1
            if frame.imu:
                last_imu = frame.imu
            if count == 1:
                h, w = frame.color_bgr.shape[:2]
                print(f"color: {w}x{h}")
                print(f"depth intrinsics: {frame.intrinsics.width}x{frame.intrinsics.height}")
    finally:
        camera.stop()

    elapsed = max(1e-6, time.perf_counter() - first_time)
    print(f"frames: {count}")
    print(f"avg fps: {count / elapsed:.2f}")
    print(f"last imu: {last_imu if last_imu else 'no imu sample in captured frames'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
