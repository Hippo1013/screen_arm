from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="D435i RGB + IMU smoke test for face_pose_module_v2.")
    parser.add_argument("--config", default="../config.yaml")
    parser.add_argument("--frames", type=int, default=60)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    test_dir = Path(__file__).resolve().parent
    module_dir = test_dir.parent
    sys.path.insert(0, str(module_dir))

    from camera_realsense_rgb import RealSenseRgbCamera

    config_path = Path(args.config)
    if not config_path.is_absolute():
        config_path = test_dir / config_path
    with config_path.resolve().open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    camera = RealSenseRgbCamera(config["camera"])
    camera.start()
    count = 0
    start = time.perf_counter()
    last_imu = {}
    try:
        while count < args.frames:
            frame = camera.read()
            if frame is None:
                continue
            count += 1
            last_imu = frame.imu
            if count == 1:
                intr = frame.intrinsics
                print(
                    "Color intrinsics: "
                    f"{intr.width}x{intr.height}, fx={intr.fx:.2f}, fy={intr.fy:.2f}, "
                    f"ppx={intr.ppx:.2f}, ppy={intr.ppy:.2f}"
                )
    finally:
        camera.stop()

    elapsed = max(1e-6, time.perf_counter() - start)
    print(f"Read {count} RGB frames in {elapsed:.2f}s ({count / elapsed:.1f} FPS).")
    print(f"Latest IMU: {last_imu}")
    return 0 if count > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
