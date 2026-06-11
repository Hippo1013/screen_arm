from __future__ import annotations

import argparse
import sys
import time
from collections import Counter
from pathlib import Path

import yaml

MODULE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MODULE_DIR))

from camera_realsense import RealSenseCamera  # noqa: E402
from depth_projector import DepthProjector  # noqa: E402
from face_geometry import FaceGeometryEstimator  # noqa: E402
from face_landmarker import FaceLandmarkerDetector  # noqa: E402
from filters import PoseStabilizer  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="人脸建模完整链路无窗口冒烟测试")
    parser.add_argument("--config", default=str(MODULE_DIR / "config.yaml"), help="配置文件路径")
    parser.add_argument("--frames", type=int, default=90, help="处理帧数")
    return parser.parse_args()


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def main() -> int:
    args = parse_args()
    config = load_config(Path(args.config))

    camera = RealSenseCamera(config["camera"])
    valid_count = 0
    imu_count = 0
    statuses: Counter[str] = Counter()
    start = time.perf_counter()

    camera.start()
    projector = DepthProjector(config["geometry"])
    landmarker = FaceLandmarkerDetector(config["mediapipe"], MODULE_DIR)
    estimator = FaceGeometryEstimator(config["geometry"], projector)
    stabilizer = PoseStabilizer(config["filter"])

    try:
        for _ in range(args.frames):
            frame = camera.read()
            if frame is None:
                statuses["no_frame"] += 1
                continue

            result = landmarker.detect(frame.color_bgr, int(time.monotonic() * 1000))
            pose = estimator.estimate(
                result,
                frame.color_bgr.shape,
                frame.depth_frame,
                frame.intrinsics,
                time.time(),
            )
            pose = stabilizer.update(pose.copy_with(imu=frame.imu))
            statuses[pose.status] += 1
            valid_count += int(pose.valid)
            imu_count += int(bool(frame.imu))
    finally:
        camera.stop()
        landmarker.close()

    elapsed = max(1e-6, time.perf_counter() - start)
    print(f"frames: {args.frames}")
    print(f"avg fps: {args.frames / elapsed:.2f}")
    print(f"valid poses: {valid_count}")
    print(f"frames with imu: {imu_count}")
    print(f"statuses: {dict(statuses)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
