from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import yaml


@dataclass
class SimpleIntrinsics:
    fx: float
    fy: float
    ppx: float
    ppy: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run 3DDFA_V2 on an offline image and save visualization.")
    parser.add_argument("--config", default="../config.yaml")
    parser.add_argument("--image", default="../third_party/3DDFA_V2/examples/inputs/emma.jpg")
    parser.add_argument("--output", default="../outputs/offline_image_pose.jpg")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    test_dir = Path(__file__).resolve().parent
    module_dir = test_dir.parent
    sys.path.insert(0, str(module_dir))

    from face_detector import create_face_detector
    from filters import PoseStabilizer
    from three_ddfa_pose import ThreeDDFAPoseEstimator
    from visualizer import PoseVisualizer

    config_path = Path(args.config)
    if not config_path.is_absolute():
        config_path = test_dir / config_path
    with config_path.resolve().open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    image_path = Path(args.image)
    if not image_path.is_absolute():
        image_path = test_dir / image_path
    frame = cv2.imread(str(image_path.resolve()))
    if frame is None:
        print(f"Could not read image: {image_path}")
        return 2

    height, width = frame.shape[:2]
    intrinsics = SimpleIntrinsics(
        fx=float(max(width, height)),
        fy=float(max(width, height)),
        ppx=float(width) * 0.5,
        ppy=float(height) * 0.5,
    )

    detector = create_face_detector(config["detector"], module_dir)
    estimator = ThreeDDFAPoseEstimator(config["three_ddfa"], module_dir)
    stabilizer = PoseStabilizer(config["filter"])
    visualizer = PoseVisualizer(
        config["visualization"],
        arrow_length_m=float(config["three_ddfa"].get("normal_arrow_length_m", 0.12)),
    )

    pose = estimator.estimate(frame, intrinsics, detector, timestamp=time.time(), imu={})
    pose = stabilizer.update(pose)
    if not pose.valid:
        print(f"Offline pose invalid: {pose.status}")
        return 3

    display = visualizer.draw(frame.copy(), pose, fps=0.0, intrinsics=intrinsics)
    output_path = Path(args.output)
    if not output_path.is_absolute():
        output_path = test_dir / output_path
    output_path.resolve().parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path.resolve()), display)
    print(f"Offline pose valid: {pose.to_udp_dict()}")
    print(f"Saved visualization: {output_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
