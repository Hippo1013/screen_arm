from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Save one live D435i RGB visualization frame with 3DDFA overlay.")
    parser.add_argument("--config", default="../config.yaml")
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--output", default="../outputs/live_visual_snapshot.jpg")
    parser.add_argument("--require-valid", action="store_true", help="Return non-zero if no valid face pose is found")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    test_dir = Path(__file__).resolve().parent
    module_dir = test_dir.parent
    sys.path.insert(0, str(module_dir))

    from camera_realsense_rgb import RealSenseRgbCamera
    from face_detector import create_face_detector
    from filters import PoseStabilizer
    from three_ddfa_pose import ThreeDDFAPoseEstimator
    from visualizer import PoseVisualizer

    config_path = Path(args.config)
    if not config_path.is_absolute():
        config_path = test_dir / config_path
    with config_path.resolve().open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    camera = RealSenseRgbCamera(config["camera"])
    detector = create_face_detector(config["detector"], module_dir)
    estimator = ThreeDDFAPoseEstimator(config["three_ddfa"], module_dir)
    stabilizer = PoseStabilizer(config["filter"])
    visualizer = PoseVisualizer(
        config["visualization"],
        arrow_length_m=float(config["three_ddfa"].get("normal_arrow_length_m", 0.12)),
    )

    output_path = Path(args.output)
    if not output_path.is_absolute():
        output_path = test_dir / output_path
    output_path = output_path.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    last_pose = None
    last_display = None
    camera.start()
    try:
        for _ in range(max(1, args.frames)):
            frame = camera.read()
            if frame is None:
                continue
            pose = estimator.estimate(frame.color_bgr, frame.intrinsics, detector, timestamp=time.time(), imu=frame.imu)
            pose = stabilizer.update(pose)
            last_pose = pose
            last_display = visualizer.draw(frame.color_bgr.copy(), pose, fps=0.0, intrinsics=frame.intrinsics)
            if pose.valid and pose.status != "lost_hold":
                break
    finally:
        camera.stop()

    if last_display is None or last_pose is None:
        print("No frame captured.")
        return 2

    cv2.imwrite(str(output_path), last_display)
    print(f"Saved live visualization: {output_path}")
    print(f"Latest pose: {last_pose.to_udp_dict()}")
    if args.require_valid and not last_pose.valid:
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
