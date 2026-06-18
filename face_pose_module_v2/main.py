from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any

import yaml

from camera_realsense_rgb import RealSenseRgbCamera
from face_detector import create_face_detector
from filters import PoseStabilizer
from pose_file_writer import JsonPoseFileWriter
from three_ddfa_pose import ThreeDDFAPoseEstimator
from udp_sender import UdpPoseSender
from visualizer import PoseVisualizer


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="D435i RGB + 3DDFA_V2 face pose module")
    parser.add_argument("--config", default="config.yaml", help="Config file path")
    parser.add_argument("--no-udp", action="store_true", help="Disable UDP output")
    parser.add_argument("--no-window", action="store_true", help="Disable OpenCV visualization window")
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after N frames; 0 means run continuously")
    parser.add_argument("--pose-file", default="", help="Write latest pose JSON to this path")
    parser.add_argument("--window-x", type=int, default=None, help="OpenCV window X position")
    parser.add_argument("--window-y", type=int, default=None, help="OpenCV window Y position")
    parser.add_argument("--window-width", type=int, default=None, help="OpenCV window width")
    parser.add_argument("--window-height", type=int, default=None, help="OpenCV window height")
    parser.add_argument("--log-file", default="", help="Write Python stdout/stderr to this file")
    return parser.parse_args()


def load_config(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")
    with path.open("r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def apply_visualization_overrides(config: dict[str, Any], args: argparse.Namespace) -> None:
    visualization = config.setdefault("visualization", {})
    if args.window_x is not None and args.window_y is not None:
        visualization["window_position"] = [int(args.window_x), int(args.window_y)]
    if args.window_width is not None and args.window_height is not None:
        visualization["window_size"] = [int(args.window_width), int(args.window_height)]


def setup_log_file(path: str) -> None:
    if not path:
        return
    log_path = Path(path)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = log_path.open("a", encoding="utf-8", buffering=1)
    sys.stdout = log_file
    sys.stderr = log_file
    print(f"\n--- face_pose_module_v2 started at {time.strftime('%Y-%m-%d %H:%M:%S')} ---", flush=True)


def main() -> int:
    args = parse_args()
    setup_log_file(args.log_file)
    module_dir = Path(__file__).resolve().parent
    config_path = Path(args.config)
    if not config_path.is_absolute():
        config_path = module_dir / config_path

    camera = None
    udp_sender = None
    pose_file_writer = None
    visualizer = None

    try:
        config = load_config(config_path)
        apply_visualization_overrides(config, args)
        camera = RealSenseRgbCamera(config["camera"])
        detector = create_face_detector(config["detector"], module_dir)
        estimator = ThreeDDFAPoseEstimator(config["three_ddfa"], module_dir)
        stabilizer = PoseStabilizer(config["filter"])
        visualizer = PoseVisualizer(
            config["visualization"],
            arrow_length_m=float(config["three_ddfa"].get("normal_arrow_length_m", 0.12)),
        )
        if args.pose_file:
            pose_file_writer = JsonPoseFileWriter(args.pose_file)
        if bool(config["udp"].get("enabled", True)) and not args.no_udp:
            udp_sender = UdpPoseSender(config["udp"])
    except Exception as exc:
        print(f"[init error] {exc}", file=sys.stderr)
        return 2

    print("Starting RealSense RGB + IMU streams...")
    try:
        camera.start()
    except Exception as exc:
        print(f"[camera error] {exc}", file=sys.stderr)
        return 3

    if args.no_window and visualizer is not None:
        visualizer.enabled = False

    print("Running 3DDFA_V2 face pose estimation. Press q in the OpenCV window to quit.")
    last_time = time.perf_counter()
    fps = 0.0
    processed_frames = 0

    try:
        while True:
            frame = camera.read()
            if frame is None:
                continue

            now = time.perf_counter()
            dt = max(1e-6, now - last_time)
            last_time = now
            fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps > 0 else 1.0 / dt

            pose = estimator.estimate(
                frame.color_bgr,
                frame.intrinsics,
                detector,
                timestamp=time.time(),
                imu=frame.imu,
            )
            pose = stabilizer.update(pose)
            processed_frames += 1

            if pose_file_writer is not None:
                pose_file_writer.write(pose)

            if udp_sender is not None:
                udp_sender.send(pose)

            if visualizer is not None and visualizer.enabled:
                display = visualizer.draw(frame.color_bgr.copy(), pose, fps, frame.intrinsics)
                if visualizer.show(display):
                    break

            if args.max_frames > 0 and processed_frames >= args.max_frames:
                break
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[runtime error] {exc}", file=sys.stderr)
        return 4
    finally:
        camera.stop()
        if udp_sender is not None:
            udp_sender.close()
        if visualizer is not None and visualizer.enabled:
            visualizer.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
