from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any

import yaml

from camera_realsense import RealSenseCamera
from depth_projector import DepthProjector
from face_geometry import FaceGeometryEstimator
from face_landmarker import FaceLandmarkerDetector
from filters import PoseStabilizer
from udp_sender import UdpPoseSender
from visualizer import PoseVisualizer


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="D435i + MediaPipe 人脸位姿建模")
    parser.add_argument("--config", default="config.yaml", help="配置文件路径")
    parser.add_argument("--no-udp", action="store_true", help="关闭 UDP 输出")
    parser.add_argument("--no-window", action="store_true", help="关闭 OpenCV 可视化窗口")
    parser.add_argument("--max-frames", type=int, default=0, help="处理指定帧数后自动退出，0 表示持续运行")
    return parser.parse_args()


def load_config(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"找不到配置文件: {path}")
    with path.open("r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def main() -> int:
    args = parse_args()
    module_dir = Path(__file__).resolve().parent
    config_path = Path(args.config)
    if not config_path.is_absolute():
        config_path = module_dir / config_path

    camera = None
    landmarker = None
    udp_sender = None
    visualizer = None
    try:
        config = load_config(config_path)
        camera = RealSenseCamera(config["camera"])
    except Exception as exc:
        print(f"[init error] {exc}", file=sys.stderr)
        return 2

    print("启动 RealSense 相机...")
    try:
        camera.start()
    except Exception as exc:
        print(f"[camera error] {exc}", file=sys.stderr)
        return 3

    try:
        projector = DepthProjector(config["geometry"])
        landmarker = FaceLandmarkerDetector(config["mediapipe"], module_dir)
        estimator = FaceGeometryEstimator(config["geometry"], projector)
        stabilizer = PoseStabilizer(config["filter"])
        visualizer = PoseVisualizer(
            config["visualization"],
            normal_arrow_length_m=float(config["geometry"]["normal_arrow_length_m"]),
        )
        if bool(config["udp"].get("enabled", True)) and not args.no_udp:
            udp_sender = UdpPoseSender(config["udp"])
    except Exception as exc:
        print(f"[init error] {exc}", file=sys.stderr)
        camera.stop()
        return 2

    if args.no_window:
        visualizer.enabled = False

    print("开始人脸位姿建模。按 q 退出窗口。")
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

            timestamp_s = time.time()
            timestamp_ms = int(time.monotonic() * 1000)
            result = landmarker.detect(frame.color_bgr, timestamp_ms)
            pose = estimator.estimate(
                result,
                frame.color_bgr.shape,
                frame.depth_frame,
                frame.intrinsics,
                timestamp_s,
            )
            pose = pose.copy_with(imu=frame.imu)
            pose = stabilizer.update(pose)
            processed_frames += 1

            if udp_sender is not None:
                udp_sender.send(pose)

            if visualizer.enabled:
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
        if landmarker is not None:
            landmarker.close()
        if udp_sender is not None:
            udp_sender.close()
        if visualizer is not None and visualizer.enabled:
            visualizer.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
