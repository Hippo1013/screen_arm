from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

MODULE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MODULE_DIR))

from face_landmarker import FaceLandmarkerDetector  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MediaPipe Face Landmarker 模型加载测试")
    parser.add_argument("--config", default=str(MODULE_DIR / "config.yaml"), help="配置文件路径")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    with Path(args.config).open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)["mediapipe"]

    detector = FaceLandmarkerDetector(config, MODULE_DIR)
    try:
        blank = np.zeros((480, 640, 3), dtype=np.uint8)
        result = detector.detect(blank, timestamp_ms=1)
        count = len(result.face_landmarks) if result.face_landmarks else 0
        print(f"model loaded, detected faces on blank image: {count}")
    finally:
        detector.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
