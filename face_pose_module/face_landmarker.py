from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np


class FaceLandmarkerDetector:
    def __init__(self, config: dict[str, Any], module_dir: Path) -> None:
        cv2, mp = _load_dependencies()
        self.cv2 = cv2
        self.mp = mp
        self.last_timestamp_ms = -1

        model_path = Path(config["model_path"])
        if not model_path.is_absolute():
            model_path = module_dir / model_path
        if not model_path.exists():
            raise FileNotFoundError(
                f"找不到 MediaPipe 模型文件: {model_path}。请下载 face_landmarker.task 到 assets/ 目录。"
            )

        running_mode = str(config.get("running_mode", "video")).lower()
        if running_mode != "video":
            raise ValueError("当前实时循环使用 VIDEO 模式，请在 config.yaml 中保持 mediapipe.running_mode: video。")

        BaseOptions = mp.tasks.BaseOptions
        FaceLandmarker = mp.tasks.vision.FaceLandmarker
        FaceLandmarkerOptions = mp.tasks.vision.FaceLandmarkerOptions
        VisionRunningMode = mp.tasks.vision.RunningMode

        options = FaceLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=str(model_path)),
            running_mode=VisionRunningMode.VIDEO,
            num_faces=int(config.get("num_faces", 1)),
            min_face_detection_confidence=float(config.get("min_face_detection_confidence", 0.5)),
            min_face_presence_confidence=float(config.get("min_face_presence_confidence", 0.5)),
            min_tracking_confidence=float(config.get("min_tracking_confidence", 0.5)),
            output_face_blendshapes=bool(config.get("output_face_blendshapes", False)),
            output_facial_transformation_matrixes=bool(
                config.get("output_facial_transformation_matrixes", True)
            ),
        )
        self.landmarker = FaceLandmarker.create_from_options(options)

    def detect(self, color_bgr: np.ndarray, timestamp_ms: int) -> Any:
        color_rgb = self.cv2.cvtColor(color_bgr, self.cv2.COLOR_BGR2RGB)
        mp_image = self.mp.Image(
            image_format=self.mp.ImageFormat.SRGB,
            data=np.ascontiguousarray(color_rgb),
        )
        timestamp_ms = max(int(timestamp_ms), self.last_timestamp_ms + 1)
        self.last_timestamp_ms = timestamp_ms
        return self.landmarker.detect_for_video(mp_image, timestamp_ms)

    def close(self) -> None:
        self.landmarker.close()


def _load_dependencies() -> tuple[Any, Any]:
    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - depends on local environment
        raise RuntimeError("缺少 opencv-python，无法运行 Face Landmarker。请先安装 requirements.txt。") from exc

    try:
        import mediapipe as mp
    except ImportError as exc:  # pragma: no cover - depends on local environment
        raise RuntimeError("缺少 mediapipe，无法运行 Face Landmarker。请先安装 requirements.txt。") from exc

    return cv2, mp
