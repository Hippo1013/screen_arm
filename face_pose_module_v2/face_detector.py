from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Any

import cv2
import numpy as np


@dataclass(frozen=True)
class FaceBox:
    x1: int
    y1: int
    x2: int
    y2: int
    score: float
    source: str = "opencv"

    @property
    def area(self) -> int:
        return max(0, self.x2 - self.x1) * max(0, self.y2 - self.y1)

    def as_3ddfa_box(self) -> list[float]:
        return [float(self.x1), float(self.y1), float(self.x2), float(self.y2), float(self.score)]

    def as_tuple(self) -> tuple[int, int, int, int]:
        return self.x1, self.y1, self.x2, self.y2


class OpenCvHaarFaceDetector:
    def __init__(self, config: dict[str, Any]) -> None:
        self.scale_factor = float(config.get("scale_factor", 1.1))
        self.min_neighbors = int(config.get("min_neighbors", 5))
        self.min_size_ratio = float(config.get("min_size_ratio", 0.10))
        self.max_faces = int(config.get("max_faces", 1))
        self.nms_iou_threshold = float(config.get("nms_iou_threshold", 0.35))
        self.equalize_hist = bool(config.get("equalize_hist", True))

        frontal_path = self._cascade_path(config.get("frontal_cascade"), "haarcascade_frontalface_default.xml")
        profile_path = self._cascade_path(config.get("profile_cascade"), "haarcascade_profileface.xml")
        self.frontal = self._load_cascade(frontal_path)
        self.profile = self._load_cascade(profile_path)

    def detect(self, frame_bgr: np.ndarray) -> list[FaceBox]:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        if self.equalize_hist:
            gray = cv2.equalizeHist(gray)

        min_side = max(20, int(min(gray.shape[:2]) * self.min_size_ratio))
        min_size = (min_side, min_side)
        boxes: list[FaceBox] = []
        boxes.extend(self._detect_with_cascade(self.frontal, gray, min_size, "frontal", flipped=False))
        boxes.extend(self._detect_with_cascade(self.profile, gray, min_size, "profile_left", flipped=False))
        boxes.extend(self._detect_with_cascade(self.profile, cv2.flip(gray, 1), min_size, "profile_right", flipped=True))

        boxes = self._nms(boxes, self.nms_iou_threshold)
        boxes.sort(key=lambda box: (box.score, box.area), reverse=True)
        return boxes[: self.max_faces]

    def _detect_with_cascade(
        self,
        cascade: cv2.CascadeClassifier,
        gray: np.ndarray,
        min_size: tuple[int, int],
        source: str,
        flipped: bool,
    ) -> list[FaceBox]:
        detections = cascade.detectMultiScale(
            gray,
            scaleFactor=self.scale_factor,
            minNeighbors=self.min_neighbors,
            minSize=min_size,
            flags=cv2.CASCADE_SCALE_IMAGE,
        )
        width = gray.shape[1]
        boxes: list[FaceBox] = []
        for x, y, w, h in detections:
            if flipped:
                x1 = width - int(x + w)
                x2 = width - int(x)
            else:
                x1 = int(x)
                x2 = int(x + w)
            y1 = int(y)
            y2 = int(y + h)
            area_score = float(w * h)
            source_bonus = 1.0 if source == "frontal" else 0.92
            boxes.append(FaceBox(x1=x1, y1=y1, x2=x2, y2=y2, score=area_score * source_bonus, source=source))
        return boxes

    @staticmethod
    def _nms(boxes: list[FaceBox], threshold: float) -> list[FaceBox]:
        if not boxes:
            return []

        boxes_sorted = sorted(boxes, key=lambda box: box.score, reverse=True)
        kept: list[FaceBox] = []
        for candidate in boxes_sorted:
            if all(_iou(candidate, kept_box) <= threshold for kept_box in kept):
                kept.append(candidate)
        return kept

    @staticmethod
    def _cascade_path(value: Any, default_name: str) -> Path:
        if value:
            return Path(str(value))
        return Path(cv2.data.haarcascades) / default_name

    @staticmethod
    def _load_cascade(path: Path) -> cv2.CascadeClassifier:
        cascade = cv2.CascadeClassifier(str(path))
        if cascade.empty():
            raise FileNotFoundError(f"Could not load OpenCV cascade: {path}")
        return cascade


class FaceBoxesFaceDetector:
    def __init__(self, config: dict[str, Any], module_dir: Path) -> None:
        self.max_faces = int(config.get("max_faces", 1))
        self.min_confidence = float(config.get("min_confidence", 0.5))
        repo_path = Path(str(config.get("repo_path", "third_party/3DDFA_V2")))
        if not repo_path.is_absolute():
            repo_path = module_dir / repo_path
        self.repo_path = repo_path.resolve()
        if str(self.repo_path) not in sys.path:
            sys.path.insert(0, str(self.repo_path))

        from FaceBoxes import FaceBoxes

        self.face_boxes = FaceBoxes()

    def detect(self, frame_bgr: np.ndarray) -> list[FaceBox]:
        height, width = frame_bgr.shape[:2]
        detections = self.face_boxes(frame_bgr)
        boxes: list[FaceBox] = []
        for det in detections:
            if len(det) < 5:
                continue
            score = float(det[4])
            if score < self.min_confidence:
                continue
            x1 = int(round(float(det[0])))
            y1 = int(round(float(det[1])))
            x2 = int(round(float(det[2])))
            y2 = int(round(float(det[3])))
            x1 = max(0, min(width - 1, x1))
            x2 = max(0, min(width - 1, x2))
            y1 = max(0, min(height - 1, y1))
            y2 = max(0, min(height - 1, y2))
            if x2 <= x1 or y2 <= y1:
                continue
            boxes.append(FaceBox(x1=x1, y1=y1, x2=x2, y2=y2, score=score, source="faceboxes"))

        boxes.sort(key=lambda box: (box.score, box.area), reverse=True)
        return boxes[: self.max_faces]


def create_face_detector(config: dict[str, Any], module_dir: Path | None = None) -> OpenCvHaarFaceDetector | FaceBoxesFaceDetector:
    detector_type = str(config.get("type", "opencv_haar")).lower()
    if detector_type == "opencv_haar":
        return OpenCvHaarFaceDetector(config)
    if detector_type == "faceboxes":
        if module_dir is None:
            module_dir = Path(__file__).resolve().parent
        return FaceBoxesFaceDetector(config, module_dir)
    raise ValueError(f"Unsupported detector type: {detector_type}")


def _iou(a: FaceBox, b: FaceBox) -> float:
    x1 = max(a.x1, b.x1)
    y1 = max(a.y1, b.y1)
    x2 = min(a.x2, b.x2)
    y2 = min(a.y2, b.y2)
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    union = a.area + b.area - inter
    if union <= 0:
        return 0.0
    return float(inter / union)
