from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from face_detector import FaceBox
from pose_types import FacePose


class ThreeDDFAPoseEstimator:
    def __init__(self, config: dict[str, Any], module_dir: Path) -> None:
        self.config = config
        self.module_dir = module_dir
        self.repo_dir = self._resolve_path(config.get("repo_path", "third_party/3DDFA_V2"))
        if not self.repo_dir.exists():
            raise FileNotFoundError(
                f"3DDFA_V2 repo not found: {self.repo_dir}. Run scripts/bootstrap_3ddfa_v2.py first."
            )

        if str(self.repo_dir) not in sys.path:
            sys.path.insert(0, str(self.repo_dir))

        self.use_onnx = bool(config.get("use_onnx", False))
        cfg = self._load_3ddfa_config(config)
        cfg["gpu_mode"] = bool(config.get("gpu_mode", False))
        cfg["gpu_id"] = int(config.get("gpu_id", 0))

        if self.use_onnx:
            from TDDFA_ONNX import TDDFA_ONNX

            self.tddfa = TDDFA_ONNX(**cfg)
        else:
            from TDDFA import TDDFA

            self.tddfa = TDDFA(**cfg)

        from utils.pose import calc_pose

        self.calc_pose = calc_pose
        self.pre_vertices: np.ndarray | None = None
        self.frames_since_detection = 0
        self.detect_interval = int(config.get("detect_interval", 20))
        self.tracking_min_roi_area_px = float(config.get("tracking_min_roi_area_px", 2500.0))
        self.assumed_face_distance_m = float(config.get("assumed_face_distance_m", 0.65))
        self.normal_towards_camera = bool(config.get("normal_towards_camera", True))
        self.flip_output_x = bool(config.get("flip_output_x", True))
        self.flip_output_y = bool(config.get("flip_output_y", False))
        self.model_normal_axis = _normalize(np.asarray(config.get("model_normal_axis", [0.0, 0.0, -1.0]), dtype=np.float64))
        self.model_x_axis = _normalize(np.asarray(config.get("model_x_axis", [1.0, 0.0, 0.0]), dtype=np.float64))

    def estimate(
        self,
        frame_bgr: np.ndarray,
        intrinsics: Any,
        detector: Any,
        timestamp: float,
        imu: dict[str, tuple[float, float, float]] | None,
    ) -> FacePose:
        use_detection = self.pre_vertices is None or (
            self.detect_interval > 0 and self.frames_since_detection >= self.detect_interval
        )

        if use_detection:
            pose = self._estimate_from_detection(frame_bgr, intrinsics, detector, timestamp)
        else:
            pose = self._estimate_from_tracking(frame_bgr, intrinsics, detector, timestamp)

        return pose.copy_with(imu=imu)

    def reset_tracking(self) -> None:
        self.pre_vertices = None
        self.frames_since_detection = 0

    def _estimate_from_detection(
        self,
        frame_bgr: np.ndarray,
        intrinsics: Any,
        detector: Any,
        timestamp: float,
    ) -> FacePose:
        boxes = detector.detect(frame_bgr)
        if not boxes:
            self.reset_tracking()
            return FacePose(timestamp=timestamp, valid=False, status="no_face")

        try:
            return self._estimate_from_object(
                frame_bgr,
                intrinsics,
                boxes[0],
                crop_policy="box",
                timestamp=timestamp,
                status="valid_3ddfa_detected",
                detected_box=boxes[0],
            )
        except Exception as exc:
            self.reset_tracking()
            return FacePose(timestamp=timestamp, valid=False, status=f"3ddfa_detection_error:{type(exc).__name__}")

    def _estimate_from_tracking(
        self,
        frame_bgr: np.ndarray,
        intrinsics: Any,
        detector: Any,
        timestamp: float,
    ) -> FacePose:
        if self.pre_vertices is None:
            return self._estimate_from_detection(frame_bgr, intrinsics, detector, timestamp)

        try:
            pose = self._estimate_from_object(
                frame_bgr,
                intrinsics,
                self.pre_vertices,
                crop_policy="landmark",
                timestamp=timestamp,
                status="valid_3ddfa_tracking",
                detected_box=None,
            )
        except Exception:
            self.reset_tracking()
            return self._estimate_from_detection(frame_bgr, intrinsics, detector, timestamp)

        if pose.bbox is not None:
            x1, y1, x2, y2 = pose.bbox
            roi_area = max(0, x2 - x1) * max(0, y2 - y1)
            if roi_area < self.tracking_min_roi_area_px:
                self.reset_tracking()
                return self._estimate_from_detection(frame_bgr, intrinsics, detector, timestamp)

        self.frames_since_detection += 1
        return pose

    def _estimate_from_object(
        self,
        frame_bgr: np.ndarray,
        intrinsics: Any,
        obj: FaceBox | np.ndarray,
        crop_policy: str,
        timestamp: float,
        status: str,
        detected_box: FaceBox | None,
    ) -> FacePose:
        tddfa_obj = obj.as_3ddfa_box() if isinstance(obj, FaceBox) else obj
        param_lst, roi_box_lst = self.tddfa(frame_bgr, [tddfa_obj], crop_policy=crop_policy)
        ver = self.tddfa.recon_vers(param_lst, roi_box_lst, dense_flag=False)[0]
        param = param_lst[0]
        self.pre_vertices = ver
        if crop_policy == "box":
            self.frames_since_detection = 0

        p_matrix, pose_values = self.calc_pose(param)
        rotation = np.asarray(p_matrix[:, :3], dtype=np.float64)
        normal = self._normal_from_rotation(rotation)
        x_axis = self._x_axis_from_rotation(rotation, normal)
        center_px = _estimate_center_px(ver)
        center = _pixel_to_camera(center_px, self.assumed_face_distance_m, intrinsics)
        bbox = detected_box.as_tuple() if detected_box is not None else _bbox_from_roi(roi_box_lst[0])
        landmarks_px = _landmarks_to_pixels(ver)

        return FacePose(
            timestamp=timestamp,
            valid=True,
            center=center,
            normal=normal,
            x_axis=x_axis,
            center_px=center_px,
            bbox=bbox,
            landmarks_px=landmarks_px,
            angles_deg={
                "yaw": float(pose_values[0]),
                "pitch": float(pose_values[1]),
                "roll": float(pose_values[2]),
            },
            status=status,
        )

    def _normal_from_rotation(self, rotation: np.ndarray) -> np.ndarray:
        # Project convention: this vector is the red face axis and the face-facing direction.
        normal = _normalize(rotation @ self.model_normal_axis)
        if self.normal_towards_camera and normal[2] > 0:
            normal = -normal
        elif not self.normal_towards_camera and normal[2] < 0:
            normal = -normal
        normal = self._apply_output_axis_correction(normal)
        return normal

    def _x_axis_from_rotation(self, rotation: np.ndarray, normal: np.ndarray) -> np.ndarray:
        # Auxiliary horizontal face axis kept only for compatibility/debugging; it is not the gaze direction.
        x_axis = self._apply_output_axis_correction(rotation @ self.model_x_axis)
        x_axis = x_axis - normal * float(np.dot(x_axis, normal))
        x_axis = _normalize(x_axis)
        if x_axis[0] < 0:
            x_axis = -x_axis
        return x_axis

    def _apply_output_axis_correction(self, vector: np.ndarray) -> np.ndarray:
        corrected = np.asarray(vector, dtype=np.float64).copy()
        if self.flip_output_x:
            corrected[0] *= -1.0
        if self.flip_output_y:
            corrected[1] *= -1.0
        return _normalize(corrected)

    def _load_3ddfa_config(self, config: dict[str, Any]) -> dict[str, Any]:
        config_path = self._resolve_repo_path(config.get("config_path", "configs/mb1_120x120.yml"))
        with config_path.open("r", encoding="utf-8") as file:
            cfg = yaml.safe_load(file)

        for key in ("checkpoint_fp", "bfm_fp", "param_mean_std_fp", "onnx_fp"):
            if key in cfg and cfg[key]:
                cfg[key] = str(self._resolve_repo_path(cfg[key]))
        if "onnx_fp" in config and config["onnx_fp"]:
            cfg["onnx_fp"] = str(self._resolve_repo_path(config["onnx_fp"]))
        return cfg

    def _resolve_path(self, value: Any) -> Path:
        path = Path(str(value))
        if path.is_absolute():
            return path
        return (self.module_dir / path).resolve()

    def _resolve_repo_path(self, value: Any) -> Path:
        path = Path(str(value))
        if path.is_absolute():
            return path
        return (self.repo_dir / path).resolve()


def _normalize(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        raise ValueError("Cannot normalize near-zero vector.")
    return vector / norm


def _estimate_center_px(vertices: np.ndarray) -> tuple[int, int]:
    if vertices.shape[1] >= 48:
        left_eye = np.mean(vertices[:2, 36:42], axis=1)
        right_eye = np.mean(vertices[:2, 42:48], axis=1)
        center = (left_eye + right_eye) * 0.5
    else:
        center = np.mean(vertices[:2, :], axis=1)
    return int(round(float(center[0]))), int(round(float(center[1])))


def _pixel_to_camera(pixel: tuple[int, int], depth_m: float, intrinsics: Any) -> np.ndarray:
    fx = float(getattr(intrinsics, "fx", 0.0) or 0.0)
    fy = float(getattr(intrinsics, "fy", 0.0) or 0.0)
    ppx = float(getattr(intrinsics, "ppx", 0.0) or 0.0)
    ppy = float(getattr(intrinsics, "ppy", 0.0) or 0.0)
    if fx <= 1e-6 or fy <= 1e-6:
        return np.asarray([0.0, 0.0, depth_m], dtype=np.float64)
    x = (float(pixel[0]) - ppx) / fx * depth_m
    y = (float(pixel[1]) - ppy) / fy * depth_m
    return np.asarray([x, y, depth_m], dtype=np.float64)


def _bbox_from_roi(roi_box: Any) -> tuple[int, int, int, int]:
    sx, sy, ex, ey = [int(round(float(value))) for value in roi_box]
    return sx, sy, ex, ey


def _landmarks_to_pixels(vertices: np.ndarray) -> list[tuple[int, int]]:
    return [
        (int(round(float(x))), int(round(float(y))))
        for x, y in zip(vertices[0, :].tolist(), vertices[1, :].tolist())
    ]
