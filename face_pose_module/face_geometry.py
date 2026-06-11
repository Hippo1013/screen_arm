from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from depth_projector import DepthProjector


@dataclass
class FacePose:
    timestamp: float
    valid: bool
    center: np.ndarray | None = None
    normal: np.ndarray | None = None
    x_axis: np.ndarray | None = None
    imu: dict[str, tuple[float, float, float]] | None = None
    center_px: tuple[int, int] | None = None
    selected_points_px: list[tuple[int, int]] | None = None
    plane_points_px: list[tuple[int, int]] | None = None
    plane_points_3d: np.ndarray | None = None
    rmse_m: float | None = None
    status: str = "invalid"

    def copy_with(self, **changes: Any) -> "FacePose":
        return replace(self, **changes)

    def to_udp_dict(self) -> dict[str, Any]:
        return {
            "t": float(self.timestamp),
            "valid": bool(self.valid),
            "center": _array_to_list(self.center),
            "normal": _array_to_list(self.normal),
            "x_axis": _array_to_list(self.x_axis),
            "imu": _imu_to_dict(self.imu),
        }


class FaceGeometryEstimator:
    def __init__(self, config: dict[str, Any], projector: DepthProjector) -> None:
        self.projector = projector
        self.left_eye_indices = [int(i) for i in config["left_eye_indices"]]
        self.right_eye_indices = [int(i) for i in config["right_eye_indices"]]
        self.plane_indices = [int(i) for i in config["plane_indices"]]
        self.min_valid_plane_points = int(config["min_valid_plane_points"])
        self.min_valid_eye_points = int(config["min_valid_eye_points"])
        self.outlier_sigma = float(config["outlier_sigma"])
        self.use_ransac = bool(config.get("use_ransac", True))
        self.ransac_iterations = int(config.get("ransac_iterations", 36))
        self.ransac_threshold_m = float(config.get("ransac_threshold_m", 0.018))
        self.ransac_min_inliers = int(config.get("ransac_min_inliers", 12))
        self.max_plane_rmse_m = float(config["max_plane_rmse_m"])
        self.normal_towards_camera = bool(config.get("normal_towards_camera", True))

    def estimate(
        self,
        landmarker_result: Any,
        image_shape: tuple[int, ...],
        depth_frame: Any,
        intrinsics: Any,
        timestamp: float,
    ) -> FacePose:
        if not getattr(landmarker_result, "face_landmarks", None):
            return FacePose(timestamp=timestamp, valid=False, status="no_face")

        landmarks = landmarker_result.face_landmarks[0]
        image_height, image_width = image_shape[:2]
        needed_indices = sorted(set(self.left_eye_indices + self.right_eye_indices + self.plane_indices))
        point_map = self._deproject_indices(
            landmarks,
            needed_indices,
            image_width,
            image_height,
            depth_frame,
            intrinsics,
        )

        left_eye = self._mean_point(point_map, self.left_eye_indices)
        right_eye = self._mean_point(point_map, self.right_eye_indices)
        if left_eye is None or right_eye is None:
            return FacePose(
                timestamp=timestamp,
                valid=False,
                selected_points_px=[p[1] for p in point_map.values()],
                status="eye_depth_invalid",
            )

        center = (left_eye[0] + right_eye[0]) * 0.5
        center_px = self._center_pixel(left_eye[1], right_eye[1])
        x_axis = self._estimate_x_axis(left_eye, right_eye)
        if x_axis is None:
            return FacePose(timestamp=timestamp, valid=False, status="x_axis_invalid")

        plane_points = [point_map[i][0] for i in self.plane_indices if i in point_map]
        plane_pixels = [point_map[i][1] for i in self.plane_indices if i in point_map]
        if len(plane_points) < self.min_valid_plane_points:
            return FacePose(
                timestamp=timestamp,
                valid=False,
                center=center,
                center_px=center_px,
                x_axis=x_axis,
                selected_points_px=[p[1] for p in point_map.values()],
                plane_points_px=plane_pixels,
                status="not_enough_plane_points",
            )

        points = np.asarray(plane_points, dtype=np.float64)
        pixels = list(plane_pixels)
        points, pixels = self._filter_outliers(points, pixels)
        if len(points) < self.min_valid_plane_points:
            return FacePose(
                timestamp=timestamp,
                valid=False,
                center=center,
                center_px=center_px,
                x_axis=x_axis,
                selected_points_px=[p[1] for p in point_map.values()],
                plane_points_px=pixels,
                status="too_many_outliers",
            )

        normal, rmse, inlier_points, inlier_pixels = self._fit_plane(points, pixels)
        if normal is None or rmse is None:
            return FacePose(timestamp=timestamp, valid=False, status="plane_fit_failed")
        if rmse > self.max_plane_rmse_m:
            return FacePose(
                timestamp=timestamp,
                valid=False,
                center=center,
                normal=normal,
                x_axis=x_axis,
                center_px=center_px,
                selected_points_px=[p[1] for p in point_map.values()],
                plane_points_px=inlier_pixels,
                plane_points_3d=inlier_points,
                rmse_m=rmse,
                status="plane_rmse_too_large",
            )

        normal = self._orient_normal(normal, center)
        x_axis = self._orthogonalize_axis(x_axis, normal)
        if x_axis is None:
            return FacePose(timestamp=timestamp, valid=False, status="x_axis_parallel_to_normal")

        return FacePose(
            timestamp=timestamp,
            valid=True,
            center=center,
            normal=normal,
            x_axis=x_axis,
            center_px=center_px,
            selected_points_px=[p[1] for p in point_map.values()],
            plane_points_px=inlier_pixels,
            plane_points_3d=inlier_points,
            rmse_m=rmse,
            status="valid",
        )

    def _deproject_indices(
        self,
        landmarks: Any,
        indices: list[int],
        image_width: int,
        image_height: int,
        depth_frame: Any,
        intrinsics: Any,
    ) -> dict[int, tuple[np.ndarray, tuple[int, int], float]]:
        point_map: dict[int, tuple[np.ndarray, tuple[int, int], float]] = {}
        for index in indices:
            if index < 0 or index >= len(landmarks):
                continue
            landmark = landmarks[index]
            u = float(landmark.x) * (image_width - 1)
            v = float(landmark.y) * (image_height - 1)
            projected = self.projector.deproject_pixel(
                u,
                v,
                image_width,
                image_height,
                depth_frame,
                intrinsics,
            )
            if projected is not None:
                point_map[index] = projected
        return point_map

    def _mean_point(
        self,
        point_map: dict[int, tuple[np.ndarray, tuple[int, int], float]],
        indices: list[int],
    ) -> tuple[np.ndarray, tuple[int, int]] | None:
        points = [point_map[i][0] for i in indices if i in point_map]
        pixels = [point_map[i][1] for i in indices if i in point_map]
        if len(points) < self.min_valid_eye_points:
            return None
        point = np.mean(np.asarray(points, dtype=np.float64), axis=0)
        pixel = self._mean_pixel(pixels)
        return point, pixel

    @staticmethod
    def _mean_pixel(pixels: list[tuple[int, int]]) -> tuple[int, int]:
        values = np.asarray(pixels, dtype=np.float64)
        return int(round(float(np.mean(values[:, 0])))), int(round(float(np.mean(values[:, 1]))))

    @staticmethod
    def _center_pixel(left_px: tuple[int, int], right_px: tuple[int, int]) -> tuple[int, int]:
        return (
            int(round((left_px[0] + right_px[0]) * 0.5)),
            int(round((left_px[1] + right_px[1]) * 0.5)),
        )

    @staticmethod
    def _estimate_x_axis(
        left_eye: tuple[np.ndarray, tuple[int, int]],
        right_eye: tuple[np.ndarray, tuple[int, int]],
    ) -> np.ndarray | None:
        # 按图像横坐标排序，确保 x_axis 与相机坐标系的 +X 方向一致。
        first, second = left_eye, right_eye
        if first[1][0] > second[1][0]:
            first, second = second, first
        axis = second[0] - first[0]
        return _normalize(axis)

    def _filter_outliers(
        self,
        points: np.ndarray,
        pixels: list[tuple[int, int]],
    ) -> tuple[np.ndarray, list[tuple[int, int]]]:
        if len(points) < 4:
            return points, pixels
        centroid = np.median(points, axis=0)
        distances = np.linalg.norm(points - centroid, axis=1)
        median = float(np.median(distances))
        mad = float(np.median(np.abs(distances - median)))
        if mad < 1e-9:
            return points, pixels
        threshold = median + self.outlier_sigma * 1.4826 * mad
        mask = distances <= threshold
        return points[mask], [pixel for pixel, keep in zip(pixels, mask) if bool(keep)]

    def _fit_plane(
        self,
        points: np.ndarray,
        pixels: list[tuple[int, int]],
    ) -> tuple[np.ndarray | None, float | None, np.ndarray | None, list[tuple[int, int]]]:
        if self.use_ransac and len(points) >= max(3, self.ransac_min_inliers):
            fitted = self._fit_plane_ransac(points, pixels)
            if fitted[0] is not None:
                return fitted
        normal, rmse = self._fit_plane_svd(points)
        return normal, rmse, points, pixels

    def _fit_plane_ransac(
        self,
        points: np.ndarray,
        pixels: list[tuple[int, int]],
    ) -> tuple[np.ndarray | None, float | None, np.ndarray | None, list[tuple[int, int]]]:
        rng = np.random.default_rng()
        best_mask = None
        best_count = 0
        best_rmse = float("inf")

        for _ in range(self.ransac_iterations):
            sample_ids = rng.choice(len(points), size=3, replace=False)
            sample = points[sample_ids]
            normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
            normal = _normalize(normal)
            if normal is None:
                continue

            distances = np.abs((points - sample[0]) @ normal)
            mask = distances <= self.ransac_threshold_m
            count = int(np.count_nonzero(mask))
            if count < self.ransac_min_inliers:
                continue

            _, rmse = self._fit_plane_svd(points[mask])
            if count > best_count or (count == best_count and rmse is not None and rmse < best_rmse):
                best_mask = mask
                best_count = count
                best_rmse = float(rmse)

        if best_mask is None:
            return None, None, None, []

        inlier_points = points[best_mask]
        inlier_pixels = [pixel for pixel, keep in zip(pixels, best_mask) if bool(keep)]
        normal, rmse = self._fit_plane_svd(inlier_points)
        return normal, rmse, inlier_points, inlier_pixels

    @staticmethod
    def _fit_plane_svd(points: np.ndarray) -> tuple[np.ndarray | None, float | None]:
        if len(points) < 3:
            return None, None
        centroid = np.mean(points, axis=0)
        centered = points - centroid
        try:
            _, _, vh = np.linalg.svd(centered, full_matrices=False)
        except np.linalg.LinAlgError:
            return None, None
        normal = _normalize(vh[-1])
        if normal is None:
            return None, None
        distances = centered @ normal
        rmse = float(np.sqrt(np.mean(distances * distances)))
        return normal, rmse

    def _orient_normal(self, normal: np.ndarray, center: np.ndarray) -> np.ndarray:
        dot = float(np.dot(normal, center))
        if self.normal_towards_camera and dot > 0:
            normal = -normal
        if not self.normal_towards_camera and dot < 0:
            normal = -normal
        return normal

    @staticmethod
    def _orthogonalize_axis(axis: np.ndarray, normal: np.ndarray) -> np.ndarray | None:
        axis = axis - normal * float(np.dot(axis, normal))
        return _normalize(axis)


def _normalize(vector: np.ndarray) -> np.ndarray | None:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        return None
    return vector / norm


def _array_to_list(value: np.ndarray | None) -> list[float] | None:
    if value is None:
        return None
    return [float(x) for x in value.tolist()]


def _imu_to_dict(value: dict[str, tuple[float, float, float]] | None) -> dict[str, list[float]] | None:
    if not value:
        return None
    return {key: [float(x) for x in sample] for key, sample in value.items()}
