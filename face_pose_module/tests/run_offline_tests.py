from __future__ import annotations

import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml

MODULE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MODULE_DIR))

from face_geometry import FaceGeometryEstimator, FacePose  # noqa: E402
from filters import PoseStabilizer  # noqa: E402


class FakeProjector:
    def deproject_pixel(self, u, v, image_width, image_height, depth_frame, intrinsics):
        x = (float(u) - image_width * 0.5) / 900.0
        y = (float(v) - image_height * 0.5) / 900.0
        z = 0.80 + 0.04 * x - 0.02 * y
        return np.asarray([x, y, z], dtype=np.float64), (int(round(u)), int(round(v))), z


def load_config() -> dict:
    with (MODULE_DIR / "config.yaml").open("r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def test_config() -> None:
    config = load_config()
    assert config["camera"]["depth_width"] == 1280
    assert config["camera"]["color_width"] == 1280
    assert config["camera"]["enable_imu"] is True
    assert config["camera"]["enable_infrared"] is False
    assert config["mediapipe"]["num_faces"] == 1
    assert config["udp"]["ip"] == "127.0.0.1"
    assert config["udp"]["port"] == 5005


def test_geometry_estimator() -> None:
    geometry_config = {
        "left_eye_indices": [0, 1, 2],
        "right_eye_indices": [3, 4, 5],
        "plane_indices": list(range(21)),
        "min_depth_m": 0.2,
        "max_depth_m": 2.5,
        "depth_window_size": 5,
        "depth_valid_ratio": 0.35,
        "min_valid_plane_points": 10,
        "min_valid_eye_points": 2,
        "outlier_sigma": 3.0,
        "use_ransac": True,
        "ransac_iterations": 24,
        "ransac_threshold_m": 0.01,
        "ransac_min_inliers": 10,
        "max_plane_rmse_m": 0.01,
        "normal_towards_camera": True,
    }
    estimator = FaceGeometryEstimator(geometry_config, FakeProjector())

    coords = [
        (0.42, 0.44),
        (0.43, 0.46),
        (0.44, 0.45),
        (0.56, 0.44),
        (0.57, 0.46),
        (0.58, 0.45),
        (0.50, 0.38),
        (0.48, 0.42),
        (0.52, 0.42),
        (0.46, 0.50),
        (0.54, 0.50),
        (0.44, 0.56),
        (0.56, 0.56),
        (0.50, 0.58),
        (0.40, 0.50),
        (0.60, 0.50),
        (0.45, 0.36),
        (0.55, 0.36),
        (0.47, 0.62),
        (0.53, 0.62),
        (0.50, 0.50),
    ]
    landmarks = [SimpleNamespace(x=x, y=y) for x, y in coords]
    result = SimpleNamespace(face_landmarks=[landmarks])

    pose = estimator.estimate(result, (720, 1280, 3), None, None, timestamp=1.0)
    assert pose.valid, pose.status
    assert pose.center is not None
    assert pose.normal is not None
    assert pose.x_axis is not None
    assert abs(np.linalg.norm(pose.normal) - 1.0) < 1e-9
    assert abs(np.linalg.norm(pose.x_axis) - 1.0) < 1e-9
    assert float(np.dot(pose.normal, pose.center)) < 0.0
    assert pose.rmse_m is not None and pose.rmse_m < 0.01


def test_filter_and_udp_payload() -> None:
    config = {
        "enabled": True,
        "ema_alpha_center": 0.35,
        "ema_alpha_axis": 0.35,
        "moving_average_window": 3,
        "center_deadband_m": 0.003,
        "normal_deadband_deg": 1.0,
        "x_axis_deadband_deg": 1.0,
        "hold_last_valid_seconds": 0.25,
        "hold_marks_valid": True,
    }
    stabilizer = PoseStabilizer(config)
    pose1 = FacePose(
        timestamp=10.0,
        valid=True,
        center=np.asarray([0.0, 0.0, 0.8]),
        normal=np.asarray([0.0, 0.0, -1.0]),
        x_axis=np.asarray([1.0, 0.0, 0.0]),
        imu={"accel": (0.0, -9.8, 0.1), "gyro": (0.01, 0.02, 0.03)},
        status="valid",
    )
    out1 = stabilizer.update(pose1)
    assert out1.valid

    invalid = FacePose(timestamp=10.1, valid=False, status="no_face")
    held = stabilizer.update(invalid)
    assert held.valid
    assert held.status == "lost_hold"

    payload = held.to_udp_dict()
    encoded = json.dumps(payload)
    assert '"valid": true' in encoded
    assert payload["center"] == [0.0, 0.0, 0.8]
    assert payload["imu"]["accel"] == [0.0, -9.8, 0.1]


def main() -> int:
    tests = [test_config, test_geometry_estimator, test_filter_and_udp_payload]
    for test in tests:
        test()
        print(f"[ok] {test.__name__}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
