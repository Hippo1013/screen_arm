from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path

import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Import smoke test for face_pose_module_v2.")
    parser.add_argument("--config", default="../config.yaml")
    parser.add_argument("--load-model", action="store_true", help="Instantiate the 3DDFA model")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    test_dir = Path(__file__).resolve().parent
    module_dir = test_dir.parent
    sys.path.insert(0, str(module_dir))

    required_modules = ["cv2", "numpy", "yaml", "pyrealsense2", "torch", "torchvision"]
    missing = [name for name in required_modules if importlib.util.find_spec(name) is None]
    if missing:
        print("Missing modules: " + ", ".join(missing))
        return 2

    repo_dir = module_dir / "third_party" / "3DDFA_V2"
    required_files = [
        repo_dir / "TDDFA.py",
        repo_dir / "weights" / "mb1_120x120.pth",
        repo_dir / "configs" / "bfm_noneck_v3.pkl",
    ]
    missing_files = [path for path in required_files if not path.exists()]
    if missing_files:
        for path in missing_files:
            print(f"Missing file: {path}")
        return 3

    if args.load_model:
        from three_ddfa_pose import ThreeDDFAPoseEstimator

        config_path = Path(args.config)
        if not config_path.is_absolute():
            config_path = test_dir / config_path
        with config_path.resolve().open("r", encoding="utf-8") as file:
            config = yaml.safe_load(file)
        ThreeDDFAPoseEstimator(config["three_ddfa"], module_dir)

    print("face_pose_module_v2 import smoke test passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
