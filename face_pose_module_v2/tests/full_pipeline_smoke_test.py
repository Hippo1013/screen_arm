from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run v2 full pipeline for a fixed number of camera frames.")
    parser.add_argument("--frames", type=int, default=90)
    parser.add_argument("--config", default="../config.yaml")
    parser.add_argument("--with-window", action="store_true")
    parser.add_argument("--with-udp", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    test_dir = Path(__file__).resolve().parent
    module_dir = test_dir.parent
    main_path = module_dir / "main.py"
    config_path = Path(args.config)
    if not config_path.is_absolute():
        config_path = test_dir / config_path

    command = [
        sys.executable,
        str(main_path),
        "--config",
        str(config_path.resolve()),
        "--max-frames",
        str(args.frames),
    ]
    if not args.with_window:
        command.append("--no-window")
    if not args.with_udp:
        command.append("--no-udp")

    return subprocess.call(command, cwd=str(module_dir))


if __name__ == "__main__":
    raise SystemExit(main())
