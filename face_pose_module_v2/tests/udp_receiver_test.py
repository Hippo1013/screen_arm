from __future__ import annotations

import argparse
import socket
import time


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Receive face_pose_module_v2 UDP JSON packets.")
    parser.add_argument("--ip", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--seconds", type=float, default=30.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.ip, args.port))
    sock.settimeout(0.5)
    deadline = time.monotonic() + args.seconds
    count = 0
    try:
        while time.monotonic() < deadline:
            try:
                data, address = sock.recvfrom(65535)
            except socket.timeout:
                continue
            count += 1
            print(f"[{count}] {address}: {data.decode('utf-8', errors='replace')}")
    finally:
        sock.close()
    print(f"Received {count} packets.")
    return 0 if count > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
