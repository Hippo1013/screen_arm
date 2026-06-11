from __future__ import annotations

import argparse
import socket
import time


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="UDP JSON 接收测试")
    parser.add_argument("--ip", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--seconds", type=float, default=10.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.ip, args.port))
    sock.settimeout(0.5)
    print(f"listening udp://{args.ip}:{args.port}")

    deadline = time.time() + args.seconds
    while time.time() < deadline:
        try:
            data, address = sock.recvfrom(4096)
        except socket.timeout:
            continue
        print(f"{address}: {data.decode('utf-8', errors='replace')}")

    sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
