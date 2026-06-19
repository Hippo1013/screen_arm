import json
import math
import socket
import sys
import time


def main() -> None:
    mode = sys.argv[1] if len(sys.argv) > 1 else "sweep"
    addr = ("127.0.0.1", 5005)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    t0 = time.time()

    while True:
        t = time.time() - t0
        angle = 0.0
        if mode == "sweep":
            angle = math.radians(25) * math.sin(0.2 * t)
        elif mode == "step":
            angle = math.radians(35) if int(t) % 6 < 3 else math.radians(-35)
        elif mode == "jitter":
            angle = (
                math.radians(25) * math.sin(0.2 * t)
                + math.radians(3) * math.sin(40 * t)
            )

        normal_camera = [math.sin(angle), 0.0, -math.cos(angle)]
        pose = {
            "t": t,
            "valid": mode != "invalid" or int(t) % 4 != 0,
            "normal": normal_camera,
            "imu": {"accel": [0.0, 9.81, 0.0]},
        }
        if mode == "farfield":
            pose["normal"] = [0.0, 0.0, -1.0]

        sock.sendto(json.dumps(pose).encode("utf-8"), addr)
        time.sleep(0.02)


if __name__ == "__main__":
    main()
