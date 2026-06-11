from __future__ import annotations

import json
import socket
from typing import Any

from face_geometry import FacePose


class UdpPoseSender:
    def __init__(self, config: dict[str, Any]) -> None:
        self.address = (str(config["ip"]), int(config["port"]))
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, pose: FacePose) -> None:
        payload = json.dumps(pose.to_udp_dict(), ensure_ascii=False, separators=(",", ":"))
        self.socket.sendto(payload.encode("utf-8"), self.address)

    def close(self) -> None:
        self.socket.close()
