from __future__ import annotations

import json
import os
import time
from pathlib import Path

from pose_types import FacePose


class JsonPoseFileWriter:
    def __init__(self, path: str | Path, replace_retries: int = 5, retry_delay_s: float = 0.002) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.tmp_path = self.path.with_name(f"{self.path.name}.tmp")
        self.replace_retries = int(replace_retries)
        self.retry_delay_s = float(retry_delay_s)
        self.last_warning_time = 0.0

    def write(self, pose: FacePose) -> None:
        payload = json.dumps(pose.to_udp_dict(), ensure_ascii=False, separators=(",", ":"))
        self.tmp_path.write_text(payload, encoding="utf-8")
        for attempt in range(self.replace_retries + 1):
            try:
                os.replace(self.tmp_path, self.path)
                return
            except PermissionError:
                if attempt >= self.replace_retries:
                    self._warn_throttled("pose JSON replace was locked; skipped one frame")
                    return
                time.sleep(self.retry_delay_s)

    def _warn_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_warning_time >= 2.0:
            print(f"[pose-file warning] {message}", flush=True)
            self.last_warning_time = now
