"""Low-overhead, thread-safe, append-only experiment logger for Office Robot.

No ROS, Qt or pandas dependencies. Imported by office_hmi_gui.main.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import re
import threading
import time
import uuid
from datetime import datetime, timezone


def _clean_label(value: str) -> str:
    return re.sub(r"[^a-zA-Z0-9_.-]+", "_", value.strip())[:60] or "unlabeled"


class ExperimentLogger:
    """Create one log directory per Office HMI process.

    Events use UTC wall time for display and monotonic nanoseconds for durations.
    This class deliberately does not log API tokens, HTTP headers or full HTTP
    request objects. NLP prompts ARE stored; use only non-sensitive test prompts.
    """

    def __init__(self, root_dir: str | None = None):
        root = Path(root_dir or os.getenv("OFFICE_METRICS_ROOT", "~/.ros/office_robot_metrics")).expanduser()
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        suffix = uuid.uuid4().hex[:7]
        self.run_id = f"{stamp}_{suffix}"
        self.run_dir = root / self.run_id
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.events_path = self.run_dir / "events.jsonl"
        self.scenario = _clean_label(os.getenv("OFFICE_SCENARIO", "unspecified"))
        self.variant = _clean_label(os.getenv("OFFICE_VARIANT", "unspecified"))
        self._lock = threading.Lock()
        self.log("run_meta", scenario=self.scenario, variant=self.variant,
                 description="Times are host-monotonic unless labeled nav_time_s (Nav2 feedback)")

    @staticmethod
    def new_id(prefix: str) -> str:
        return f"{_clean_label(prefix)}_{uuid.uuid4().hex[:12]}"

    def log(self, event: str, **fields) -> None:
        payload = {
            "event": event,
            "run_id": self.run_id,
            "scenario": self.scenario,
            "variant": self.variant,
            "utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "mono_ns": time.monotonic_ns(),
            **fields,
        }
        encoded = json.dumps(payload, ensure_ascii=False, allow_nan=False, separators=(",", ":"), default=str)
        with self._lock:
            with self.events_path.open("a", encoding="utf-8") as out:
                out.write(encoded + "\n")
