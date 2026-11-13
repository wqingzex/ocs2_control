"""logs/events.log: one JSON line per event (unit start / stop / exit, mode changes) so the per-unit
logs can be lined up in time. Events are rare, so every write opens, appends and closes: nothing stays open or
buffered, and a full disk or a read-only log directory never stops teleoperation."""
from __future__ import annotations

import json
import os
from datetime import datetime
from typing import Any, Dict, List, Optional

MAX_BYTES = 5 * 1024 * 1024  # events.log rolls over to events.log.1


class EventLog:
    def __init__(self, log_dir: str):
        self.path = os.path.join(log_dir, "events.log")

    def record(self, kind: str, unit: Optional[str] = None, **fields: Any) -> None:
        event: Dict[str, Any] = {"time": datetime.now().isoformat(timespec="milliseconds"), "kind": kind}
        if unit:
            event["unit"] = unit
        event.update({key: value for key, value in fields.items() if value is not None})
        try:
            if os.path.getsize(self.path) > MAX_BYTES:
                os.replace(self.path, self.path + ".1")
        except OSError:
            pass  # no file yet
        try:
            with open(self.path, "a", encoding="utf-8") as handle:
                handle.write(json.dumps(event, ensure_ascii=False, default=str) + "\n")
        except OSError:
            pass

    def tail(self, count: int = 200) -> List[str]:
        try:
            with open(self.path, encoding="utf-8", errors="replace") as handle:
                return handle.read().splitlines()[-count:]
        except OSError:
            return []
