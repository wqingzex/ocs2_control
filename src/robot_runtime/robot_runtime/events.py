"""Per-unit log housekeeping: logs/<unit>/<stamp>.log keeps the newest few runs, latest.log is a link."""
from __future__ import annotations

import os
import re

KEEP_UNIT_LOGS = 20          # newest <stamp>.log files kept per unit
_STAMPED = re.compile(r"^\d{8}_\d{6}\.log$")


def prune_unit_logs(unit_dir: str, keep: int = KEEP_UNIT_LOGS) -> None:
    """Delete the oldest <stamp>.log files of one unit beyond `keep` (latest.log is a link and stays)."""
    try:
        stamped = sorted(name for name in os.listdir(unit_dir) if _STAMPED.match(name))
    except OSError:
        return
    for name in stamped[:-keep] if keep > 0 else stamped:
        try:
            os.remove(os.path.join(unit_dir, name))
        except OSError:
            pass
