"""Offline tools of the logging system: the fault code registry and the flight recorder decoder.

They read files only (config tables, ring files, logs), need no ROS, and serve the whole chain, not one package.
Design: docs/日志.md.
"""
import os
from typing import Optional

PACKAGE = "robot_log"


def config_path(name: str) -> str:
    """A table of robot_log/config: in the source tree next to python/, else the installed share directory."""
    package_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    beside = os.path.join(package_root, "config", name)
    if os.path.isfile(beside):
        return beside
    installed: Optional[str] = None
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = os.path.join(get_package_share_directory(PACKAGE), "config", name)
    except (ImportError, LookupError):
        pass
    return installed if installed and os.path.isfile(installed) else beside
