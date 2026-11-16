"""Offline tools of the logging system: the fault code registry and the flight recorder decoder.

They read files only (config tables, ring files, logs), need no ROS, and serve the whole chain, not one package.
Design: src/robot_utils/README.md.
"""
import os

PACKAGE = "robot_utils"  # the ament package that installs config/ (and these modules)


def config_path(name: str) -> str:
    """A table of robot_utils/config: in the source tree next to python/, else the installed share directory.

    Raises when it is in neither (FileNotFoundError naming both places; ament_index_python's own error when the
    package is not installed), instead of handing back a path that does not exist."""
    package_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    beside = os.path.join(package_root, "config", name)
    if os.path.isfile(beside):
        return beside
    from ament_index_python.packages import get_package_share_directory
    installed = os.path.join(get_package_share_directory(PACKAGE), "config", name)
    if not os.path.isfile(installed):
        raise FileNotFoundError(f"{name}: not in the source tree ({beside}) and not installed ({installed})")
    return installed
