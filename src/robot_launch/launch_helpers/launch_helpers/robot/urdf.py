"""URDF generation from a robot's xacro: the ros2_control view and the OCS2 planning view.

Both views render the same robot description package with different xacro parameters:
- ros2_control: ``xacro/ros2_control/robot.xacro`` (includes ``robot.xacro`` plus the hardware blocks),
  end-effector joints stay actuated; the XML string is cached in-process.
- planning: ``xacro/robot.xacro`` with chassis and end-effector joints frozen; the XML is written to a
  content-addressed file because OCS2 reads its URDF by path.
"""

import hashlib
import os
import re
import stat
import tempfile
from typing import Optional, Sequence

import xacro

from .config import get_robot_package_path
from .profile import build_xacro_mappings, resolve_profile_path

# Entry points inside <robot>_description/.
CONTROL_XACRO = ("xacro", "ros2_control", "robot.xacro")
PLANNING_XACRO = ("xacro", "robot.xacro")

PLANNING_SCOPE_ARMS = "arms"
PLANNING_SCOPE_FULL = "full"


# --- xacro rendering shared by both views ---------------------------------------------------------

def resolve_robot_xacro(robot_name: str, xacro_relpath: Sequence[str]) -> Optional[str]:
    """Absolute path of <robot_name>_description/<xacro_relpath>, or None when package or file is missing."""
    robot_pkg_path = get_robot_package_path(robot_name)
    if robot_pkg_path is None:
        return None
    xacro_file = os.path.join(robot_pkg_path, *xacro_relpath)
    if not os.path.isfile(xacro_file):
        print(f"[WARN] xacro file not found: {xacro_file}")
        return None
    return xacro_file


def render_robot_xacro(robot_name: str, xacro_relpath: Sequence[str], mappings: dict) -> Optional[str]:
    """Process the robot's xacro with mappings and return the URDF XML; None after logging on any failure."""
    xacro_file = resolve_robot_xacro(robot_name, xacro_relpath)
    if xacro_file is None:
        return None
    try:
        return xacro.process_file(xacro_file, mappings=mappings).toxml()
    except Exception as error:
        print(f"[ERROR] Failed to render {xacro_file} for '{robot_name}': {error}")
        return None


# --- ros2_control view ----------------------------------------------------------------------------

_robot_description_cache = {}

def clear_robot_description_cache():
    """
    清除 robot_description 缓存

    在开发或测试时，如果 xacro 文件被修改，可以调用此函数清除缓存
    """
    global _robot_description_cache
    _robot_description_cache.clear()
    print("[INFO] Robot description cache cleared")

def _mappings_cache_key(robot_name, hardware, mappings):
    items = sorted((str(k), str(v)) for k, v in mappings.items())
    return f"{robot_name}_{hardware}_{items}"


def get_ros2_control_robot_description(
    robot_name,
    robot_type="",
    hardware="mock_components",
    mappings=None,
    launch_configurations=None,
    robot_profile=None,
):
    """
    Generate ros2_control robot_description from xacro with optional mappings dict
    or launch profile / prefixed arguments.
    """
    if mappings is None:
        if launch_configurations is not None:
            profile_path = robot_profile or resolve_profile_path(launch_configurations)
            mappings = build_xacro_mappings(hardware, launch_configurations, profile_path or None)
        else:
            mappings = {"ros2_control_hardware_type": hardware}
            if robot_type and robot_type.strip():
                mappings["arms"] = robot_type  # robot_type is the arm topology (m6_ccs: dual | left | right)
    # ros2_control: EEF stays actuated for the gripper controllers; OCS2 uses planning URDF only.
    mappings["eef_fixed_joints"] = "false"

    cache_key = _mappings_cache_key(robot_name, hardware, mappings)
    if cache_key in _robot_description_cache:
        return _robot_description_cache[cache_key]

    robot_description = render_robot_xacro(robot_name, CONTROL_XACRO, mappings)
    if robot_description is None:
        return None
    _robot_description_cache[cache_key] = robot_description
    return robot_description


# --- OCS2 planning view ---------------------------------------------------------------------------

_ROBOT_NAME_PATTERN = re.compile(r"[A-Za-z0-9_]+")


def _cache_root():
    """Root of the planning URDF cache, same preference as the OCS2 library folder in the controllers:
    OCS2_PLANNING_URDF_DIR, $XDG_CACHE_HOME, ~/.ros/ocs2_cache, and as a last resort a per-user /tmp directory."""
    if override := os.environ.get("OCS2_PLANNING_URDF_DIR", "").strip():
        return os.path.join(override, "ocs2_ros2")
    if xdg := os.environ.get("XDG_CACHE_HOME", "").strip():
        return os.path.join(xdg, "ocs2_ros2")
    if home := os.environ.get("HOME", "").strip():
        return os.path.join(home, ".ros", "ocs2_cache")
    return os.path.join("/tmp", f"ocs2_ros2-{os.getuid()}")


def _require_private_directory(path):
    """The directory must be a real directory owned by us that nobody else can write to; otherwise another
    local user could plant a URDF that we would load as our own."""
    info = os.lstat(path)
    if not stat.S_ISDIR(info.st_mode):
        raise PermissionError(f"{path} is not a directory (symlinks are not followed)")
    if info.st_uid != os.getuid():
        raise PermissionError(f"{path} is owned by uid {info.st_uid}, not by the current user")
    if stat.S_IMODE(info.st_mode) & 0o022:
        raise PermissionError(f"{path} is writable by other users; refusing to use it for the URDF cache")


def get_planning_urdf_cache_dir(robot_name):
    """Private, writable cache directory for the generated planning URDF files of one robot."""
    if not _ROBOT_NAME_PATTERN.fullmatch(robot_name or ""):
        raise ValueError(f"invalid robot name for the planning URDF cache: {robot_name!r}")
    root = _cache_root()
    cache_dir = os.path.join(root, robot_name, "planning_urdf")
    os.makedirs(cache_dir, mode=0o700, exist_ok=True)
    _require_private_directory(root)
    _require_private_directory(cache_dir)
    return cache_dir


def _content_digest(urdf_content):
    return hashlib.sha256(urdf_content.encode("utf-8")).hexdigest()[:16]


def _file_holds_content(path, digest):
    """True when the file exists and its content really hashes to the digest in its name."""
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return _content_digest(handle.read()) == digest
    except (OSError, UnicodeDecodeError):
        return False


def write_planning_urdf_cache(robot_name, urdf_content):
    """Store planning URDF XML under a name derived from its content; returns the absolute path.

    The file name is the hash of the generated XML, so an unchanged xacro always maps to the same
    path and a changed xacro (or changed mesh/collider include) maps to a new one. An existing file is
    reused only when its content still matches that hash. The write is atomic (temp file + rename) so
    a concurrent launch never reads a half-written file.
    """
    if not urdf_content:
        return ""
    digest = _content_digest(urdf_content)
    cache_path = os.path.abspath(
        os.path.join(get_planning_urdf_cache_dir(robot_name), f"{digest}.urdf")
    )
    if _file_holds_content(cache_path, digest):
        print(f"[INFO] Planning URDF unchanged, reusing: {cache_path}")
        return cache_path

    cache_dir = os.path.dirname(cache_path)
    fd, tmp_path = tempfile.mkstemp(dir=cache_dir, suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(urdf_content)
        os.replace(tmp_path, cache_path)
    except OSError:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
        raise
    print(f"[INFO] Wrote planning URDF: {cache_path}")
    return cache_path


_XACRO_SIDE_EEF_SUPPORT_CACHE: dict[str, bool] = {}
_XACRO_SIDE_EEF_ARG_PATTERN = re.compile(
    r'<xacro:arg\s+name=["\'](?P<name>left_end_effector|right_end_effector)["\']'
)


def _planning_xacro_supports_side_eef(robot_name: str) -> bool:
    """True when planning xacro declares per-side end effector args (left_end_effector / right_end_effector)."""
    if robot_name in _XACRO_SIDE_EEF_SUPPORT_CACHE:
        return _XACRO_SIDE_EEF_SUPPORT_CACHE[robot_name]

    planning_xacro = resolve_robot_xacro(robot_name, PLANNING_XACRO)
    if planning_xacro is None:
        _XACRO_SIDE_EEF_SUPPORT_CACHE[robot_name] = False
        return False

    try:
        with open(planning_xacro, "r", encoding="utf-8") as xacro_file:
            declared = {
                match.group("name")
                for match in _XACRO_SIDE_EEF_ARG_PATTERN.finditer(xacro_file.read())
            }
    except OSError:
        _XACRO_SIDE_EEF_SUPPORT_CACHE[robot_name] = False
        return False

    supported = declared.issuperset({"left_end_effector", "right_end_effector"})
    _XACRO_SIDE_EEF_SUPPORT_CACHE[robot_name] = supported
    return supported


# Public alias for control.compose single/dual-arm topology detection.
planning_xacro_supports_side_eef = _planning_xacro_supports_side_eef


def _resolve_planning_scope(planning_robot_name, launch_configurations, planning_scope=""):
    configs = launch_configurations or {}
    scope = (planning_scope or configs.get("planning_scope", "")).strip().lower()
    if scope in (PLANNING_SCOPE_ARMS, PLANNING_SCOPE_FULL):
        return scope
    if planning_robot_name == "m6_ccs":
        return PLANNING_SCOPE_ARMS
    return PLANNING_SCOPE_FULL


def _resolve_planning_robot_name(planning_robot_name, planning_scope):
    """Use arm_controller.robot_name from config; do not override per scope."""
    return planning_robot_name


def _planning_xacro_mappings(
    hardware,
    launch_configurations,
    robot_profile=None,
    planning_scope=PLANNING_SCOPE_FULL,
    planning_robot_name="",
):
    """Xacro mappings for OCS2 planning URDF (kinematic tree without EEF/chassis actuation DOF)."""
    configs = launch_configurations or {}

    mappings = build_xacro_mappings(hardware, configs, robot_profile)
    mappings.pop("ros2_control_hardware_type", None)
    mappings.pop("gazebo", None)
    # Planning URDF: freeze chassis wheels/steer and EEF actuation in xacro (not via .info removeJoints).
    mappings["eef_fixed_joints"] = "true"
    mappings["chassis_joints_movable"] = "false"

    if planning_scope == PLANNING_SCOPE_ARMS:
        mappings.pop("chassis", None)
        if not _planning_xacro_supports_side_eef(planning_robot_name):
            # xacro without per-side end effectors: do not pass them
            for key in ("left_end_effector", "right_end_effector", "left_end_effector_bus", "right_end_effector_bus"):
                mappings.pop(key, None)

    return mappings


def build_planning_urdf_launch_params(
    planning_robot_name,
    launch_configurations,
    hardware="mock_components",
    robot_profile=None,
    planning_scope="",
):
    """
    Generate planning URDF from xacro/robot.xacro and return controller params.

    planning_scope:
      - arms  : split-body OCS2 — dual arms from arm_base (m6_ccs xacro)
      - full  : full-body WBC — whole robot (e.g. fa_w2 xacro)

    Returns dict with planning_urdf_variant / planning_urdf_path, or empty dict
    when xacro planning URDF cannot be generated.
    """
    configs = launch_configurations or {}

    profile_path = robot_profile
    if not profile_path:
        profile_path = resolve_profile_path(configs) or None

    scope = _resolve_planning_scope(planning_robot_name, configs, planning_scope)
    effective_robot_name = _resolve_planning_robot_name(planning_robot_name, scope)
    if effective_robot_name != planning_robot_name:
        print(
            f"[INFO] planning_scope={scope}: using '{effective_robot_name}' "
            f"planning URDF (launch/config robot: {planning_robot_name})"
        )

    # Always regenerate: xacro is cheap, and the file name comes from the generated content, so an
    # edited xacro/include can never be served from a stale cache entry.
    urdf_xml = get_planning_robot_description(
        effective_robot_name,
        launch_configurations=configs,
        robot_profile=profile_path,
        hardware=hardware,
        planning_scope=scope,
    )
    if not urdf_xml:
        print(f"[WARN] Failed to generate planning URDF from xacro for '{effective_robot_name}'")
        return {}

    written_path = write_planning_urdf_cache(effective_robot_name, urdf_xml)
    if not written_path:
        return {}

    return {
        "planning_urdf_variant": "xacro",
        "planning_urdf_path": written_path,
    }


def resolve_planning_urdf_file_or_fail(
    robot_name,
    launch_configurations,
    hardware="mock_components",
    robot_profile=None,
    planning_scope="",
):
    """Return cached xacro planning URDF path, or None after logging an error."""
    profile_path = robot_profile
    if not profile_path:
        profile_path = resolve_profile_path(launch_configurations or {}) or None

    params = build_planning_urdf_launch_params(
        robot_name,
        launch_configurations,
        hardware,
        profile_path,
        planning_scope=planning_scope,
    )
    path = (params.get("planning_urdf_path") or "").strip()
    if params.get("planning_urdf_variant") == "xacro" and path and os.path.isfile(path):
        return path

    scope_label = planning_scope or "default"
    print(
        f"[ERROR] Failed to resolve xacro planning URDF for '{robot_name}' "
        f"(scope={scope_label}): variant={params.get('planning_urdf_variant')!r} path={path!r}"
    )
    return None


def get_planning_robot_description(
    robot_name,
    launch_configurations=None,
    robot_profile=None,
    hardware="mock_components",
    planning_scope="",
):
    """Generate kinematic planning URDF from xacro/robot.xacro (no ros2_control)."""
    profile_path = robot_profile
    if not profile_path and launch_configurations is not None:
        profile_path = resolve_profile_path(launch_configurations) or None

    scope = _resolve_planning_scope(robot_name, launch_configurations or {}, planning_scope)
    effective_robot_name = _resolve_planning_robot_name(robot_name, scope)
    mappings = _planning_xacro_mappings(
        hardware,
        launch_configurations or {},
        profile_path,
        scope,
        effective_robot_name,
    )
    return render_robot_xacro(effective_robot_name, PLANNING_XACRO, mappings)
