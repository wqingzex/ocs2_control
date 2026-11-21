"""Configure home_1 for the motion controllers of the selected robot entry."""
from __future__ import annotations

import math
import os
import re
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from launch_helpers import (build_xacro_mappings, load_robot_config, load_robot_profile, resolve_control_patch,
                            resolve_control_sides, resolve_robot_variant)
from launch_helpers.robot.urdf import PLANNING_XACRO, render_robot_xacro
from robot_config import config_file
from robot_config.setup_steps import main_for

from .config import load_lenient
from .robot import MOTION_CONTROLLERS, controller_launch


def home_settings(path):
    cfg = load_lenient(path)
    _, args = controller_launch(cfg, path)
    profile = load_robot_profile(path)
    left, right = resolve_control_sides(args, profile)
    merged, _, _ = load_robot_config(args["robot"], control_left=left, control_right=right,
                                     control_patch=resolve_control_patch(profile),
                                     robot_variant=resolve_robot_variant(args, profile))
    names = MOTION_CONTROLLERS[cfg["robot"]["entry"]]
    settings = {}
    for name in names:
        params = merged.get(name, {}).get("ros__parameters", {})
        joints = params.get("joints")
        if not joints:
            raise ValueError(f"{name} 未定义 HOME 使用的关节顺序")
        settings[name] = (joints, params.get("home_1", []))

    # HOME can be configured before serial devices. Only render the robot's geometry and joint limits.
    mappings = build_xacro_mappings("mock_components", args, path)
    xml = render_robot_xacro(args["robot"], PLANNING_XACRO, mappings)
    limits = {}
    for joint in ET.fromstring(xml).findall("joint"):
        if joint.get("type") == "continuous":
            limits[joint.get("name")] = (-math.inf, math.inf)
        elif (limit := joint.find("limit")) is not None and "lower" in limit.attrib and "upper" in limit.attrib:
            limits[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))
    return settings, limits


def read_joint_positions(required, timeout_s=3.0):
    """Take one complete sample from the state broadcaster in the current ROS domain."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState

    sample = {}

    def receive(message):
        positions = dict(zip(message.name, message.position))
        if len(message.name) == len(message.position) and required.issubset(positions):
            sample.update(positions)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = rclpy.create_node("setup_home_snapshot")
        node.create_subscription(JointState, "/joint_states", receive, qos_profile_sensor_data)
        deadline = time.monotonic() + timeout_s
        while not sample and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    if not sample:
        raise RuntimeError("没有收到包含全部 HOME 关节的 /joint_states，请检查运控的关节反馈或改用手动输入")
    return sample


def home_updates(settings, limits, candidates):
    updates = {}
    for name, (joints, current) in settings.items():
        values = candidates[name]
        if not isinstance(values, list) or len(values) != len(joints):
            raise ValueError(f"{name} 的 HOME 需要 {len(joints)} 个关节值")
        for joint, value in zip(joints, values):
            if joint not in limits:
                raise ValueError(f"机器人模型缺少 {joint} 的关节限位")
            low, high = limits[joint]
            if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{joint} 的 HOME 值 {value} 超出模型范围 [{low:.4f}, {high:.4f}] rad")
        if values != current:
            updates[name] = {"ros__parameters": {"home_1": [float(value) for value in values]}}
    return updates


def save_home(path, original, updates):
    if updates:
        if Path(path).read_text(encoding="utf-8") != original:
            raise RuntimeError("本机配置在设置期间已变化，请重新运行 setup.sh home")
        config_file.set_fields(path, "control.patch", updates)


def required_joints(settings):
    return {joint for joints, _ in settings.values() for joint in joints}


def terminal_home(path, original, settings, limits, io):
    for name, (joints, home) in settings.items():
        io.info(f"{name} 关节顺序：{', '.join(joints)}")
        io.info(f"当前 HOME（rad）：{list(home) if isinstance(home, list) else home}")
    method = io.ask("HOME 设置 keep（沿用）| capture（读取当前反馈）| manual（输入弧度）", "keep").lower()
    if method not in {"keep", "capture", "manual"}:
        raise ValueError("HOME 设置请选择 keep、capture 或 manual")
    sample = read_joint_positions(required_joints(settings)) if method == "capture" else {}

    candidates = {}
    for name, (joints, home) in settings.items():
        if method == "capture":
            home = [sample[joint] for joint in joints]
        elif method == "manual":
            text = io.ask(f"{name} 输入 {len(joints)} 个弧度值（空格或逗号分隔）", "keep")
            if text.lower() != "keep":
                try:
                    home = [float(value) for value in re.split(r"[\s,]+", text.strip("[] \t"))]
                except ValueError as exc:
                    raise ValueError(f"{name} 的 HOME 值必须是弧度数值") from exc
        candidates[name] = home

    updates = home_updates(settings, limits, candidates)
    for name in updates:
        io.info(f"{name} 新 HOME（rad）：{candidates[name]}")
    if updates and io.ask("保存上述 HOME 到本机配置？y/n", "n").lower() in ("y", "yes", "是"):
        save_home(path, original, updates)
        io.info("已保存 HOME，下次启动运控时加载")
    return True


def step_home(path, _ws_root, io):
    original = Path(path).read_text(encoding="utf-8")
    settings, limits = home_settings(path)
    if not io.assume_yes and (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        try:
            from .home_window import run_home_window
        except ImportError as exc:
            raise RuntimeError("缺少 HOME 窗口依赖 python3-pyqt5，请先运行 ./scripts/setup.sh deps") from exc

        return run_home_window(path, original, settings, limits, io)
    return terminal_home(path, original, settings, limits, io)


if __name__ == "__main__":
    raise SystemExit(main_for(step_home))
