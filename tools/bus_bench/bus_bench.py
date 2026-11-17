#!/usr/bin/env python3
"""Benchmark how an end effector behaves over different communication settings (see README.md).

For every combination of bus URI x io_mode x update rate it starts the driver (EndEffectorHardware on rig.xacro, or
mock_components with --mock), samples /joint_states and reports the delivered rate, interval jitter, gaps and drift.
With --move it also drives one joint through a small step and reports how long the readback takes to follow.

Run it inside the container through bus_bench.sh. Without --move nothing commands the hand to move.
"""
import argparse
import csv
import os
import re
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

HERE = Path(__file__).resolve().parent
LAUNCH = HERE / "rig.launch.py"
READY = re.compile(r"on .*: ready")
STALE = ["[l]ib/controller_manager/ros2_control_node", "[l]ib/robot_state_publisher/robot_state_publisher"]
FAULT = re.compile(r"HW-00|Deactivating|\[ERROR\]")
MAX_STEP = 0.3  # rad: the largest step --move accepts
STALE_S = 0.5
# device -> the joint a step response moves (suffix) and its limits
DEVICES = {"xhand1": {"joint": "index_joint2", "lower": 0.0, "upper": 1.919}}


def percentile(values, p):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, max(0, round(p * len(ordered)) - 1))] if ordered else float("nan")


def seconds(value):
    return Duration(sec=int(value), nanosec=int((value % 1) * 1e9))


def stop_nodes():
    for pattern in STALE:
        subprocess.run(["pkill", "-9", "-f", pattern], check=False)


def running(pattern):
    return subprocess.run(["pgrep", "-f", pattern], capture_output=True).returncode == 0


def device_path(bus):
    match = re.match(r"^usb_[a-z0-9]+://([^?]*)", bus)
    return match.group(1) if match else None


def holders(path):
    """Processes that have the device open."""
    target, found = os.path.realpath(path), []
    for fd_dir in Path("/proc").glob("[0-9]*/fd"):
        try:
            if any(os.path.realpath(link) == target for link in fd_dir.iterdir()):
                found.append(fd_dir.parent.name)
        except OSError:
            continue
    return found


class Probe(Node):
    def __init__(self):
        super().__init__("bus_bench")
        self.rows, self.latest, self.stamp = [], {}, 0.0
        self.pub = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)
        self.create_subscription(JointState, "/joint_states", self.on_state,
                                 QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))

    def on_state(self, msg):
        now = time.monotonic()
        self.latest, self.stamp = dict(zip(msg.name, msg.position)), now
        self.rows.append((now, self.latest))

    def fresh(self):
        return bool(self.latest) and time.monotonic() - self.stamp < STALE_S

    def spin_for(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_fresh(self, timeout):
        end = time.monotonic() + timeout
        while not self.fresh() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        return self.fresh()

    def send(self, joint, waypoints):
        msg = JointTrajectory(joint_names=[joint])
        msg.points = [JointTrajectoryPoint(positions=[float(p)], time_from_start=seconds(t)) for t, p in waypoints]
        self.pub.publish(msg)


def controllers_yaml(rate, move_joint):
    text = ("controller_manager:\n  ros__parameters:\n"
            f"    update_rate: {rate}\n"
            "    joint_state_broadcaster:\n      type: joint_state_broadcaster/JointStateBroadcaster\n")
    if move_joint:
        text += ("    joint_trajectory_controller:\n      type: joint_trajectory_controller/JointTrajectoryController\n"
                 f"joint_trajectory_controller:\n  ros__parameters:\n    joints:\n      - {move_joint}\n"
                 "    command_interfaces:\n      - position\n    state_interfaces:\n      - position\n")
    return text


def sample(probe, duration, max_gap_ms):
    probe.rows.clear()
    probe.spin_for(duration)
    rows = list(probe.rows)
    if len(rows) < 3:
        return {"hz": 0.0, "p50": float("nan"), "p99": float("nan"), "max": float("nan"), "gaps": -1, "drift": float("nan")}
    dts = [(b[0] - a[0]) * 1000.0 for a, b in zip(rows, rows[1:])]
    span = rows[-1][0] - rows[0][0]
    drift = max(max(r[1][n] for r in rows) - min(r[1][n] for r in rows) for n in rows[0][1])
    return {"hz": (len(rows) - 1) / span, "p50": percentile(dts, 0.5), "p99": percentile(dts, 0.99), "max": max(dts),
            "gaps": sum(1 for d in dts if d > max_gap_ms), "drift": drift}


def step_response(probe, joint, step, hold, limits):
    """Command a near-instant step and time the readback. Returns (t10, t50, t90 in ms or None, final error)."""
    start = probe.latest[joint]
    target = start + step
    if not (limits["lower"] + 0.02 <= target <= limits["upper"] - 0.02):
        raise SystemExit(f"step target {target:.3f} is outside the joint limits")
    trace = []
    t0 = time.monotonic()
    probe.send(joint, [(0.0, start), (0.04, target), (hold, target), (hold + 1.5, start)])
    while (t := time.monotonic() - t0) < hold + 2.0:
        rclpy.spin_once(probe, timeout_sec=0.005)
        if not probe.fresh():
            print("  step response: joint state went stale, stopping", flush=True)
            break
        trace.append((t, probe.latest[joint]))
    crossings = []
    for fraction in (0.1, 0.5, 0.9):
        level = start + fraction * step
        hit = next((t for t, p in trace if (p >= level if step > 0 else p <= level)), None)
        crossings.append(None if hit is None else hit * 1000.0)
    return (*crossings, trace[-1][1] - start if trace else float("nan"))


def run_one(cfg, args, probe, out_dir):
    rate, mode, bus = cfg
    spec = DEVICES[args.device]
    joint = f"{args.side}_hand_{spec['joint']}"
    controllers = out_dir / f"controllers_{rate}.yaml"
    controllers.write_text(controllers_yaml(rate, joint if args.move else None))
    log_path = out_dir / f"launch_{re.sub(r'[^A-Za-z0-9]+', '_', bus)[-40:]}_{mode}_{rate}.log"
    env = dict(os.environ, BB_DEVICE=args.device, BB_SIDE=args.side, BB_BUS=bus, BB_IO_MODE=mode,
               BB_TORQUE=str(args.torque), BB_MOCK="true" if args.mock else "false", BB_CONTROLLERS=str(controllers),
               BB_MOVE="1" if args.move else "0")
    row = {"bus": "mock" if args.mock else bus, "mode": mode, "rate": rate}
    with open(log_path, "w") as log:
        proc = subprocess.Popen(["ros2", "launch", str(LAUNCH)], stdout=log, stderr=subprocess.STDOUT, env=env,
                                start_new_session=True)
    try:
        ready = READY if not args.mock else re.compile(r"Configured and activated joint_state_broadcaster")
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline and not ready.search(log_path.read_text(errors="replace")):
            if proc.poll() is not None:
                break
            time.sleep(0.5)
        if not ready.search(log_path.read_text(errors="replace")) or not probe.wait_fresh(10):
            row["error"] = "driver did not come up (see " + log_path.name + ")"
            return row
        probe.spin_for(2.0)
        row.update(sample(probe, args.duration, args.max_gap_ms))
        if args.move:
            row["t10"], row["t50"], row["t90"], row["step_err"] = step_response(
                probe, joint, args.step, args.hold, spec)
        row["faults"] = len(FAULT.findall(log_path.read_text(errors="replace")))
        return row
    finally:
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=6)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        stop_nodes()
        time.sleep(1.5)


def fmt(value, spec):
    return "-" if value is None or value != value else format(value, spec)


def print_table(rows, move):
    head = f"{'bus':36s} {'mode':5s} {'rate':>5s} {'Hz':>7s} {'p50ms':>7s} {'p99ms':>7s} {'maxms':>7s} {'gaps':>5s} {'drift':>7s} {'faults':>6s}"
    if move:
        head += f" {'t10ms':>7s} {'t50ms':>7s} {'t90ms':>7s} {'err':>7s}"
    print("\n" + head)
    for r in rows:
        if "error" in r:
            print(f"{r['bus'][-36:]:36s} {r['mode']:5s} {r['rate']:5d}  {r['error']}")
            continue
        line = (f"{r['bus'][-36:]:36s} {r['mode']:5s} {r['rate']:5d} {r['hz']:7.1f} {fmt(r['p50'], '7.2f'):>7s} "
                f"{fmt(r['p99'], '7.2f'):>7s} {fmt(r['max'], '7.2f'):>7s} {r['gaps']:5d} {r['drift']:7.4f} {r['faults']:6d}")
        if move:
            line += " " + " ".join(f"{fmt(r.get(k), '7.1f'):>7s}" for k in ("t10", "t50", "t90")) + f" {fmt(r.get('step_err'), '7.3f'):>7s}"
        print(line)


def parse():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--device", default="xhand1", choices=sorted(DEVICES))
    p.add_argument("--side", default="left", choices=["left", "right"])
    p.add_argument("--bus", action="append", default=[], help="bus URI, repeat to compare (e.g. usb_rs485:///dev/serial/by-id/...?baud=3000000)")
    p.add_argument("--mock", action="store_true", help="mock_components instead of hardware (no bus is opened)")
    p.add_argument("--rates", default="100", help="controller update rates in Hz, comma separated")
    p.add_argument("--io-modes", default="async", help="async, sync or both, comma separated")
    p.add_argument("--duration", type=float, default=15.0, help="seconds sampled per configuration")
    p.add_argument("--max-gap-ms", type=float, default=100.0, help="an interval longer than this counts as a gap")
    p.add_argument("--torque", type=float, default=0.8)
    p.add_argument("--move", action="store_true", help="also command a step and time the readback (the hand moves)")
    p.add_argument("--step", type=float, default=0.2, help="step size in rad for --move (max %.1f)" % MAX_STEP)
    p.add_argument("--hold", type=float, default=1.5, help="seconds the step is held")
    p.add_argument("--yes", action="store_true", help="do not ask for confirmation before --move")
    p.add_argument("--out", default="build/bus_bench", help="directory for logs and the result csv")
    args = p.parse_args()
    if not args.mock and not args.bus:
        p.error("give at least one --bus, or --mock")
    if args.mock:
        args.bus = ["mock"]
    if abs(args.step) > MAX_STEP:
        p.error(f"|--step| must be <= {MAX_STEP}")
    return args


def main():
    args = parse()
    for pattern in STALE:
        if running(pattern):
            sys.exit(f"a ros2_control_node / robot_state_publisher is already running ({pattern}); stop it first")
    if not args.mock:
        for bus in args.bus:
            path = device_path(bus)
            if not path or not os.path.exists(path):
                sys.exit(f"{bus}: no such device (only usb_rs485 / usb_serial buses are supported by the rig)")
            if holders(path):
                sys.exit(f"{path} is held by process {holders(path)}: stop it first")
    if args.move and not args.yes:
        print(f"--move: the {args.side} {args.device} will move {DEVICES[args.device]['joint']} by {args.step:+.2f} rad "
              f"(torque {args.torque}) once per configuration.\nHands clear of obstacles, power on, emergency stop in reach.")
        try:
            answer = input("Type YES to continue: ").strip()
        except EOFError:
            answer = ""
        if answer != "YES":
            sys.exit("not confirmed, nothing sent")

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    probe = Probe()
    configs = [(int(r), m, b) for b in args.bus for m in args.io_modes.split(",") for r in args.rates.split(",")]
    results = []
    try:
        for index, cfg in enumerate(configs, 1):
            print(f"[{index}/{len(configs)}] bus={cfg[2][-50:]} mode={cfg[1]} rate={cfg[0]} Hz ...", flush=True)
            results.append(run_one(cfg, args, probe, out_dir))
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        stop_nodes()
        probe.destroy_node()
        rclpy.shutdown()
    print_table(results, args.move)
    keys = sorted({k for r in results for k in r})
    with open(out_dir / "results.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(results)
    print(f"\nresults: {out_dir / 'results.csv'}")
    return 1 if any("error" in r for r in results) else 0


if __name__ == "__main__":
    sys.exit(main())
