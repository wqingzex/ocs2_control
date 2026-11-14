"""Units: the processes a run consists of, one process group and one log each.

A unit is a command plus what the supervisor needs to start it in order (dependencies), watch it (topics) and
decide when it is up (ready_log). Units whose lifecycle is not a plain command (a daemon that forks and exits)
bring a Driver. The units themselves are built by the packages that own the devices."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Protocol, Tuple


LOG_MODE_ENV = "ROBOT_LOG_MODE"  # read by robot_log::debugMode() in every component


class Driver(Protocol):
    """Lifecycle of a unit that is not one plain process: a daemon that forks and exits, a service we only share."""

    def running(self) -> bool: ...
    def pids(self) -> List[int]: ...
    def start(self, log_path: str) -> List[int]:
        """Start it if needed; return the PIDs this call started ([] if it was running already)."""
    def stop(self, pids: List[int]) -> None:
        """Stop the given PIDs (only what this supervisor started)."""


@dataclass(frozen=True)
class Unit:
    name: str
    title: str                       # shown in status / console
    cmd: Tuple[str, ...]             # empty when a driver runs the unit
    depends: Tuple[str, ...] = ()
    group: str = ""                  # what kind of unit this is (e.g. "camera"); the console treats groups alike
    topics: Tuple[str, ...] = ()     # health topics (message rate)
    driver: Optional[Driver] = None  # set for units that are not one plain process
    env: Tuple[Tuple[str, str], ...] = ()  # extra environment variables for the process
    unset_env: Tuple[str, ...] = ()  # inherited variables the process must not see (e.g. proxy settings)
    on_demand: bool = False          # skipped by "start everything"; started only when named
    ready_log: str = ""              # start() returns once the log has this text (or the unit exited, or the timeout passed)
    ready_timeout_s: float = 0.0


@dataclass
class UnitSet:
    units: Dict[str, Unit] = field(default_factory=dict)

    def names(self) -> List[str]:
        return list(self.units)

    def __contains__(self, name: str) -> bool:
        return name in self.units

    def __getitem__(self, name: str) -> Unit:
        if name not in self.units:
            raise KeyError(f"未知模块 '{name}'，可选: {', '.join(self.units)}")
        return self.units[name]

    def start_order(self, names: Optional[List[str]] = None) -> List[str]:
        """The given units (default: all but on-demand ones) plus their dependencies, dependencies first."""
        order: List[str] = []

        def visit(name: str, stack: Tuple[str, ...]):
            if name in order:
                return
            if name in stack:
                raise ValueError(f"模块依赖成环: {' -> '.join(stack + (name,))}")
            for dep in self[name].depends:
                visit(dep, stack + (name,))
            order.append(name)

        for name in names or [n for n, u in self.units.items() if not u.on_demand]:
            visit(name, ())
        return order

    def launch_order(self, names: Optional[List[str]] = None) -> List[str]:
        """What to start for names: like start_order, but on-demand units never pull in their dependencies."""
        requested = names or [n for n, u in self.units.items() if not u.on_demand]
        regular = [n for n in requested if not self[n].on_demand]
        order = self.start_order(regular) if regular else []
        return order + [n for n in requested if self[n].on_demand and n not in order]

    def stop_order(self, names: Optional[List[str]] = None) -> List[str]:
        """The given units (default: all) plus everything that depends on them, dependents first."""
        wanted = set(names or self.names())
        changed = True
        while changed:
            changed = False
            for unit in self.units.values():
                if unit.name not in wanted and wanted.intersection(unit.depends):
                    wanted.add(unit.name)
                    changed = True
        return [n for n in reversed(self.start_order(self.names())) if n in wanted]
