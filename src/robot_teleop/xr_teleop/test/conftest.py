"""Fixtures to run units through a real Supervisor."""
import time

import pytest

from robot_runtime.supervisor import Supervisor, UnitError
from robot_runtime.units import UnitSet


@pytest.fixture
def make_supervisor(tmp_path):
    created = []

    def factory(units):
        sup = Supervisor(UnitSet(units), str(tmp_path / "run"), str(tmp_path / "logs"))
        created.append(sup)
        return sup

    yield factory
    for sup in created:  # never leave processes behind
        for name in sup.units.names():
            try:
                sup.stop(name, timeout_s=1.0)
            except UnitError:
                pass


def wait_for(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False
