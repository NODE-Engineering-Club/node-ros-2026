"""Root pytest configuration.

The ROS 2 packages in ``src/`` are laid out so that every ``core/`` subpackage is
importable with *no ROS installation present*.  That is deliberate: the sonar
parser, the simulator, the recorder and the diagnostics logic are all plain
Python and must be unit-testable on a laptop, in CI, and on the Jetson without
sourcing a ROS workspace.

This file puts each package root on ``sys.path`` so ``pytest`` works from the
repository root:

    pytest

Only the thin ``*_node.py`` wrappers import ``rclpy``; they are not exercised by
these tests.
"""

import sys
from pathlib import Path

#: The GUI packages, and only those. Putting every package in ``src/`` on the
#: path would also add the competition packages, whose modules are built and
#: sourced by colcon — importing them from source instead is a good way to get a
#: confusing shadowing bug in somebody else's subsystem.
_GUI_PACKAGES = (
    "asket_common",
    "asket_sim",
    "omniscan_bridge",
    "mission_recorder",
    "system_test",
    "gui_backend",
)

_SRC = Path(__file__).parent / "src"

for _name in _GUI_PACKAGES:
    _pkg = _SRC / _name
    if (_pkg / _name).is_dir():
        sys.path.insert(0, str(_pkg))
