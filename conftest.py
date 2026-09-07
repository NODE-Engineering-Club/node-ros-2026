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

_SRC = Path(__file__).parent / "src"

for _pkg in sorted(p for p in _SRC.iterdir() if p.is_dir()):
    if (_pkg / _pkg.name).is_dir():
        sys.path.insert(0, str(_pkg))
