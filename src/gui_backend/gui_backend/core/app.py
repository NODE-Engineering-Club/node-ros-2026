"""Run the backend without ROS.

    python3 -m gui_backend.core.app --sim

This is the development entry point: a full GUI against the simulator, on a
laptop, with no ROS installed and no hardware present. The ROS entry point is
``gui_backend/gui_backend_node.py``, which runs this same app with a
``RosSource`` instead.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import uvicorn

from asket_sim.core.world import SimWorld, WorldConfig

from .hub import Hub
from .server import create_app
from .sim_source import SimSource

DEFAULT_STATIC = Path(__file__).resolve().parents[1] / "static"


def build_app(
    time_scale: float = 1.0,
    tiles_path: str | None = None,
    static_dir: str | None = None,
    heading_source: str = "magnetometer",
    seed: int = 1,
):
    cfg = WorldConfig(seed=seed)
    cfg.vessel.heading_source = heading_source
    source = SimSource(SimWorld(cfg), time_scale=time_scale)
    hub = Hub(source)
    return create_app(
        hub,
        static_dir=static_dir or DEFAULT_STATIC,
        tiles_path=tiles_path,
    )


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sim", action="store_true", help="use simulated sources (the default here)")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--time-scale", type=float, default=1.0,
                    help=">1 runs the simulated survey faster than real time")
    ap.add_argument("--tiles", default=None, help="path to an .mbtiles file")
    ap.add_argument("--static", default=None, help="path to the built frontend")
    ap.add_argument("--heading-source", default="magnetometer",
                    choices=["magnetometer", "gnss_compass"])
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args(argv)

    app = build_app(
        time_scale=args.time_scale,
        tiles_path=args.tiles,
        static_dir=args.static,
        heading_source=args.heading_source,
        seed=args.seed,
    )
    print(f"Asket mission GUI (simulated) on http://{args.host}:{args.port}")
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
