"""Offline map tiles.

**There is no internet in the field.** A map that silently falls back to a blank
grey rectangle because it cannot reach a tile server is not a degraded map, it
is a useless one — and the operator will not know why.

So tiles are served from a pre-downloaded MBTiles file on the Jetson, read with
the standard library's ``sqlite3`` (MBTiles is just SQLite, and pulling in a
tile library for this would be an arm64 build dependency for nothing). If the
file is missing or does not cover the survey area, the backend says so
explicitly and the frontend draws a coordinate graticule instead of pretending.

MBTiles stores rows in TMS order, with y counted from the south. Web map clients
count from the north. Getting that flip wrong produces a map that looks almost
right, which is worse than one that looks obviously wrong.
"""

from __future__ import annotations

import sqlite3
import threading
from dataclasses import dataclass
from pathlib import Path


@dataclass
class TileSetInfo:
    available: bool
    path: str = ""
    name: str = ""
    format: str = "png"
    min_zoom: int = 0
    max_zoom: int = 0
    #: [west, south, east, north] in degrees, if the file declares it.
    bounds: list[float] | None = None
    tile_count: int = 0
    message: str = ""

    def to_dict(self) -> dict:
        return {
            "available": self.available,
            "name": self.name,
            "format": self.format,
            "min_zoom": self.min_zoom,
            "max_zoom": self.max_zoom,
            "bounds": self.bounds,
            "tile_count": self.tile_count,
            "message": self.message,
        }

    def covers(self, lat: float, lon: float) -> bool:
        if not self.available or not self.bounds:
            return self.available
        west, south, east, north = self.bounds
        return west <= lon <= east and south <= lat <= north


class MBTiles:
    """Read-only MBTiles reader. Thread-safe by way of one connection per thread."""

    def __init__(self, path: str | Path | None) -> None:
        self.path = Path(path) if path else None
        self._local = threading.local()
        self.info = self._read_info()

    # -- connection -------------------------------------------------------

    def _connection(self) -> sqlite3.Connection | None:
        if not self.path or not self.path.is_file():
            return None
        conn = getattr(self._local, "conn", None)
        if conn is None:
            conn = sqlite3.connect(
                f"file:{self.path}?mode=ro", uri=True, check_same_thread=False
            )
            self._local.conn = conn
        return conn

    def _read_info(self) -> TileSetInfo:
        if not self.path:
            return TileSetInfo(
                False,
                message=(
                    "No offline tile file configured. The map will show a "
                    "coordinate grid only. Set tiles_path to an .mbtiles file "
                    "covering the survey area."
                ),
            )
        if not self.path.is_file():
            return TileSetInfo(
                False,
                path=str(self.path),
                message=(
                    f"Offline tile file {self.path} not found. The map will show "
                    "a coordinate grid only."
                ),
            )
        try:
            conn = self._connection()
            meta = {
                k: v for k, v in conn.execute("SELECT name, value FROM metadata")
            }
            zooms = conn.execute(
                "SELECT MIN(zoom_level), MAX(zoom_level), COUNT(*) FROM tiles"
            ).fetchone()
        except sqlite3.Error as exc:
            return TileSetInfo(
                False, path=str(self.path), message=f"Tile file unreadable: {exc}"
            )

        bounds = None
        if "bounds" in meta:
            try:
                bounds = [float(x) for x in meta["bounds"].split(",")]
            except ValueError:
                bounds = None

        min_zoom = int(meta.get("minzoom", zooms[0] or 0))
        max_zoom = int(meta.get("maxzoom", zooms[1] or 0))
        return TileSetInfo(
            available=bool(zooms[2]),
            path=str(self.path),
            name=meta.get("name", self.path.stem),
            format=meta.get("format", "png"),
            min_zoom=min_zoom,
            max_zoom=max_zoom,
            bounds=bounds,
            tile_count=int(zooms[2] or 0),
            message=(
                f"{zooms[2]} tiles, zoom {min_zoom}-{max_zoom}"
                if zooms[2]
                else "Tile file contains no tiles."
            ),
        )

    # -- tiles ------------------------------------------------------------

    def tile(self, z: int, x: int, y: int) -> bytes | None:
        """One tile, addressed in **XYZ** (y from the north) as clients use.

        The TMS flip happens here, once, rather than in the client where it
        would be easy to get subtly wrong.
        """
        conn = self._connection()
        if conn is None:
            return None
        tms_y = (1 << z) - 1 - y
        try:
            row = conn.execute(
                "SELECT tile_data FROM tiles "
                "WHERE zoom_level=? AND tile_column=? AND tile_row=?",
                (z, x, tms_y),
            ).fetchone()
        except sqlite3.Error:
            return None
        return bytes(row[0]) if row else None

    @property
    def content_type(self) -> str:
        return {
            "png": "image/png",
            "jpg": "image/jpeg",
            "jpeg": "image/jpeg",
            "webp": "image/webp",
            "pbf": "application/x-protobuf",
        }.get(self.info.format.lower(), "application/octet-stream")
