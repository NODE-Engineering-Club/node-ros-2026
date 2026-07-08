"""Small-distance geographic offset helpers (equirectangular approximation).

Valid for offsets on the order of tens of metres — the error from ignoring
Earth's ellipsoidal shape is negligible at that scale.
"""
import math

EARTH_RADIUS_M = 6378137.0  # WGS84 equatorial radius


def offset_latlon(lat_deg, lon_deg, north_m=0.0, east_m=0.0):
    """Return (lat, lon) shifted by north_m/east_m metres from (lat_deg, lon_deg)."""
    lat_rad = math.radians(lat_deg)
    dlat = north_m / EARTH_RADIUS_M
    dlon = east_m / (EARTH_RADIUS_M * math.cos(lat_rad))
    return lat_deg + math.degrees(dlat), lon_deg + math.degrees(dlon)
