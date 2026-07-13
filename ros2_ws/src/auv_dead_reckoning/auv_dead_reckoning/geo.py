"""Kucuk mesafeler icin ekvirektangular yaklastirmali coğrafi yardimcilar."""
import math

EARTH_R = 6371000.0  # m


def bearing_distance(lat1, lon1, lat2, lon2):
    """(lat1,lon1) -> (lat2,lon2): kerteriz (derece, 0=Kuzey) ve mesafe (m)."""
    lat1r, lat2r = math.radians(lat1), math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    x_east = dlon * math.cos((lat1r + lat2r) / 2.0) * EARTH_R
    y_north = dlat * EARTH_R
    dist = math.hypot(x_east, y_north)
    brg = math.degrees(math.atan2(x_east, y_north)) % 360.0
    return brg, dist


def offset_to_latlon(lat, lon, x_east_m, y_north_m):
    """Basit ters donusum: metre ofsetten enlem/boylam."""
    dlat = y_north_m / EARTH_R
    dlon = x_east_m / (EARTH_R * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)
