"""Kucuk mesafe cografi yardimcilari (auv_dead_reckoning.geo ile ayni;
paketler arasi bagimliligi gevsek tutmak icin yerel kopya)."""
import math

EARTH_R = 6371000.0


def bearing_distance(lat1, lon1, lat2, lon2):
    lat1r, lat2r = math.radians(lat1), math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    x_east = dlon * math.cos((lat1r + lat2r) / 2.0) * EARTH_R
    y_north = dlat * EARTH_R
    dist = math.hypot(x_east, y_north)
    brg = math.degrees(math.atan2(x_east, y_north)) % 360.0
    return brg, dist
