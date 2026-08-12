import math as m


def cart2sph(x, y, z):
    """Convert Cartesian coordinates to spherical (distance, elevation, azimuth)."""
    xy_sq = x**2 + y**2
    r = m.sqrt(xy_sq + z**2)
    elev = m.atan2(z, m.sqrt(xy_sq))
    az = m.atan2(y, x)
    return r, elev, az
