"""
Conservative planar geometry; no ROS, odometry, or hardware dependencies.

The swept set is a convex over-approximation, including interval padding. It
can reject a safe arc, but does not leave gaps between poses or omit corners.
Finite beams support only bounded angular gaps; infinity is never a clear ray.
"""

from dataclasses import dataclass
import math

import numpy as np

TAU = 2 * math.pi


@dataclass(frozen=True)
class Mount:
    """Scan origin in the footprint frame, metres and radians."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def apply(self, point):
        x, y = point
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + c * x - s * y, self.y + s * x + c * y

    def inverse(self, point):
        x, y = point[0] - self.x, point[1] - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * x + s * y, -s * x + c * y


def cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def hull(points):
    """Return a counterclockwise convex hull."""
    points = sorted(set(points))
    if len(points) < 3:
        raise ValueError('Degenerate footprint')
    lower, upper = [], []
    for dest, source in ((lower, points), (upper, reversed(points))):
        for p in source:
            while len(dest) > 1 and cross(dest[-2], dest[-1], p) <= 1e-12:
                dest.pop()
            dest.append(p)
    return tuple(lower[:-1] + upper[:-1])


def enclosing_polygon(points, n=32):
    """Bound hull complexity with outward support lines, preserving containment."""
    lines = []
    for i in range(n):
        nx, ny = math.cos(TAU * i / n), math.sin(TAU * i / n)
        lines.append((nx, ny, max(nx * x + ny * y for x, y in points)))
    result = []
    for a, b in zip(lines[-1:] + lines[:-1], lines):
        det = a[0] * b[1] - b[0] * a[1]
        result.append(((a[2] * b[1] - b[2] * a[1]) / det,
                       (a[0] * b[2] - b[0] * a[2]) / det))
    return tuple(result)


def inflate(poly, margin):
    """Offset convex half planes; mitered corners conservatively include disks."""
    if margin == 0:
        return poly
    lines = []
    for a, b in zip(poly, poly[1:] + poly[:1]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        nx, ny = dy / length, -dx / length
        lines.append((nx, ny, nx * a[0] + ny * a[1] + margin))
    result = []
    for a, b in zip(lines[-1:] + lines[:-1], lines):
        det = a[0] * b[1] - b[0] * a[1]
        result.append(((a[2] * b[1] - b[2] * a[1]) / det,
                       (a[0] * b[2] - b[0] * a[2]) / det))
    return tuple(result)


def signed_clearance(point, poly):
    """Euclidean distance outside; negative distance to nearest edge inside."""
    inside = True
    nearest = math.inf
    px, py = point
    for a, b in zip(poly, poly[1:] + poly[:1]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        t = max(0.0, min(1.0, ((px - a[0]) * dx + (py - a[1]) * dy) /
                         (dx * dx + dy * dy)))
        nearest = min(nearest, math.hypot(px - a[0] - t * dx, py - a[1] - t * dy))
        inside = inside and dx * (py - a[1]) - dy * (px - a[0]) >= -1e-10
    return -nearest if inside else nearest


def minimum_clearance(points, poly, cap=None):
    """Vectorize the point/edge narrow phase without changing its geometry."""
    if not points:
        return -math.inf
    p = np.asarray(points, dtype=float)
    if cap is not None:
        radius = max(math.hypot(*q) for q in poly) + cap
        p = p[np.sum(p * p, axis=1) <= radius * radius]
        if not len(p):
            return cap
    a = np.asarray(poly, dtype=float)
    edges = np.roll(a, -1, axis=0) - a
    delta = p[:, None, :] - a[None, :, :]
    projection = np.clip(np.sum(delta * edges, axis=2) / np.sum(edges * edges, axis=1), 0, 1)
    difference = delta - projection[:, :, None] * edges
    distances = np.sqrt(np.min(np.sum(difference * difference, axis=2), axis=1))
    inside = np.all(edges[:, 0] * delta[:, :, 1] - edges[:, 1] * delta[:, :, 0] >= -1e-10, axis=1)
    result = float(np.min(np.where(inside, -distances, distances)))
    return min(cap, result) if cap is not None else result


def path_clearance(points, footprint):
    """First contact translation along +x, not r*cos(angle) or central range."""
    nearest = math.inf
    for x, y in points:
        intersections = []
        for a, b in zip(footprint, footprint[1:] + footprint[:1]):
            if abs(b[1] - a[1]) < 1e-12:
                if abs(y - a[1]) < 1e-10:
                    intersections.extend((a[0], b[0]))
            elif min(a[1], b[1]) <= y <= max(a[1], b[1]):
                intersections.append(a[0] + (y - a[1]) * (b[0] - a[0]) / (b[1] - a[1]))
        if intersections and x >= min(intersections):
            nearest = min(nearest, max(0.0, x - max(intersections)))
    return nearest


def pose(v, w, t):
    if abs(w) < 1e-9:
        return v * t, 0.0, 0.0
    return v * math.sin(w * t) / w, v * (1 - math.cos(w * t)) / w, w * t


def swept_polygon(footprint, v, w, horizon, step, linear_scale=1.0, angular_scale=1.0,
                  linear_error=0.0, angular_error=0.0):
    """
    Cover every pose of the model with a Lipschitz interval displacement bound.

    A footprint point moves at <= |v| + radius*|w|. Each interval is covered
    by its midpoint footprint inflated by this speed times half the interval.
    Convexifying their union is conservative. Gain scales must bound actual
    motion for a physical interpretation; requested Twist alone is not feedback.
    """
    v, w = v * linear_scale, w * angular_scale
    n = max(1, math.ceil(horizon / step))
    dt = horizon / n
    radius = max(math.hypot(*p) for p in footprint)
    bound = (abs(v) + radius * abs(w)) * dt / 2
    points = list(footprint)
    for i in range(n):
        x, y, yaw = pose(v, w, (i + 0.5) * dt)
        points.extend(Mount(x, y, yaw).apply(p) for p in footprint)
    poly = hull(points)
    if len(poly) > 32:
        poly = hull(enclosing_polygon(poly))
    # Absolute model-error bounds in physical m/s and rad/s. Angular error also
    # changes the centre trajectory, not just the footprint's orientation.
    uncertainty = linear_error * horizon + angular_error * \
        (radius * horizon + abs(v) * horizon**2 / 2)
    return inflate(poly, bound + uncertainty + 1e-8)


def wedge_radius(poly, start, width):
    """Maximum radial extent of a polygon within a CCW angular wedge."""
    distances = [math.hypot(x, y) for x, y in poly
                 if (math.atan2(y, x) - start) % TAU <= width + 1e-9]
    # Norm is convex: clipped polygon vertices maximize it. Include intersections
    # with both wedge boundary rays, so no angular sampling holes remain.
    for angle in (start, start + width):
        ux, uy = math.cos(angle), math.sin(angle)
        for a, b in zip(poly, poly[1:] + poly[:1]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            det = ux * dy - uy * dx
            if abs(det) < 1e-12:
                continue
            radius = (a[0] * dy - a[1] * dx) / det
            t = (a[0] * uy - a[1] * ux) / det
            if radius >= 0 and -1e-9 <= t <= 1 + 1e-9:
                distances.append(radius)
    return max(distances, default=0.0)


@dataclass(frozen=True)
class MotionCheck:
    usable: bool
    clearance: float
    coverage: float
    reason: str


def check_polygon(scan, swept, max_gap):
    """
    Check observed endpoints and finite-supported visibility of the whole set.

    A pair of finite returns supports a wedge only up to the lesser range times
    cos(gap/2), and only for gaps <= max_gap. This explicit bounded interpolation
    model cannot establish the absence of objects between LiDAR rays. It never
    extends rays to range_max or treats empty/infinite sectors as clear.
    """
    # Scores saturate at 1 m; distant returns cannot collide or lower that score.
    # This broad phase avoids point/edge work for every far return at 25 Hz.
    clearance = minimum_clearance(scan.points, swept, cap=1.0)
    if clearance <= 0:
        return MotionCheck(False, clearance, 0.0, 'swept_footprint_collision')
    sensor_poly = tuple(scan.mount.inverse(p) for p in swept)
    beams = scan.beams
    if len(beams) < 3:
        return MotionCheck(False, clearance, 0.0, 'coverage_unknown')
    gaps = [(beams[(i + 1) % len(beams)][0] - a) % TAU for i, (a, _) in enumerate(beams)]
    max_radius = max(math.hypot(*p) for p in sensor_poly)
    if (max(gaps) <= max_gap + 1e-9 and
            min(r for _, r in beams) * math.cos(max(gaps) / 2) > max_radius):
        return MotionCheck(True, clearance, 1.0, 'clear')
    covered = required = 0.0
    for (a, r), (b, q) in zip(beams, beams[1:] + beams[:1]):
        gap = (b - a) % TAU
        if gap < 1e-10:
            continue
        extent = wedge_radius(sensor_poly, a, gap)
        if extent > 1e-8:
            required += gap
            if gap <= max_gap + 1e-9 and extent < min(r, q) * math.cos(gap / 2):
                covered += gap
    fraction = covered / required if required > 0 else 0.0
    return MotionCheck(fraction >= 1 - 1e-8, clearance, fraction,
                       'clear' if fraction >= 1 - 1e-8 else 'coverage_unknown')


def make_footprint(mode, vertices, radius, margin):
    if mode == 'legacy_envelope':
        # Circumscribed, never inscribed: includes the old configured 0.36 m disk.
        n = 24
        r = (radius + margin) / math.cos(math.pi / n)
        return tuple((r * math.cos(TAU * i / n), r * math.sin(TAU * i / n)) for i in range(n))
    if mode != 'polygon' or len(vertices) < 3:
        raise ValueError('polygon mode requires measured convex footprint_xy vertices')
    if not all(len(p) == 2 and all(math.isfinite(v) for v in p) for p in vertices):
        raise ValueError('Nonfinite or malformed footprint')
    original = tuple(tuple(p) for p in vertices)
    poly = hull(original)
    turns = [cross(original[i - 1], original[i], original[(i + 1) % len(original)])
             for i in range(len(original))]
    if len(poly) != len(original) or any(turn <= 0 for turn in turns):
        raise ValueError('footprint_xy must be strictly convex, CCW, without duplicate vertices')
    if signed_clearance((0, 0), poly) >= 0:
        raise ValueError('Footprint frame origin must be inside the footprint')
    return inflate(poly, margin)
