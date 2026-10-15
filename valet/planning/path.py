"""Path container shared by planners and controllers."""
from dataclasses import dataclass, field
import math
import numpy as np


@dataclass
class Path:
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    direction: np.ndarray            # +1 forward, -1 reverse (per waypoint)
    info: dict = field(default_factory=dict)

    def __len__(self):
        return len(self.x)

    @property
    def length(self) -> float:
        return float(np.sum(np.hypot(np.diff(self.x), np.diff(self.y))))

    @property
    def n_gear_switches(self) -> int:
        return int(np.sum(np.diff(self.direction) != 0))

    def reverse_length(self) -> float:
        d = np.hypot(np.diff(self.x), np.diff(self.y))
        return float(np.sum(d[self.direction[1:] < 0]))

    def segments(self):
        """Split into constant-direction pieces (cusps are shared points)."""
        cuts = list(np.nonzero(np.diff(self.direction) != 0)[0] + 1)
        bounds = [0] + cuts + [len(self.x)]
        segs = []
        for a, b in zip(bounds[:-1], bounds[1:]):
            lo = max(a - 1, 0)          # include the cusp point at the start
            segs.append(Path(self.x[lo:b].copy(), self.y[lo:b].copy(), self.yaw[lo:b].copy(),
                             np.full(b - lo, self.direction[a]), {}))
        return segs

    def curvature(self) -> np.ndarray:
        """Discrete curvature |dyaw/ds| between waypoints."""
        ds = np.hypot(np.diff(self.x), np.diff(self.y))
        dyaw = np.abs((np.diff(self.yaw) + np.pi) % (2 * np.pi) - np.pi)
        with np.errstate(divide="ignore", invalid="ignore"):
            k = np.where(ds > 1e-6, dyaw / ds, 0.0)
        return k


def from_points(points, direction=1, spacing=0.25) -> Path:
    """Resample a polyline at fixed spacing and compute tangent headings."""
    pts = np.asarray(points, float)
    d = np.hypot(*np.diff(pts, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(d)])
    n = max(int(math.ceil(s[-1] / spacing)), 1)
    si = np.linspace(0.0, s[-1], n + 1)
    x, y = np.interp(si, s, pts[:, 0]), np.interp(si, s, pts[:, 1])
    yaw = np.arctan2(np.gradient(y), np.gradient(x))
    if direction < 0:
        yaw = (yaw + np.pi + np.pi) % (2 * np.pi) - np.pi
    return Path(x, y, yaw, np.full(len(x), direction))
