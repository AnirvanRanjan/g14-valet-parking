"""Small 2-D geometry toolkit: oriented boxes, ray casting, collision tests."""
from dataclasses import dataclass
import math
import numpy as np


def wrap_angle(a):
    """Wrap an angle (scalar or array) to (-pi, pi]."""
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi if isinstance(a, np.ndarray) \
        else (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class OBB:
    """Oriented bounding box given by centre, heading and full length/width."""
    cx: float
    cy: float
    yaw: float
    length: float
    width: float
    kind: str = "static"      # "wall", "car", "static"

    def corners(self) -> np.ndarray:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        hl, hw = self.length / 2, self.width / 2
        local = np.array([[hl, hw], [-hl, hw], [-hl, -hw], [hl, -hw]])
        rot = np.array([[c, -s], [s, c]])
        return local @ rot.T + np.array([self.cx, self.cy])


def car_footprint(x, y, yaw, vp, margin=0.0) -> OBB:
    """Body rectangle of the car whose rear-axle centre is at (x, y)."""
    off = vp.centre_offset
    return OBB(x + off * math.cos(yaw), y + off * math.sin(yaw), yaw,
               vp.length + 2 * margin, vp.width + 2 * margin, "ego")


def _axes(box: OBB):
    c, s = math.cos(box.yaw), math.sin(box.yaw)
    return np.array([[c, s], [-s, c]])


def obb_overlap(a: OBB, b: OBB) -> bool:
    """Separating-axis test for two oriented rectangles."""
    ca, cb = a.corners(), b.corners()
    for ax in np.vstack([_axes(a), _axes(b)]):
        pa, pb = ca @ ax, cb @ ax
        if pa.max() < pb.min() or pb.max() < pa.min():
            return False
    return True


def circle_obb_distance(px, py, box: OBB) -> float:
    """Distance from a point to an OBB (0 if the point is inside)."""
    c, s = math.cos(box.yaw), math.sin(box.yaw)
    dx, dy = px - box.cx, py - box.cy
    lx, ly = c * dx + s * dy, -s * dx + c * dy
    qx = max(abs(lx) - box.length / 2, 0.0)
    qy = max(abs(ly) - box.width / 2, 0.0)
    return math.hypot(qx, qy)


def obb_min_distance(a: OBB, b: OBB, samples: int = 9) -> float:
    """Approximate minimum gap between two boxes (0 when they overlap)."""
    if obb_overlap(a, b):
        return 0.0
    best = math.inf
    for src, dst in ((a, b), (b, a)):
        cs = src.corners()
        for i in range(4):
            p, q = cs[i], cs[(i + 1) % 4]
            for t in np.linspace(0.0, 1.0, samples):
                pt = p + t * (q - p)
                best = min(best, circle_obb_distance(pt[0], pt[1], dst))
    return best


def ray_cast_boxes(ox, oy, dirs, boxes, max_range):
    """Cast rays from (ox[i], oy[i]) along unit vectors dirs[i] against boxes.

    Vectorised slab method in each box frame. Returns hit distances (n_rays,)
    clipped to max_range.
    """
    ox, oy = np.asarray(ox, float), np.asarray(oy, float)
    dirs = np.asarray(dirs, float)
    best = np.full(len(ox), max_range)
    if not boxes:
        return best
    cx = np.array([b.cx for b in boxes])[:, None]
    cy = np.array([b.cy for b in boxes])[:, None]
    yaw = np.array([b.yaw for b in boxes])[:, None]
    hl = np.array([b.length / 2 for b in boxes])[:, None]
    hw = np.array([b.width / 2 for b in boxes])[:, None]
    c, s = np.cos(yaw), np.sin(yaw)
    dx, dy = ox[None, :] - cx, oy[None, :] - cy
    lx, ly = c * dx + s * dy, -s * dx + c * dy                    # ray origin, box frame
    ux = c * dirs[None, :, 0] + s * dirs[None, :, 1]
    uy = -s * dirs[None, :, 0] + c * dirs[None, :, 1]
    with np.errstate(divide="ignore", invalid="ignore"):
        tx1, tx2 = (-hl - lx) / ux, (hl - lx) / ux
        ty1, ty2 = (-hw - ly) / uy, (hw - ly) / uy
    tx1 = np.where(np.isnan(tx1), -np.inf, tx1); tx2 = np.where(np.isnan(tx2), np.inf, tx2)
    ty1 = np.where(np.isnan(ty1), -np.inf, ty1); ty2 = np.where(np.isnan(ty2), np.inf, ty2)
    tmin = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
    tmax = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
    hit = (tmax >= np.maximum(tmin, 0.0))
    t = np.where(tmin >= 0.0, tmin, 0.0)                         # origin inside -> 0
    t = np.where(hit, t, np.inf)
    return np.minimum(best, t.min(axis=0))


def ray_cast_circles(ox, oy, dirs, centres, radius, max_range):
    """Ray casting against circles (pedestrians)."""
    best = np.full(len(ox), max_range)
    for (px, py) in centres:
        fx, fy = np.asarray(ox) - px, np.asarray(oy) - py
        b = fx * dirs[:, 0] + fy * dirs[:, 1]
        cc = fx ** 2 + fy ** 2 - radius ** 2
        disc = b ** 2 - cc
        ok = disc >= 0
        t = -b - np.sqrt(np.where(ok, disc, 0.0))
        t = np.where(ok & (t >= 0), t, np.inf)
        best = np.minimum(best, t)
    return best
