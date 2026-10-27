"""The world: occupancy map (ROS map_server format), user edits, LiDAR raycasting
and car footprint collision, all built on a Euclidean distance transform (EDT)."""
import io
import math
import os

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt


class World:
    def __init__(self, yaml_path: str):
        self.yaml_path = yaml_path
        self.name = os.path.splitext(os.path.basename(yaml_path))[0]
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        self.meta = meta
        self.resolution = float(meta['resolution'])
        origin = meta.get('origin', [0.0, 0.0, 0.0])
        self.origin = (float(origin[0]), float(origin[1]))
        img_path = meta['image']
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(yaml_path), img_path)
        img = np.asarray(Image.open(img_path).convert('L'), dtype=np.float32)
        if int(meta.get('negate', 0)):
            img = 255.0 - img
        # Row 0 of the image is the TOP of the map; flip so row 0 = min y.
        img = np.flipud(img)
        # Same binarisation as f1tenth_gym: <= 128 is an obstacle, unknown (205) counts as free.
        self.base_occ = img <= 128.0
        self.height, self.width = self.base_occ.shape
        self.edits = np.zeros_like(self.base_occ, dtype=np.int8)  # +1 wall, -1 erased
        spawn = meta.get('websim_spawn')
        self.spawn = tuple(float(v) for v in spawn) if spawn else None
        self.description = meta.get('websim_description', '')
        self.version = 0
        self._rebuild()

    # ------------------------------------------------------------------ map
    @property
    def occ(self):
        occ = self.base_occ.copy()
        occ[self.edits == 1] = True
        occ[self.edits == -1] = False
        return occ

    def _rebuild(self):
        occ = self.occ
        # distance (m) from every cell to the nearest obstacle cell; 0 on obstacles
        self.edt = (distance_transform_edt(~occ) * self.resolution).astype(np.float32)
        self.version += 1

    def world_to_cell(self, x, y):
        return ((np.asarray(x) - self.origin[0]) / self.resolution,
                (np.asarray(y) - self.origin[1]) / self.resolution)

    def edt_at(self, x, y):
        """EDT lookup for arrays of world points; outside the map counts as free/far."""
        cx, cy = self.world_to_cell(x, y)
        ix = np.floor(cx).astype(np.int64)
        iy = np.floor(cy).astype(np.int64)
        inside = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
        out = np.full(ix.shape, np.inf, dtype=np.float32)
        out[inside] = self.edt[iy[inside], ix[inside]]
        return out, inside

    def paint(self, points, radius, mode):
        """Paint walls (mode='wall') or erase them (mode='erase') along a stroke of world points."""
        r = max(radius / self.resolution, 0.5)
        val = 1 if mode == 'wall' else -1
        yy, xx = np.ogrid[0:self.height, 0:self.width]
        pts = list(points)
        # Densify the stroke so fast mouse moves leave no gaps
        dense = []
        for i, (x, y) in enumerate(pts):
            if i == 0:
                dense.append((x, y))
                continue
            px, py = pts[i - 1]
            d = math.hypot(x - px, y - py)
            n = max(1, int(d / (radius * 0.5 + 1e-6)))
            for k in range(1, n + 1):
                dense.append((px + (x - px) * k / n, py + (y - py) * k / n))
        for (x, y) in dense:
            cx, cy = self.world_to_cell(x, y)
            x0, x1 = int(max(cx - r - 1, 0)), int(min(cx + r + 2, self.width))
            y0, y1 = int(max(cy - r - 1, 0)), int(min(cy + r + 2, self.height))
            if x0 >= x1 or y0 >= y1:
                continue
            sub = (xx[:, x0:x1] + 0.5 - cx) ** 2 + (yy[y0:y1, :] + 0.5 - cy) ** 2 <= r * r
            self.edits[y0:y1, x0:x1][sub] = val
        self._rebuild()

    def clear_edits(self):
        self.edits[:] = 0
        self._rebuild()

    def grid_png(self) -> bytes:
        """Current effective occupancy as PNG (black = obstacle, white = free), image-row 0 = top."""
        occ = np.flipud(self.occ)
        img = np.where(occ, 0, 255).astype(np.uint8)
        # mark user-painted cells with distinct grey levels so the UI can tint them
        edits = np.flipud(self.edits)
        img[edits == 1] = 60
        img[edits == -1] = 230
        buf = io.BytesIO()
        Image.fromarray(img, mode='L').save(buf, format='PNG', optimize=False)
        return buf.getvalue()

    def occupancy_grid_data(self):
        """Values for nav_msgs/OccupancyGrid (row-major, row 0 = min y)."""
        return np.where(self.occ, 100, 0).astype(np.int8).ravel().tolist()

    def free_point_near(self, x, y):
        d, inside = self.edt_at(np.array([x]), np.array([y]))
        return bool(inside[0]) and float(d[0]) > 0.3

    def default_spawn(self):
        if self.spawn:
            return self.spawn
        # Most open free cell as fallback
        iy, ix = np.unravel_index(np.argmax(self.edt), self.edt.shape)
        return (self.origin[0] + (ix + 0.5) * self.resolution,
                self.origin[1] + (iy + 0.5) * self.resolution, 0.0)

    # ---------------------------------------------------------------- LiDAR
    def raycast(self, ox, oy, yaw, angles, max_range, max_iters=256):
        """Sphere-tracing on the EDT. Returns one range per angle (same approach as f1tenth_gym)."""
        th = yaw + angles
        dx, dy = np.cos(th), np.sin(th)
        t = np.zeros_like(angles, dtype=np.float64)
        ranges = np.full(angles.shape, max_range, dtype=np.float64)
        active = np.ones(angles.shape, dtype=bool)
        eps = self.resolution * 0.5
        for _ in range(max_iters):
            idx = np.nonzero(active)[0]
            if idx.size == 0:
                break
            px = ox + dx[idx] * t[idx]
            py = oy + dy[idx] * t[idx]
            d, inside = self.edt_at(px, py)
            hit = inside & (d <= eps)
            ranges[idx[hit]] = t[idx[hit]]
            out = ~inside
            done = hit | out
            active[idx[done]] = False
            go = ~done
            # conservative step: EDT is measured between cell centres, so back off one
            # cell to never jump over a 1-pixel wall
            t[idx[go]] += np.maximum(d[go] - self.resolution, 0.35 * self.resolution)
            too_far = t > max_range
            active &= ~too_far
        return np.minimum(ranges, max_range)

    # ------------------------------------------------------------ collision
    def footprint_collides(self, x, y, yaw, length, width):
        hl, hw = length / 2.0, width / 2.0
        n = 6
        xs, ys = [], []
        for i in range(n + 1):
            a = -hl + 2 * hl * i / n
            for b in (-hw, hw):
                xs.append(a); ys.append(b)
        for i in range(1, 3):
            b = -hw + 2 * hw * i / 3
            for a in (-hl, hl):
                xs.append(a); ys.append(b)
        xs, ys = np.array(xs), np.array(ys)
        c, s = math.cos(yaw), math.sin(yaw)
        wx = x + c * xs - s * ys
        wy = y + s * xs + c * ys
        d, inside = self.edt_at(wx, wy)
        return bool(np.any(inside & (d <= self.resolution * 0.5)))
