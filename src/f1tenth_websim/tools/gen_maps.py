"""Generate the built-in websim maps (ROS map_server format: PNG + YAML).
Usage: python3 tools/gen_maps.py maps/"""
import math
import os
import sys

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt

RES = 0.05


def save(outdir, name, free, origin, spawn, description):
    """free: bool array with row 0 = min y."""
    img = np.where(free, 254, 0).astype(np.uint8)
    img = np.flipud(img)
    Image.fromarray(img, mode='L').save(os.path.join(outdir, name + '.png'))
    meta = {
        'image': name + '.png',
        'resolution': RES,
        'origin': [float(origin[0]), float(origin[1]), 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
        'websim_spawn': [round(float(v), 3) for v in spawn],
        'websim_description': description,
    }
    with open(os.path.join(outdir, name + '.yaml'), 'w') as f:
        yaml.safe_dump(meta, f, sort_keys=False, allow_unicode=True)
    print('wrote', name, img.shape)


def grid(w_m, h_m):
    return np.zeros((int(round(h_m / RES)), int(round(w_m / RES))), dtype=bool)


def track_from_centerline(pts, half_width, margin=2.0):
    pts = np.asarray(pts)
    minx, miny = pts.min(0) - half_width - margin
    maxx, maxy = pts.max(0) + half_width + margin
    g = grid(maxx - minx, maxy - miny)
    h, w = g.shape
    line = np.ones_like(g)  # True = not centerline
    # rasterise the centreline densely
    dense = []
    for i in range(len(pts)):
        a, b = pts[i], pts[(i + 1) % len(pts)]
        n = max(1, int(np.hypot(*(b - a)) / (RES * 0.3)))
        for k in range(n):
            dense.append(a + (b - a) * k / n)
    dense = np.array(dense)
    ix = ((dense[:, 0] - minx) / RES).astype(int).clip(0, w - 1)
    iy = ((dense[:, 1] - miny) / RES).astype(int).clip(0, h - 1)
    line[iy, ix] = False
    dist = distance_transform_edt(line) * RES
    return dist <= half_width, (minx, miny)


def catmull_rom_closed(ctrl, n_per_seg=60):
    ctrl = np.asarray(ctrl, dtype=float)
    out = []
    N = len(ctrl)
    for i in range(N):
        p0, p1, p2, p3 = ctrl[i - 1], ctrl[i], ctrl[(i + 1) % N], ctrl[(i + 2) % N]
        for t in np.linspace(0, 1, n_per_seg, endpoint=False):
            t2, t3 = t * t, t * t * t
            out.append(0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
                              + (-p0 + 3 * p1 - 3 * p2 + p3) * t3))
    return np.array(out)


def main(outdir):
    os.makedirs(outdir, exist_ok=True)

    # 1) Straight corridor closed at the end: the AEB (lab 2) test bench
    L, W = 26.0, 2.4
    g = grid(L + 1.0, W + 1.0)
    x0, y0 = int(0.5 / RES), int(0.5 / RES)
    g[y0:y0 + int(W / RES), x0:x0 + int(L / RES)] = True
    save(outdir, 'pasillo_aeb', g, (-0.5, -W / 2 - 0.5), (1.5, 0.0, 0.0),
         'Pasillo recto de 26 m cerrado al fondo. Para probar el frenado de emergencia (lab 2).')

    # 2) Oval
    a, b = 9.0, 5.0
    ctrl = [(a * math.cos(t), b * math.sin(t)) for t in np.linspace(0, 2 * math.pi, 200, endpoint=False)]
    free, origin = track_from_centerline(ctrl, 1.1)
    save(outdir, 'ovalo', free, origin, (0.0, -b, 0.0),
         'Óvalo de 2,2 m de ancho. Bueno para seguimiento de pared y primeras vueltas.')

    # 3) Twisty circuit
    ctrl = [(0, 0), (10, -1), (16, 2), (17, 8), (12, 10), (8, 7), (4, 10), (-2, 11),
            (-6, 7), (-3, 4), (-8, 1), (-6, -3)]
    pts = catmull_rom_closed(ctrl)
    free, origin = track_from_centerline(pts, 1.15)
    d = pts[1] - pts[0]
    save(outdir, 'circuito', free, origin, (float(pts[0][0]), float(pts[0][1]), math.atan2(d[1], d[0])),
         'Circuito revirado con curvas de distinto radio. Para follow the gap y pure pursuit.')

    # 4) Building loop with corridors, a column and doorway niches (Levine-style)
    g = grid(30.0, 16.0)
    cw = 2.0  # corridor width
    def rect(x, y, w, h, val=True):
        g[int(y / RES):int((y + h) / RES), int(x / RES):int((x + w) / RES)] = val
    rect(1.0, 1.0, 28.0, cw)           # bottom corridor
    rect(1.0, 13.0, 28.0, cw)          # top corridor
    rect(1.0, 1.0, cw, 14.0)           # left corridor
    rect(27.0, 1.0, cw, 14.0)          # right corridor
    rect(13.0, 3.0, 2.5, 1.0)          # niche (door recess) off bottom corridor
    rect(8.0, 12.2, 1.2, 0.8)          # niche off top corridor
    rect(20.0, 12.0, 1.6, 1.0)
    rect(25.8, 7.0, 1.2, 1.6)          # niche off right corridor
    rect(20.0, 1.6, 0.4, 0.4, False)   # column in bottom corridor
    save(outdir, 'edificio', g, (0.0, 0.0), (5.0, 2.0, 0.0),
         'Bucle de pasillos tipo edificio, con huecos de puertas y una columna.')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'maps')
