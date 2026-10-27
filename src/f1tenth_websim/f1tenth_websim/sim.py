"""Simulation core: physics stepping, command multiplexing, collisions, laps, events.
Knows nothing about ROS or the web; both talk to it through plain methods."""
import glob
import math
import os
import threading
import time
from collections import deque

import numpy as np

from .dynamics import VehicleParams, VehicleState, step as dyn_step
from .world import World

MODES = ('ros', 'kbd', 'mix')
MODE_NAMES = {'ros': 'ROS', 'kbd': 'Teclado', 'mix': 'Teclado + ROS'}


class Command:
    __slots__ = ('speed', 'steer', 'stamp')

    def __init__(self, speed=0.0, steer=0.0, stamp=0.0):
        self.speed, self.steer, self.stamp = speed, steer, stamp


class Simulator:
    def __init__(self, maps_dirs, map_name=None, params=None, cfg=None):
        cfg = cfg or {}
        self.p = params or VehicleParams()
        self.dt = float(cfg.get('timestep', 0.01))
        self.scan_beams = int(cfg.get('scan_beams', 1080))
        self.scan_fov = float(cfg.get('scan_fov', 4.7))
        self.scan_range = float(cfg.get('scan_max_range', 30.0))
        self.scan_noise = float(cfg.get('scan_noise_std', 0.01))
        self.scan_rate = float(cfg.get('scan_rate', 40.0))
        self.odom_rate = float(cfg.get('odom_rate', 100.0))
        self.scan_offset = float(cfg.get('scan_distance_to_base_link', 0.0))
        self.mix_priority_window = float(cfg.get('mix_priority_window', 0.25))
        self.kbd_timeout = 0.5
        self.angle_min = -self.scan_fov / 2.0
        self.angle_inc = self.scan_fov / self.scan_beams  # same convention as gym_bridge
        self.angles = self.angle_min + np.arange(self.scan_beams) * self.angle_inc

        self.lock = threading.RLock()
        self.maps_dirs = [d for d in maps_dirs if d and os.path.isdir(d)]
        self.maps = self._discover_maps()
        if not self.maps:
            raise RuntimeError(f'No maps (*.yaml) found in {self.maps_dirs}')
        if map_name not in self.maps:
            map_name = 'pasillo_aeb' if 'pasillo_aeb' in self.maps else sorted(self.maps)[0]
        self.world = None
        self.load_map(map_name)

        self.mode = cfg.get('mode', 'mix') if cfg.get('mode', 'mix') in MODES else 'mix'
        self.paused = False
        self.estop = False
        # Freno de emergencia pedido por ROS. Caduca solo si el nodo deja de
        # publicarlo, para que el coche no quede bloqueado si ese nodo muere.
        self.ros_estop = False
        self.ros_estop_stamp = 0.0
        self.ros_estop_timeout = float(cfg.get('estop_timeout', 1.0))
        self.rtf = 1.0
        self.auto_reset = False
        self.ros_cmd = Command()
        self.kbd_cmd = Command()
        self.ros_cmd_count = 0
        self.ros_seen = False
        self.events = deque(maxlen=200)
        self.listeners = []  # callables(event_dict)
        self._event_id = 0

    # ---------------------------------------------------------------- maps
    def _discover_maps(self):
        maps = {}
        for d in self.maps_dirs:
            for path in sorted(glob.glob(os.path.join(d, '*.yaml'))):
                name = os.path.splitext(os.path.basename(path))[0]
                maps.setdefault(name, path)
        return maps

    def map_list(self):
        out = []
        for name, path in sorted(self.maps.items()):
            desc = ''
            try:
                import yaml
                with open(path) as f:
                    desc = (yaml.safe_load(f) or {}).get('websim_description', '')
            except Exception:
                pass
            out.append({'name': name, 'description': desc})
        return out

    def load_map(self, name):
        with self.lock:
            prev_version = self.world.version if self.world else 0
            self.world = World(self.maps[name])
            self.world.version = prev_version + 1000
            self.spawn = self.world.default_spawn()
            self.reset(log=False)
        if hasattr(self, 'events'):
            self.log('info', f'Mapa cargado: {name}')

    # -------------------------------------------------------------- events
    def log(self, level, msg):
        self._event_id += 1
        ev = {'id': self._event_id, 'level': level, 'msg': msg, 'simt': round(self.simt, 2),
              'wall': time.time()}
        self.events.append(ev)
        for cb in list(self.listeners):
            try:
                cb(ev)
            except Exception:
                pass

    # --------------------------------------------------------------- state
    def reset(self, pose=None, log=True):
        with self.lock:
            if pose is not None:
                self.spawn = tuple(pose)
            x, y, yaw = self.spawn
            self.state = VehicleState(x=x, y=y, yaw=yaw)
            self.simt = getattr(self, 'simt', 0.0)
            self.epoch = getattr(self, 'epoch', 0) + 1
            self.colliding = False
            self._coll_pos = (x, y)
            self.n_collisions = 0
            self.trail = deque(maxlen=3000)
            self.lap_start = self.simt
            self.lap_last = None
            self.lap_best = None
            self.lap_count = 0
            self._lap_armed = False
            self._prev_along = 0.0
            self.distance = 0.0
            self.max_speed = 0.0
            self.last_source = 'none'
            self.applied = Command()
            self._scan_due = self.simt
            self._odom_due = self.simt
            self.scan = self._compute_scan()
        if log:
            self.log('info', 'Reinicio: coche en la posición de salida')

    def set_pose(self, x, y, yaw):
        with self.lock:
            self.spawn = (x, y, yaw)
            self.reset(log=False)
        self.log('info', f'Coche colocado en ({x:.2f}, {y:.2f}), {math.degrees(yaw):.0f}°')

    def set_mode(self, mode):
        if mode in MODES and mode != self.mode:
            self.mode = mode
            self.log('info', f'Control: {MODE_NAMES[mode]}')

    def set_paused(self, paused):
        if paused != self.paused:
            self.paused = paused
            self.log('info', 'Pausado' if paused else 'En marcha')

    def set_estop(self, on):
        if on != self.estop:
            self.estop = on
            self.log('warn' if on else 'info', 'Freno de emergencia ACTIVADO' if on else 'Freno de emergencia liberado')

    def set_ros_estop(self, on):
        """Freno pedido desde ROS. Tiene la misma prioridad absoluta que el manual."""
        if on:
            self.ros_estop_stamp = time.monotonic()
        if on != self.ros_estop:
            self.ros_estop = on
            self.log('warn' if on else 'info',
                     'Freno de emergencia ACTIVADO desde ROS' if on else 'Freno de emergencia (ROS) liberado')

    def ros_estop_active(self, now=None):
        if not self.ros_estop:
            return False
        now = now if now is not None else time.monotonic()
        if (now - self.ros_estop_stamp) >= self.ros_estop_timeout:
            self.ros_estop = False   # el nodo dejó de publicarlo: se libera solo
            self.log('info', 'Freno de emergencia (ROS) caducado')
            return False
        return True

    # ------------------------------------------------------------ commands
    def on_ros_drive(self, speed, steer):
        now = time.monotonic()
        self.ros_cmd = Command(float(speed), float(steer), now)
        self.ros_cmd_count += 1
        if not self.ros_seen:
            self.ros_seen = True
            self.log('info', 'Primer mensaje recibido en /drive')

    def on_kbd(self, speed, steer):
        self.kbd_cmd = Command(float(speed), float(steer), time.monotonic())

    def _select_command(self):
        now = time.monotonic()
        kbd_fresh = (now - self.kbd_cmd.stamp) < self.kbd_timeout
        kbd = self.kbd_cmd if kbd_fresh else Command()
        # El freno de emergencia manda sobre todo lo demás, venga de la interfaz o de ROS
        if self.estop:
            return Command(0.0, self.state.steer), 'estop'
        if self.ros_estop_active(now):
            return Command(0.0, self.state.steer), 'estop_ros'
        if self.mode == 'ros':
            return (self.ros_cmd, 'ros') if self.ros_cmd.stamp else (Command(), 'none')
        if self.mode == 'kbd':
            return kbd, ('kbd' if kbd_fresh else 'none')
        # mix: a recent /drive message wins over the keyboard (like a priority mux)
        if self.ros_cmd.stamp and (now - self.ros_cmd.stamp) < self.mix_priority_window:
            return self.ros_cmd, 'ros'
        return kbd, ('kbd' if kbd_fresh else 'none')

    # ---------------------------------------------------------------- step
    def step(self, n=1):
        """Advance n physics steps. Returns (scan_ready, odom_ready) flags."""
        scan_ready = odom_ready = False
        with self.lock:
            for _ in range(n):
                cmd, src = self._select_command()
                if src != self.last_source:
                    self.last_source = src
                self.applied = cmd
                prev = self.state
                new = dyn_step(prev, cmd.speed, cmd.steer, self.dt, self.p)
                if self.world.footprint_collides(new.x, new.y, new.yaw, self.p.length, self.p.width):
                    # stay where we were, stopped; reversing out is still possible next step
                    new = VehicleState(prev.x, prev.y, new.steer, 0.0, prev.yaw, 0.0, 0.0)
                    self._coll_pos = (prev.x, prev.y)
                    if not self.colliding:
                        self.colliding = True
                        self.n_collisions += 1
                        self.log('error', f'Colisión a {abs(prev.v):.2f} m/s')
                        if self.auto_reset:
                            self.reset(log=False)
                            self.log('info', 'Reinicio automático tras colisión')
                            continue
                elif self.colliding and math.hypot(new.x - self._coll_pos[0], new.y - self._coll_pos[1]) > 0.15:
                    # only "free" again once we actually moved away from the impact point
                    self.colliding = False
                self.distance += math.hypot(new.x - prev.x, new.y - prev.y)
                self.state = new
                self.max_speed = max(self.max_speed, abs(new.v))
                self.simt += self.dt
                self._update_lap()
                if not self.trail or math.hypot(new.x - self.trail[-1][0], new.y - self.trail[-1][1]) > 0.05:
                    self.trail.append((round(new.x, 3), round(new.y, 3), round(new.v, 2)))
                if self.simt >= self._odom_due - 1e-9:
                    self._odom_due = max(self._odom_due + 1.0 / self.odom_rate, self.simt)
                    odom_ready = True
                if self.simt >= self._scan_due - 1e-9:
                    # fixed-rate schedule (not "now + period") so 40 Hz really is 40 Hz
                    self._scan_due = max(self._scan_due + 1.0 / self.scan_rate, self.simt)
                    self.scan = self._compute_scan()
                    scan_ready = True
        return scan_ready, odom_ready

    def _compute_scan(self):
        s = self.state
        ox = s.x + self.scan_offset * math.cos(s.yaw)
        oy = s.y + self.scan_offset * math.sin(s.yaw)
        r = self.world.raycast(ox, oy, s.yaw, self.angles, self.scan_range)
        if self.scan_noise > 0:
            r = r + np.random.normal(0.0, self.scan_noise, r.shape)
        return np.clip(r, 0.0, self.scan_range).astype(np.float32)

    def _update_lap(self):
        """Start/finish line: perpendicular to the spawn heading, through the spawn point."""
        sx, sy, syaw = self.spawn
        s = self.state
        dx, dy = s.x - sx, s.y - sy
        along = dx * math.cos(syaw) + dy * math.sin(syaw)
        lateral = abs(-dx * math.sin(syaw) + dy * math.cos(syaw))
        if not self._lap_armed and abs(along) > 3.0:
            self._lap_armed = True
        if self._lap_armed and lateral < 3.0 and self._prev_along < 0.0 <= along:
            lap = self.simt - self.lap_start
            self.lap_count += 1
            self.lap_last = lap
            best = self.lap_best is None or lap < self.lap_best
            if best:
                self.lap_best = lap
            self.log('lap', f'Vuelta {self.lap_count}: {lap:.2f} s' + ('  (mejor)' if best else ''))
            self.lap_start = self.simt
            self._lap_armed = False
        self._prev_along = along

    # --------------------------------------------------------------- edits
    def paint(self, points, radius, mode):
        with self.lock:
            self.world.paint(points, radius, mode)
            self.scan = self._compute_scan()

    def clear_edits(self):
        with self.lock:
            self.world.clear_edits()
            self.scan = self._compute_scan()
        self.log('info', 'Paredes dibujadas borradas')
