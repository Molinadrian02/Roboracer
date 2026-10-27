"""Entry point: `ros2 run f1tenth_websim websim` (or `python3 -m f1tenth_websim.main --no-ros`)."""
import argparse
import asyncio
import json
import math
import os
import signal
import sys
import time

import yaml

from .sim import Simulator, MODES

HERE = os.path.dirname(os.path.abspath(__file__))


def _share_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('f1tenth_websim')
    except Exception:
        return os.path.dirname(HERE)  # running from the source tree


def _find_dir(*candidates):
    for c in candidates:
        if c and os.path.isdir(c):
            return c
    return None


class App:
    def __init__(self, args, ros_args):
        share = _share_dir()
        cfg = {}
        cfg_path = args.config or _find_path(os.path.join(share, 'config', 'websim.yaml'))
        if cfg_path:
            with open(cfg_path) as f:
                cfg = yaml.safe_load(f) or {}
        self.cfg = cfg
        maps_dirs = list(args.maps or []) + list(cfg.get('extra_map_dirs', []) or [])
        maps_dirs.append(_find_dir(os.path.join(share, 'maps'), os.path.join(os.path.dirname(HERE), 'maps')))
        maps_dirs = [os.path.expanduser(d) for d in maps_dirs if d]
        self.sim = Simulator(maps_dirs, args.map or cfg.get('map'), cfg=cfg)
        self.web_dir = _find_dir(os.path.join(share, 'web'), os.path.join(HERE, 'web'))
        self.port = args.port or int(cfg.get('port', 8080))
        self.host = args.host or cfg.get('host', '0.0.0.0')
        self.ros = None
        self.ros_error = None
        if not args.no_ros:
            try:
                from .ros_bridge import RosBridge
                self.ros = RosBridge(self.sim, cfg, ros_args)
            except Exception as e:
                self.ros_error = repr(e)
                print(f'[websim] ROS 2 not available, running without ROS: {e!r}')
        self.sim.listeners.append(self._on_event)
        self._pending_events = []
        self._map_dirty_at = None

    # ---------------------------------------------------------------- web
    def routes(self):
        def grid(_q):
            return 200, 'image/png', self.sim.world.grid_png()

        def maps(_q):
            return 200, 'application/json', json.dumps(self.sim.map_list()).encode()

        return {'/api/grid.png': grid, '/api/maps': maps}

    def _on_event(self, ev):
        self._pending_events.append(ev)

    def hello(self):
        s, w = self.sim, self.sim.world
        return {
            't': 'hello',
            'map': self.map_meta(),
            'maps': s.map_list(),
            'scan': {'angle_min': s.angle_min, 'inc': s.angle_inc, 'n': s.scan_beams,
                     'range': s.scan_range, 'offset': s.scan_offset, 'rate': s.scan_rate},
            'car': {'length': s.p.length, 'width': s.p.width, 's_max': s.p.s_max,
                    'v_max': s.p.v_max, 'wheelbase': s.p.lf + s.p.lr},
            'trail': list(s.trail),
            'events': list(s.events)[-60:],
            'ros': self.ros is not None,
            'ros_error': self.ros_error,
            'topics': {'drive': self.ros.drive_topic if self.ros else '/drive',
                       'scan': self.ros.scan_topic if self.ros else '/scan',
                       'odom': self.ros.odom_topic if self.ros else '/ego_racecar/odom'},
        }

    def map_meta(self):
        w = self.sim.world
        return {'name': w.name, 'resolution': w.resolution, 'origin': w.origin,
                'width': w.width, 'height': w.height, 'version': w.version,
                'description': w.description, 'spawn': self.sim.spawn}

    def state_msg(self):
        s = self.sim
        st = s.state
        msg = {
            't': 's',
            'simt': round(s.simt, 3),
            'paused': s.paused, 'estop': s.estop, 'ros_estop': s.ros_estop_active(),
            'mode': s.mode, 'rtf': s.rtf,
            'auto_reset': s.auto_reset,
            'src': s.last_source,
            'x': round(st.x, 4), 'y': round(st.y, 4), 'yaw': round(st.yaw, 4),
            'v': round(st.v, 3), 'steer': round(st.steer, 4), 'yaw_rate': round(st.yaw_rate, 3),
            'cmd': [round(s.applied.speed, 3), round(s.applied.steer, 4)],
            'coll': s.colliding, 'ncoll': s.n_collisions,
            'epoch': s.epoch,
            'map_version': s.world.version,
            'lap': {'cur': round(s.simt - s.lap_start, 2), 'last': s.lap_last, 'best': s.lap_best,
                    'n': s.lap_count},
            'dist': round(s.distance, 1), 'vmax': round(s.max_speed, 2),
            'scan': [int(r * 100) for r in s.scan],
        }
        if self.ros:
            msg['ros'] = self.ros.status()
            samples = self.ros.drain_plot_samples()
            if samples:
                msg['samples'] = samples
        if self._pending_events:
            msg['events'], self._pending_events = self._pending_events, []
        return msg

    async def on_ws_open(self, client):
        await client.send_text(json.dumps(self.hello()))

    async def on_ws_message(self, client, m):
        s = self.sim
        t = m.get('t')
        if t == 'kbd':
            s.on_kbd(float(m.get('v', 0.0)), float(m.get('s', 0.0)))
        elif t == 'mode' and m.get('mode') in MODES:
            s.set_mode(m['mode'])
        elif t == 'pause':
            s.set_paused(bool(m.get('on')))
        elif t == 'estop':
            s.set_estop(bool(m.get('on')))
        elif t == 'reset':
            s.reset()
        elif t == 'pose':
            s.set_pose(float(m['x']), float(m['y']), float(m['yaw']))
        elif t == 'rtf':
            s.rtf = min(max(float(m.get('v', 1.0)), 0.05), 4.0)
            s.log('info', f'Velocidad de simulación ×{s.rtf:g}')
        elif t == 'step':
            n = int(m.get('n', 1))
            sr, orr = s.step(max(1, min(n, 500)))
            if self.ros:
                self.ros.publish(True, True)
        elif t == 'map' and m.get('name') in s.maps:
            s.load_map(m['name'])
            if self.ros:
                self.ros.publish_map()
            await self.server.broadcast({'t': 'map', 'map': self.map_meta()})
        elif t == 'paint':
            pts = [(float(a), float(b)) for a, b in m.get('pts', [])][:400]
            if pts:
                s.paint(pts, float(m.get('r', 0.15)), 'wall' if m.get('mode') == 'wall' else 'erase')
                self._map_dirty_at = time.monotonic()
        elif t == 'clear_edits':
            s.clear_edits()
            self._map_dirty_at = time.monotonic()
        elif t == 'auto_reset':
            s.auto_reset = bool(m.get('on'))
        elif t == 'plot' and self.ros:
            self.ros.set_plot_keys([str(k) for k in m.get('keys', [])])

    # --------------------------------------------------------------- loops
    async def sim_loop(self):
        s = self.sim
        last = time.monotonic()
        acc = 0.0
        while True:
            await asyncio.sleep(0.004)
            now = time.monotonic()
            elapsed, last = min(now - last, 0.1), now
            if s.paused:
                acc = 0.0
                continue
            acc += elapsed * s.rtf
            n = int(acc / s.dt)
            if n <= 0:
                continue
            acc -= n * s.dt
            scan_ready, odom_ready = s.step(min(n, 40))
            if self.ros:
                try:
                    self.ros.publish(scan_ready, odom_ready)
                except Exception as e:
                    print(f'[websim] publish failed: {e!r}')

    async def ui_loop(self):
        while True:
            await asyncio.sleep(1 / 30)
            if self.server.clients:
                await self.server.broadcast(self.state_msg())
            # republish /map once a painting burst has settled
            if self._map_dirty_at and time.monotonic() - self._map_dirty_at > 0.4:
                self._map_dirty_at = None
                if self.ros:
                    self.ros.publish_map()

    async def run(self):
        from .webserver import WebServer
        self.server = WebServer(self.web_dir, self.routes(), self.on_ws_message, self.on_ws_open)
        await self.server.start(self.host, self.port)
        url_host = 'localhost' if self.host in ('0.0.0.0', '') else self.host
        print(f'\n  websim listo →  http://{url_host}:{self.port}\n'
              f'  ROS 2: {"conectado" if self.ros else "desactivado"}   mapa: {self.sim.world.name}\n',
              flush=True)
        await asyncio.gather(self.sim_loop(), self.ui_loop())


def _find_path(p):
    return p if p and os.path.isfile(p) else None


def main(argv=None):
    argv = sys.argv if argv is None else argv
    try:
        from rclpy.utilities import remove_ros_args
        clean = remove_ros_args(argv)
    except Exception:
        clean = argv
    ap = argparse.ArgumentParser(prog='websim', description='Simulador web F1TENTH compatible con f1tenth_gym_ros')
    ap.add_argument('--port', type=int, default=None)
    ap.add_argument('--host', default=None)
    ap.add_argument('--map', default=None, help='nombre del mapa (sin .yaml)')
    ap.add_argument('--maps', action='append', help='carpeta extra con mapas .yaml/.png (repetible)')
    ap.add_argument('--config', default=None, help='fichero websim.yaml')
    ap.add_argument('--no-ros', action='store_true', help='ejecutar sin ROS 2 (solo teclado)')
    args, _ = ap.parse_known_args(clean[1:])
    app = App(args, argv)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    task = loop.create_task(app.run())
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, task.cancel)
        except NotImplementedError:
            pass
    try:
        loop.run_until_complete(task)
    except (asyncio.CancelledError, KeyboardInterrupt):
        pass
    finally:
        if app.ros:
            app.ros.shutdown()
        print('[websim] cerrado')


if __name__ == '__main__':
    main()
