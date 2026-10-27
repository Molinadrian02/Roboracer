"""ROS 2 side. Same topics and frames as f1tenth_gym_ros (gym_bridge), so lab nodes
work unchanged:  /scan, /ego_racecar/odom, /map, TF map->ego_racecar/base_link,
/drive (AckermannDriveStamped) and /initialpose."""
import math
import threading
import time
from collections import deque

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

try:
    from rosidl_runtime_py.utilities import get_message
except ImportError:  # pragma: no cover
    get_message = None

# Topic types the telemetry panel can plot, and which fields to offer for each.
PLOTTABLE = {
    'std_msgs/msg/Float32': ['data'], 'std_msgs/msg/Float64': ['data'],
    'std_msgs/msg/Int8': ['data'], 'std_msgs/msg/Int16': ['data'],
    'std_msgs/msg/Int32': ['data'], 'std_msgs/msg/Int64': ['data'],
    'std_msgs/msg/UInt8': ['data'], 'std_msgs/msg/UInt16': ['data'],
    'std_msgs/msg/UInt32': ['data'], 'std_msgs/msg/Bool': ['data'],
    'ackermann_msgs/msg/AckermannDriveStamped': ['drive.speed', 'drive.steering_angle'],
    'nav_msgs/msg/Odometry': ['twist.twist.linear.x', 'twist.twist.angular.z'],
    'geometry_msgs/msg/Twist': ['linear.x', 'angular.z'],
}
OWN_TOPICS = {'/scan', '/ego_racecar/odom', '/map', '/tf', '/tf_static', '/websim/collision',
              '/parameter_events', '/rosout'}


def _yaw_to_quat(q, yaw):
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)


def _get_field(msg, path):
    for part in path.split('.'):
        msg = getattr(msg, part)
    return float(msg)


class RosBridge:
    def __init__(self, sim, cfg, args=None):
        self.sim = sim
        rclpy.init(args=args)
        self.node = Node('websim')
        n = self.node
        self.base_frame = cfg.get('ego_namespace', 'ego_racecar')
        self.scan_topic = cfg.get('ego_scan_topic', '/scan')
        self.odom_topic = cfg.get('ego_odom_topic', '/ego_racecar/odom')
        self.drive_topic = cfg.get('ego_drive_topic', '/drive')
        self.estop_topic = cfg.get('ego_estop_topic', '/emergency_brake')

        self.pub_scan = n.create_publisher(LaserScan, self.scan_topic, 10)
        self.pub_odom = n.create_publisher(Odometry, self.odom_topic, 10)
        self.pub_coll = n.create_publisher(Bool, '/websim/collision', 10)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_map = n.create_publisher(OccupancyGrid, '/map', map_qos)
        self.tfb = TransformBroadcaster(n)
        self.static_tfb = StaticTransformBroadcaster(n)

        n.create_subscription(AckermannDriveStamped, self.drive_topic, self._on_drive, 10)
        n.create_subscription(PoseWithCovarianceStamped, '/initialpose', self._on_initialpose, 10)
        n.create_subscription(Bool, self.estop_topic, self._on_estop, 10)

        self.lock = threading.Lock()
        self.drive_stamps = deque(maxlen=400)
        self.last_drive = None
        self.drive_publishers = []
        self.topics = []           # [{'topic', 'type', 'fields'}]
        self.plot_subs = {}        # key 'topic|field' -> subscription
        self.plot_samples = {}     # key -> list of (simt, value)
        self.scan_stamps = deque(maxlen=200)
        self._last_coll = None

        n.create_timer(1.0, self._refresh_graph)
        self._publish_static_tf()
        self.publish_map()

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(n)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self):
        try:
            self.executor.spin()
        except Exception as e:  # shutdown races
            print(f'[websim] ROS executor stopped: {e!r}')

    def shutdown(self):
        try:
            self.executor.shutdown()
            self.node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

    # ------------------------------------------------------------ inputs
    def _on_drive(self, msg):
        self.sim.on_ros_drive(msg.drive.speed, msg.drive.steering_angle)
        with self.lock:
            self.drive_stamps.append(time.monotonic())
            self.last_drive = (msg.drive.speed, msg.drive.steering_angle)

    def _on_estop(self, msg):
        self.sim.set_ros_estop(bool(msg.data))

    def _on_initialpose(self, msg):
        p = msg.pose.pose
        yaw = math.atan2(2.0 * (p.orientation.w * p.orientation.z + p.orientation.x * p.orientation.y),
                         1.0 - 2.0 * (p.orientation.y ** 2 + p.orientation.z ** 2))
        self.sim.set_pose(p.position.x, p.position.y, yaw)

    def _refresh_graph(self):
        n = self.node
        try:
            infos = n.get_publishers_info_by_topic(self.drive_topic)
            pubs = sorted({(i.node_namespace.rstrip('/') + '/' + i.node_name) for i in infos
                           if i.node_name != n.get_name()})
            topics = []
            for name, types in n.get_topic_names_and_types():
                if name in OWN_TOPICS:
                    continue
                for t in types:
                    if t in PLOTTABLE:
                        topics.append({'topic': name, 'type': t, 'fields': PLOTTABLE[t]})
            with self.lock:
                self.drive_publishers = pubs
                self.topics = topics
        except Exception as e:
            print(f'[websim] graph refresh failed: {e!r}')

    # ------------------------------------------------------------ plots
    def set_plot_keys(self, keys):
        """keys: list of 'topic|field'. Creates/destroys subscriptions to match."""
        keys = list(dict.fromkeys(keys))[:6]
        with self.lock:
            known = {t['topic']: t['type'] for t in self.topics}
        for key in list(self.plot_subs):
            if key not in keys:
                self.node.destroy_subscription(self.plot_subs.pop(key))
                with self.lock:
                    self.plot_samples.pop(key, None)
        for key in keys:
            if key in self.plot_subs:
                continue
            topic, _, field = key.partition('|')
            type_str = known.get(topic)
            if not type_str or get_message is None:
                continue
            msg_cls = get_message(type_str)
            field = field or PLOTTABLE[type_str][0]

            def cb(msg, key=key, field=field):
                try:
                    v = _get_field(msg, field)
                except Exception:
                    return
                with self.lock:
                    buf = self.plot_samples.setdefault(key, [])
                    if len(buf) < 2000:
                        buf.append((round(self.sim.simt, 3), v))
            qos = qos_profile_sensor_data if 'scan' in topic else 10
            self.plot_subs[key] = self.node.create_subscription(msg_cls, topic, cb, qos)
        return list(self.plot_subs)

    def drain_plot_samples(self):
        with self.lock:
            out = self.plot_samples
            self.plot_samples = {k: [] for k in out}
        return {k: v for k, v in out.items() if v}

    def status(self):
        now = time.monotonic()
        with self.lock:
            hz = sum(1 for t in self.drive_stamps if now - t < 1.0)
            scan_hz = sum(1 for t in self.scan_stamps if now - t < 1.0)
            age = (now - self.drive_stamps[-1]) if self.drive_stamps else None
            return {
                'ok': True,
                'drive_hz': hz,
                'scan_hz': scan_hz,
                'drive_age': round(age, 2) if age is not None else None,
                'drive_pubs': self.drive_publishers,
                'last_drive': self.last_drive,
                'topics': self.topics,
                'plots': list(self.plot_subs),
            }

    # ------------------------------------------------------------ outputs
    def _publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = f'{self.base_frame}/base_link'
        t.child_frame_id = f'{self.base_frame}/laser'
        t.transform.translation.x = self.sim.scan_offset
        t.transform.rotation.w = 1.0
        self.static_tfb.sendTransform(t)

    def publish(self, scan_ready, odom_ready):
        s = self.sim.state
        stamp = self.node.get_clock().now().to_msg()
        if odom_ready:
            o = Odometry()
            o.header.stamp = stamp
            o.header.frame_id = 'map'
            o.child_frame_id = f'{self.base_frame}/base_link'
            o.pose.pose.position.x = s.x
            o.pose.pose.position.y = s.y
            _yaw_to_quat(o.pose.pose.orientation, s.yaw)
            o.twist.twist.linear.x = s.v * math.cos(s.slip)
            o.twist.twist.linear.y = s.v * math.sin(s.slip)
            o.twist.twist.angular.z = s.yaw_rate
            self.pub_odom.publish(o)

            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'map'
            t.child_frame_id = f'{self.base_frame}/base_link'
            t.transform.translation.x = s.x
            t.transform.translation.y = s.y
            _yaw_to_quat(t.transform.rotation, s.yaw)
            self.tfb.sendTransform(t)

            if self._last_coll != self.sim.colliding:
                self._last_coll = self.sim.colliding
                self.pub_coll.publish(Bool(data=bool(self.sim.colliding)))
        if scan_ready:
            m = LaserScan()
            m.header.stamp = stamp
            m.header.frame_id = f'{self.base_frame}/laser'
            m.angle_min = self.sim.angle_min
            m.angle_max = self.sim.scan_fov / 2.0
            m.angle_increment = self.sim.angle_inc
            m.range_min = 0.0
            m.range_max = self.sim.scan_range
            m.ranges = self.sim.scan.tolist()
            self.pub_scan.publish(m)
            with self.lock:
                self.scan_stamps.append(time.monotonic())

    def publish_map(self):
        w = self.sim.world
        g = OccupancyGrid()
        g.header.stamp = self.node.get_clock().now().to_msg()
        g.header.frame_id = 'map'
        g.info.resolution = float(w.resolution)
        g.info.width = int(w.width)
        g.info.height = int(w.height)
        g.info.origin.position.x = w.origin[0]
        g.info.origin.position.y = w.origin[1]
        g.info.origin.orientation.w = 1.0
        g.data = w.occupancy_grid_data()
        self.pub_map.publish(g)
