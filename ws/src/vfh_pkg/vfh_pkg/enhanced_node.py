"""ROS adapter for the hardware-independent enhanced controller."""

from dataclasses import fields
import json
import math
import time

from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

from .motion_core import Config, MotionController
from .motion_geometry import Mount


class EnhancedAvoidance:
    """Own scan ingestion and a steady 25 Hz publication clock; no fusion inputs."""

    def __init__(self, node):
        self.node = node
        node.declare_parameter('scan_topic', '/scan')
        node.declare_parameter('cmd_topic', '/cmd_vel_vfh')
        values = {}
        for item in fields(Config):
            default = '[]' if item.name == 'footprint_xy' else item.default
            node.declare_parameter(item.name, default)
            value = node.get_parameter(item.name).value
            values[item.name] = tuple(tuple(p) for p in json.loads(
                value)) if item.name == 'footprint_xy' else value
        self.core = MotionController(Config(**values))
        self.cfg = self.core.cfg
        self.tf_buffer = self.tf_listener = None
        if self.cfg.transform_mode == 'tf':
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, node)
        self.cmd_pub = node.create_publisher(Twist, node.get_parameter('cmd_topic').value, 1)
        self.scan_sub = node.create_subscription(LaserScan, node.get_parameter('scan_topic').value,
                                                 self.on_scan, qos_profile_sensor_data)
        self.last_log = -math.inf
        self.log_key = None
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = node.create_timer(1 / self.cfg.control_hz, self.tick, clock=self.steady_clock)
        node.get_logger().info(
            f'Enhanced LiDAR-only controller: profile={self.cfg.speed_profile} '
            f'requested_limits={self.core.limits} geometry={self.cfg.geometry_mode} '
            f'curves={self.cfg.enable_curves}; '
            'legacy geometry/model are unverified until measured')

    def mount_for(self, msg):
        """Use scan-time base<-scan TF once; never also apply configured offsets."""
        transform = self.tf_buffer.lookup_transform(self.cfg.base_frame, msg.header.frame_id,
                                                    Time.from_msg(msg.header.stamp))
        stamp_ns = transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec
        scan_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns and abs(scan_ns - stamp_ns) * 1e-9 > self.cfg.transform_timeout_sec:
            raise ValueError('transform_unavailable_or_stale')
        q = transform.transform.rotation
        if (not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w))
                or abs(q.x) > 1e-4 or abs(q.y) > 1e-4
                or abs(q.z * q.z + q.w * q.w - 1) > 1e-3):
            raise ValueError('transform_nonplanar_or_invalid')
        t = transform.transform.translation
        return Mount(t.x, t.y, 2 * math.atan2(q.z, q.w))

    def on_scan(self, msg):
        now = time.monotonic()
        mount = None
        if self.tf_buffer is not None:
            try:
                mount = self.mount_for(msg)
            except Exception:
                self.core.last_arrival = now
                self.core.observation_status = 'transform_unavailable_or_stale'
                self.core.invalidate('transform_unavailable_or_stale')
                self.cmd_pub.publish(Twist())
                return
        accepted = self.core.observe(msg, now, self.node.get_clock().now().nanoseconds, mount)
        if not accepted and self.core.input_fault:
            self.cmd_pub.publish(Twist())

    def tick(self):
        now = time.monotonic()
        decision = self.core.tick(now, self.node.get_clock().now().nanoseconds)
        # A slow planning tick must not publish motion based on an age captured
        # before computation. Recheck at publication, independently of callbacks.
        published_at = time.monotonic()
        if self.core.scan is not None:
            scan = self.core.scan
            age = max(published_at - scan.received + max(0, scan.source_age),
                      (self.node.get_clock().now().nanoseconds - scan.stamp_ns) * 1e-9)
            decision.scan_age = age
            if age > self.cfg.scan_timeout_sec:
                self.core.invalidate('scan_timeout_during_planning')
                self.core.begin_recovery(published_at)
                self.core.stop(decision, 'scan_timeout_during_planning', published_at)
        if published_at - now > self.cfg.control_compute_budget_sec:
            self.core.begin_recovery(published_at)
            self.core.stop(decision, 'control_compute_budget_exceeded', published_at)
        msg = Twist()
        msg.linear.x, msg.angular.z = decision.command.v, decision.command.w
        self.cmd_pub.publish(msg)
        self.core.last_publication = published_at
        key = (decision.state, decision.reason, decision.passing_side)
        if now - self.last_log >= (0.2 if key != self.log_key else 0.5):
            def clean(value):
                if isinstance(value, float) and not math.isfinite(value):
                    return None
                if hasattr(value, '__dataclass_fields__'):
                    return {k: clean(v) for k, v in vars(value).items()}
                return value
            record = clean(decision)
            record.update(
                event='motion',
                geometry=self.cfg.geometry_mode,
                coverage_max_gap_deg=self.cfg.coverage_max_gap_deg,
                arrival_age_sec=None if self.core.last_arrival is None else now -
                self.core.last_arrival,
                accepted_age_sec=None if self.core.last_accepted is None else now -
                self.core.last_accepted,
                requested_units='m/s,rad/s; physical calibration required')
            self.node.get_logger().info(json.dumps(record, sort_keys=True, allow_nan=False))
            self.last_log, self.log_key = now, key
