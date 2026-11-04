#!/usr/bin/env python3
import math
import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Bool, Float32


class VFHObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('vfh_obstacle_avoidance')

        # ---------------------------------
        # Topics
        # ---------------------------------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_topic', '/cmd_vel_vfh')
        self.declare_parameter('fusion_penalty_topic', '/fusion/sector_penalties')
        self.declare_parameter('fusion_stop_topic', '/fusion/critical_stop')
        self.declare_parameter('fusion_speed_topic', '/fusion/speed_limit')

        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        fusion_penalty_topic = self.get_parameter('fusion_penalty_topic').get_parameter_value().string_value
        fusion_stop_topic = self.get_parameter('fusion_stop_topic').get_parameter_value().string_value
        fusion_speed_topic = self.get_parameter('fusion_speed_topic').get_parameter_value().string_value

        # ---------------------------------
        # Core VFH behavior
        # ---------------------------------
        self.declare_parameter('sector_angle_deg', 5.0)
        self.declare_parameter('front_fov_deg', 160.0)
        self.declare_parameter('robot_radius', 0.20)
        self.declare_parameter('safety_margin', 0.16)
        self.declare_parameter('max_range_clip', 3.5)
        self.declare_parameter('obstacle_influence_dist', 1.7)

        self.declare_parameter('hist_threshold', 1.15)
        self.declare_parameter('min_open_sectors', 3)

        self.declare_parameter('forward_speed', 0.24)
        self.declare_parameter('min_speed', 0.05)
        self.declare_parameter('max_yaw_rate', 1.0)
        self.declare_parameter('steer_gain', 1.15)

        self.declare_parameter('front_slowdown_dist', 1.0)
        self.declare_parameter('front_stop_dist', 0.34)
        self.declare_parameter('hard_turn_dist', 0.52)

        self.declare_parameter('forward_weight', 2.6)
        self.declare_parameter('smooth_weight', 1.3)
        self.declare_parameter('clearance_weight', 1.8)
        self.declare_parameter('commitment_weight', 0.9)

        self.declare_parameter('ema_alpha', 0.35)
        self.declare_parameter('debug_log', True)

        # ---------------------------------
        # Escape / corner recovery behavior
        # ---------------------------------
        self.declare_parameter('escape_trigger_cycles', 6)
        self.declare_parameter('blocked_open_fraction', 0.10)
        self.declare_parameter('escape_turn_angle_deg', 180.0)
        self.declare_parameter('escape_turn_rate', 0.8)
        self.declare_parameter('escape_clearance_dist', 0.55)
        self.declare_parameter('escape_holdoff_cycles', 10)

        # ---------------------------------
        # Hazard escape behavior
        # ---------------------------------
        self.declare_parameter('hazard_escape_angle_deg', 135.0)
        self.declare_parameter('hazard_escape_turn_rate', 0.9)

        # ---------------------------------
        # Fusion-priority tuning
        # ---------------------------------
        self.declare_parameter('fusion_penalty_weight', 2.4)
        self.declare_parameter('fusion_clearance_boost', 2.0)
        self.declare_parameter('fusion_forward_suppression', 1.8)
        self.declare_parameter('fusion_smooth_suppression', 0.9)
        self.declare_parameter('fusion_priority_speed_limit', 0.75)

        self.sector_angle_deg = float(self.get_parameter('sector_angle_deg').value)
        self.front_fov_deg = float(self.get_parameter('front_fov_deg').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.safety_margin = float(self.get_parameter('safety_margin').value)
        self.max_range_clip = float(self.get_parameter('max_range_clip').value)
        self.obstacle_influence_dist = float(self.get_parameter('obstacle_influence_dist').value)

        self.hist_threshold = float(self.get_parameter('hist_threshold').value)
        self.min_open_sectors = int(self.get_parameter('min_open_sectors').value)

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.steer_gain = float(self.get_parameter('steer_gain').value)

        self.front_slowdown_dist = float(self.get_parameter('front_slowdown_dist').value)
        self.front_stop_dist = float(self.get_parameter('front_stop_dist').value)
        self.hard_turn_dist = float(self.get_parameter('hard_turn_dist').value)

        self.forward_weight = float(self.get_parameter('forward_weight').value)
        self.smooth_weight = float(self.get_parameter('smooth_weight').value)
        self.clearance_weight = float(self.get_parameter('clearance_weight').value)
        self.commitment_weight = float(self.get_parameter('commitment_weight').value)

        self.ema_alpha = float(self.get_parameter('ema_alpha').value)
        self.debug_log = bool(self.get_parameter('debug_log').value)

        self.escape_trigger_cycles = int(self.get_parameter('escape_trigger_cycles').value)
        self.blocked_open_fraction = float(self.get_parameter('blocked_open_fraction').value)
        self.escape_turn_angle_deg = float(self.get_parameter('escape_turn_angle_deg').value)
        self.escape_turn_rate = float(self.get_parameter('escape_turn_rate').value)
        self.escape_clearance_dist = float(self.get_parameter('escape_clearance_dist').value)
        self.escape_holdoff_cycles = int(self.get_parameter('escape_holdoff_cycles').value)

        self.hazard_escape_angle_deg = float(self.get_parameter('hazard_escape_angle_deg').value)
        self.hazard_escape_turn_rate = float(self.get_parameter('hazard_escape_turn_rate').value)

        self.fusion_penalty_weight = float(self.get_parameter('fusion_penalty_weight').value)
        self.fusion_clearance_boost = float(self.get_parameter('fusion_clearance_boost').value)
        self.fusion_forward_suppression = float(self.get_parameter('fusion_forward_suppression').value)
        self.fusion_smooth_suppression = float(self.get_parameter('fusion_smooth_suppression').value)
        self.fusion_priority_speed_limit = float(self.get_parameter('fusion_priority_speed_limit').value)

        # Conservative front-stop release for sparse pointcloud-derived scans.
        self.declare_parameter('scan_timeout_sec', 0.35)
        self.declare_parameter('front_release_margin', 0.10)
        self.declare_parameter('front_release_scans', 5)
        self.scan_timeout_sec = float(self.get_parameter('scan_timeout_sec').value)
        self.front_release_margin = float(self.get_parameter('front_release_margin').value)
        self.front_release_scans = int(self.get_parameter('front_release_scans').value)
        if (not math.isfinite(self.scan_timeout_sec) or self.scan_timeout_sec <= 0 or
                not math.isfinite(self.front_release_margin) or self.front_release_margin < 0 or
                self.front_release_scans < 1 or not math.isfinite(self.max_yaw_rate) or
                self.max_yaw_rate <= 0 or not 0 <= self.min_speed <= self.forward_speed or
                not 0 < self.front_stop_dist < self.front_slowdown_dist):
            raise ValueError('Invalid VFH speed, stop-release, or timeout parameters')
        self.last_scan_received = None
        self.front_stop_latched = True
        self.front_clear_count = 0
        self.front_valid_count = 0
        self.front_raw_min = float('nan')
        self.last_guard_reason = None

        # Continuous reactive mode: commit to one side until a clear front is
        # observed repeatedly. These limits are configuration, not calibration.
        self.declare_parameter('continuous_avoidance', True)
        self.declare_parameter('avoid_turn_timeout_sec', 20.0)
        self.declare_parameter('avoid_clear_scans', 5)
        self.declare_parameter('min_side_returns', 2)
        self.continuous_avoidance = bool(self.get_parameter('continuous_avoidance').value)
        self.avoid_turn_timeout_sec = float(self.get_parameter('avoid_turn_timeout_sec').value)
        self.avoid_clear_scans = int(self.get_parameter('avoid_clear_scans').value)
        self.min_side_returns = int(self.get_parameter('min_side_returns').value)
        if (not math.isfinite(self.avoid_turn_timeout_sec) or self.avoid_turn_timeout_sec <= 0 or
                self.avoid_clear_scans < 1 or self.min_side_returns < 1 or
                not 0 < self.front_stop_dist < self.hard_turn_dist < self.front_slowdown_dist or
                not math.isfinite(self.robot_radius) or self.robot_radius <= 0 or
                not math.isfinite(self.safety_margin) or self.safety_margin < 0):
            raise ValueError('Invalid continuous-avoidance thresholds, geometry or timeout')
        self.avoid_phase = 'cruise'
        self.avoid_turn_sign = 0.0
        self.avoid_clear_count = 0
        self.avoid_started_at = None
        self.avoid_log_key = None
        self.avoid_last_log = -math.inf
        self.avoid_front_min = None

        # histogram sizing
        self.num_sectors = max(5, int(round(self.front_fov_deg / self.sector_angle_deg)))
        if self.num_sectors % 2 == 0:
            self.num_sectors += 1

        self.total_hist_fov_rad = math.radians(self.front_fov_deg)
        self.sector_width_rad = self.total_hist_fov_rad / self.num_sectors
        self.center_sector = self.num_sectors // 2

        # state
        self.prev_target_angle = 0.0
        self.prev_cmd_w = 0.0
        self.last_turn_sign = 1.0

        self.mode = 'normal'
        self.blocked_counter = 0
        self.escape_holdoff_counter = 0

        self.escape_target_rad = math.radians(self.escape_turn_angle_deg)
        self.escape_accum_rad = 0.0
        self.escape_turn_dir = 1.0
        self.last_scan_stamp = None

        # fusion state
        self.fusion_penalties = [0.0] * self.num_sectors
        self.fusion_critical_stop = False
        self.fusion_speed_limit = 1.0

        # hazard escape state
        self.hazard_escape_active = False
        self.hazard_escape_accum_rad = 0.0
        self.hazard_escape_target_rad = math.radians(self.hazard_escape_angle_deg)
        self.hazard_escape_turn_dir = 1.0

        # subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.penalty_sub = self.create_subscription(
            Float32MultiArray,
            fusion_penalty_topic,
            self.on_fusion_penalties,
            10
        )

        self.stop_sub = self.create_subscription(
            Bool,
            fusion_stop_topic,
            self.on_fusion_stop,
            10
        )

        self.speed_sub = self.create_subscription(
            Float32,
            fusion_speed_topic,
            self.on_fusion_speed,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        self.watchdog_timer = self.create_timer(0.05, self.check_scan_timeout)

        self.get_logger().info('VFH obstacle avoidance node started.')
        self.get_logger().info(f'scan_topic={scan_topic}, cmd_topic={cmd_topic}')
        self.get_logger().info(
            f'front_fov_deg={self.front_fov_deg:.1f}, sectors={self.num_sectors}, mode={self.mode}'
        )

    # ---------------- Fusion callbacks ----------------
    def on_fusion_penalties(self, msg: Float32MultiArray):
        if len(msg.data) == self.num_sectors:
            self.fusion_penalties = list(msg.data)

    def on_fusion_stop(self, msg: Bool):
        self.fusion_critical_stop = bool(msg.data)
        if self.fusion_critical_stop:
            self.guard_stop('fusion_stop')

    def on_fusion_speed(self, msg: Float32):
        self.fusion_speed_limit = self.clamp(float(msg.data), 0.0, 1.0)

    # ---------------- Main scan callback ----------------
    def scan_callback(self, msg: LaserScan):
        self.last_scan_received = time.monotonic()
        if (not msg.ranges or not math.isfinite(msg.angle_min) or
                not math.isfinite(msg.angle_increment) or msg.angle_increment == 0 or
                not math.isfinite(msg.range_min) or not math.isfinite(msg.range_max) or
                msg.range_min < 0 or msg.range_max <= msg.range_min):
            self.guard_stop('invalid_scan')
            return

        front = [float(r) for i, r in enumerate(msg.ranges)
                 if abs(msg.angle_min + i * msg.angle_increment) <= math.radians(15.0)
                 and math.isfinite(r) and r > 0 and msg.range_min <= r <= msg.range_max]
        self.front_valid_count = len(front)
        self.front_raw_min = min(front) if front else float('nan')
        if not front:
            self.guard_stop('front_unknown')
            return
        if self.fusion_critical_stop:
            self.guard_stop('fusion_stop')
            return
        self.last_guard_reason = None
        front_min = self.front_raw_min
        if front_min <= self.front_stop_dist:
            self.front_stop_latched = True
            self.front_clear_count = 0
        elif self.front_stop_latched:
            if front_min >= self.front_stop_dist + self.front_release_margin:
                self.front_clear_count += 1
                if self.front_clear_count >= self.front_release_scans:
                    self.front_stop_latched = False
            else:
                self.front_clear_count = 0

        if self.continuous_avoidance:
            self.run_continuous_avoidance(msg, front_min)
            return

        dt = self.compute_dt(msg)
        hist, sector_clearance, _ = self.build_histogram(msg)

        if len(self.fusion_penalties) == self.num_sectors:
            hist = [h + p for h, p in zip(hist, self.fusion_penalties)]

        open_mask = [h < self.hist_threshold for h in hist]
        valleys = self.find_open_valleys(open_mask)

        open_count = sum(1 for x in open_mask if x)
        open_fraction = open_count / max(1, len(open_mask))

        clogged = (open_fraction < self.blocked_open_fraction) and (front_min < self.escape_clearance_dist)

        if self.escape_holdoff_counter > 0:
            self.escape_holdoff_counter -= 1

        if self.mode == 'normal' and self.escape_holdoff_counter == 0:
            if clogged:
                self.blocked_counter += 1
            else:
                self.blocked_counter = 0

            if self.blocked_counter >= self.escape_trigger_cycles:
                self.start_escape(sector_clearance)
        else:
            if not clogged:
                self.blocked_counter = 0

        if self.hazard_escape_active:
            cmd = self.run_hazard_escape(dt)
            self.publish_command(cmd)

            if self.debug_log:
                self.get_logger().warn(
                    f'[HAZARD ESCAPE] turned={self.hazard_escape_accum_rad:.2f}/{self.hazard_escape_target_rad:.2f} '
                    f'v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}'
                )
            return

        if self.mode == 'escape':
            cmd = self.run_escape(dt)
            self.publish_command(cmd)

            if self.debug_log:
                self.get_logger().info(
                    f'[ESCAPE] turned={self.escape_accum_rad:.2f}/{self.escape_target_rad:.2f} '
                    f'v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}'
                )
            return

        fusion_urgency = self.compute_fusion_urgency()

        target_sector = self.choose_best_sector(
            valleys=valleys,
            sector_clearance=sector_clearance,
            front_min=front_min,
            fusion_urgency=fusion_urgency
        )
        target_angle = self.sector_to_angle(target_sector)

        cmd = Twist()

        raw_w = self.steer_gain * target_angle
        raw_w = self.clamp(raw_w, -self.max_yaw_rate, self.max_yaw_rate)

        if front_min < self.hard_turn_dist or fusion_urgency > 0.55:
            turn_dir = 1.0 if target_angle >= 0.0 else -1.0
            if abs(target_angle) < math.radians(8.0):
                turn_dir = self.last_turn_sign
            raw_w = turn_dir * max(abs(raw_w), (0.65 + 0.20 * fusion_urgency) * self.max_yaw_rate)

        cmd.angular.z = (1.0 - self.ema_alpha) * self.prev_cmd_w + self.ema_alpha * raw_w
        cmd.angular.z = self.clamp(cmd.angular.z, -self.max_yaw_rate, self.max_yaw_rate)

        if self.front_stop_latched or front_min <= self.front_stop_dist:
            cmd.linear.x = 0.0
        else:
            speed_scale = self.linear_scale(front_min, self.front_stop_dist, self.front_slowdown_dist)
            turn_scale = max(0.25, 1.0 - 0.65 * min(1.0, abs(cmd.angular.z) / self.max_yaw_rate))
            cmd.linear.x = self.forward_speed * speed_scale * turn_scale

            cmd.linear.x *= self.fusion_speed_limit

            if fusion_urgency > 0.0:
                cmd.linear.x *= max(0.25, 1.0 - 0.45 * fusion_urgency)

            if cmd.linear.x > 0.0:
                cmd.linear.x = min(self.forward_speed * self.fusion_speed_limit,
                                   max(self.min_speed, cmd.linear.x))

        if (front_min < self.hard_turn_dist or fusion_urgency > 0.65) and abs(cmd.angular.z) > 0.7 * self.max_yaw_rate:
            cmd.linear.x = min(cmd.linear.x, 0.05)

        self.publish_command(cmd)

        self.prev_target_angle = target_angle
        self.prev_cmd_w = cmd.angular.z
        if abs(cmd.angular.z) > 1e-3:
            self.last_turn_sign = 1.0 if cmd.angular.z > 0.0 else -1.0

        if self.debug_log:
            self.get_logger().info(
                f'[NORMAL] front_min={front_min:.2f} front_valid={self.front_valid_count} '
                f'front_latched={int(self.front_stop_latched)} clear_count={self.front_clear_count} '
                f'open_frac={open_fraction:.2f} '
                f'blocked_ctr={self.blocked_counter} '
                f'target_sector={target_sector} '
                f'target_angle={target_angle:.2f} '
                f'fusion_urgency={fusion_urgency:.2f} '
                f'v={cmd.linear.x:.2f} '
                f'w={cmd.angular.z:.2f} '
                f'fusion_stop={int(self.fusion_critical_stop)} '
                f'fusion_speed={self.fusion_speed_limit:.2f} '
                f'penalty_max={max(self.fusion_penalties) if self.fusion_penalties else 0.0:.2f} '
                f'valleys={valleys}'
            )

    def run_continuous_avoidance(self, scan, front_min):
        now = time.monotonic()
        points = []
        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or distance <= 0 or not scan.range_min <= distance <= scan.range_max:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            # LaserScan convention: +x front, +y left. Mounting must be verified.
            angle = math.atan2(math.sin(angle), math.cos(angle))
            points.append((angle, float(distance)))
        inflate = self.robot_radius + self.safety_margin
        # Include observed hazards in the configured forward footprint corridor,
        # rather than considering only the narrow central angle window.
        corridor = [r * math.cos(a) for a, r in points
                    if r * math.cos(a) > 0 and abs(r * math.sin(a)) <= inflate]
        front = min([front_min] + corridor)
        self.avoid_front_min = front
        left = [r for a, r in points if math.radians(20) <= a <= math.radians(100)]
        right = [r for a, r in points if math.radians(-100) <= a <= math.radians(-20)]
        left_min = min(left) if len(left) >= self.min_side_returns else None
        right_min = min(right) if len(right) >= self.min_side_returns else None
        nearest = min((r for _, r in points), default=math.inf)
        cmd = Twist()
        reason = 'front_clear'

        if self.avoid_phase == 'blocked':
            reason = 'avoidance_timeout_restart_required'
        elif self.fusion_speed_limit <= 0:
            self.avoid_clear_count = 0
            reason = 'fusion_speed_zero'
        else:
            if self.avoid_phase == 'cruise' and front <= self.hard_turn_dist:
                self.avoid_phase = 'choose_side'
                self.avoid_started_at = now
                self.avoid_clear_count = 0
                self.avoid_turn_sign = 0.0
                # This callback is a braking step, even with good side readings.
                reason = 'brake_for_obstacle'
            elif self.avoid_phase in ('choose_side', 'turn'):
                if now - self.avoid_started_at >= self.avoid_turn_timeout_sec:
                    self.avoid_phase = 'blocked'
                    reason = 'avoidance_timeout_restart_required'
                else:
                    if front >= self.front_slowdown_dist and not self.front_stop_latched:
                        self.avoid_clear_count += 1
                    else:
                        self.avoid_clear_count = 0
                    if self.avoid_clear_count >= self.avoid_clear_scans:
                        self.avoid_phase = 'cruise'
                        self.avoid_started_at = None
                        self.prev_cmd_w = 0.0
                        self.prev_target_angle = 0.0
                        reason = 'clear_path_confirmed_brake_before_forward'
                    elif nearest <= inflate:
                        self.avoid_clear_count = 0
                        reason = 'observed_hazard_inside_configured_turn_clearance'
                    elif left_min is None or right_min is None:
                        self.avoid_clear_count = 0
                        reason = 'side_coverage_unknown'
                    elif self.avoid_clear_count > 0:
                        # Stop while confirming the newly clear direction.
                        reason = 'confirming_clear_path'
                    else:
                        if self.avoid_phase == 'choose_side':
                            self.avoid_turn_sign = 1.0 if left_min >= right_min else -1.0
                            self.avoid_phase = 'turn'
                        cmd.angular.z = self.avoid_turn_sign * min(self.max_yaw_rate, abs(self.escape_turn_rate))
                        self.last_turn_sign = self.avoid_turn_sign
                        reason = 'committed_turn'
            elif nearest <= inflate:
                self.front_stop_latched = True
                self.front_clear_count = 0
                reason = 'observed_hazard_inside_configured_clearance'
            elif not self.front_stop_latched:
                scale = self.linear_scale(front, self.front_stop_dist, self.front_slowdown_dist)
                cmd.linear.x = min(self.forward_speed * self.fusion_speed_limit,
                                   max(self.min_speed, self.forward_speed * scale * self.fusion_speed_limit))
            else:
                reason = 'waiting_for_front_release'
        self.publish_command(cmd)
        self.prev_cmd_w = cmd.angular.z
        key = (self.avoid_phase, reason, self.avoid_turn_sign)
        if self.debug_log and (key != self.avoid_log_key or now - self.avoid_last_log >= 0.5):
            self.get_logger().info(
                f'[AVOID] phase={self.avoid_phase} reason={reason} '
                f'front_min={front:.3f} front_valid={self.front_valid_count} '
                f'front_latched={int(self.front_stop_latched)} clear_count={self.avoid_clear_count} '
                f'left_min={left_min} right_min={right_min} side={self.avoid_turn_sign:+.0f} '
                f'v={cmd.linear.x:.3f} w={cmd.angular.z:.3f}'
            )
            self.avoid_log_key, self.avoid_last_log = key, now

    def publish_command(self, cmd: Twist):
        # Apply bounds to every path, including escape modes.
        if not math.isfinite(cmd.linear.x) or not math.isfinite(cmd.angular.z):
            cmd = Twist()
        cmd.linear.x = self.clamp(cmd.linear.x, 0.0, self.forward_speed)
        cmd.angular.z = self.clamp(cmd.angular.z, -self.max_yaw_rate, self.max_yaw_rate)
        if self.front_stop_latched:
            cmd.linear.x = 0.0
        self.cmd_pub.publish(cmd)

    def guard_stop(self, reason):
        self.front_stop_latched = True
        self.front_clear_count = 0
        self.avoid_clear_count = 0
        # A data gap stops motion without erasing the selected passing side.
        self.prev_cmd_w = 0.0
        self.prev_target_angle = 0.0
        self.mode = 'normal'
        self.hazard_escape_active = False
        self.blocked_counter = 0
        self.last_scan_stamp = None
        self.cmd_pub.publish(Twist())
        if reason != self.last_guard_reason:
            self.get_logger().warn(f'[STOP] reason={reason}; fresh finite front returns required')
        self.last_guard_reason = reason

    def check_scan_timeout(self):
        if (self.last_scan_received is None or
                time.monotonic() - self.last_scan_received > self.scan_timeout_sec):
            self.guard_stop('scan_timeout')

    # ---------------- Escape logic ----------------
    def start_escape(self, sector_clearance: List[float]):
        self.mode = 'escape'
        self.blocked_counter = 0
        self.escape_accum_rad = 0.0

        left_clear = sum(sector_clearance[self.center_sector + 1:]) if self.center_sector + 1 < self.num_sectors else 0.0
        right_clear = sum(sector_clearance[:self.center_sector]) if self.center_sector > 0 else 0.0

        self.escape_turn_dir = 1.0 if left_clear >= right_clear else -1.0

        self.get_logger().warn(
            f'Entering ESCAPE mode: turning {"left" if self.escape_turn_dir > 0 else "right"} '
            f'for {self.escape_turn_angle_deg:.1f} deg'
        )

    def run_escape(self, dt: float) -> Twist:
        cmd = Twist()

        remaining = self.escape_target_rad - self.escape_accum_rad
        if remaining <= 0.0:
            self.finish_escape()
            return Twist()

        turn_mag = self.escape_turn_rate
        if remaining < math.radians(40.0):
            turn_mag = max(0.35, self.escape_turn_rate * 0.55)

        cmd.linear.x = 0.0
        turn_mag = min(abs(turn_mag), self.max_yaw_rate, abs(self.escape_turn_rate))
        cmd.angular.z = self.escape_turn_dir * turn_mag

        self.escape_accum_rad += abs(cmd.angular.z) * dt
        self.prev_cmd_w = cmd.angular.z
        self.last_turn_sign = 1.0 if cmd.angular.z > 0.0 else -1.0

        if self.escape_accum_rad >= self.escape_target_rad:
            self.finish_escape()

        return cmd

    def finish_escape(self):
        self.mode = 'normal'
        self.escape_accum_rad = 0.0
        self.escape_holdoff_counter = self.escape_holdoff_cycles
        self.prev_target_angle = 0.0
        self.prev_cmd_w = 0.0
        self.get_logger().warn('Escape complete, returning to NORMAL mode.')

    # ---------------- Hazard escape logic ----------------
    def start_hazard_escape(self, sector_clearance: List[float]):
        self.hazard_escape_active = True
        self.hazard_escape_accum_rad = 0.0

        left_clear = sum(sector_clearance[self.center_sector + 1:]) if self.center_sector + 1 < self.num_sectors else 0.0
        right_clear = sum(sector_clearance[:self.center_sector]) if self.center_sector > 0 else 0.0

        self.hazard_escape_turn_dir = 1.0 if left_clear >= right_clear else -1.0

        self.get_logger().warn(
            f'Hazard escape: turning {"left" if self.hazard_escape_turn_dir > 0 else "right"} '
            f'for {self.hazard_escape_angle_deg:.1f} deg'
        )

    def run_hazard_escape(self, dt: float) -> Twist:
        cmd = Twist()

        remaining = self.hazard_escape_target_rad - self.hazard_escape_accum_rad
        if remaining <= 0.0:
            self.hazard_escape_active = False
            return Twist()

        turn_mag = self.hazard_escape_turn_rate
        if remaining < math.radians(35.0):
            turn_mag = max(0.40, self.hazard_escape_turn_rate * 0.60)

        cmd.linear.x = 0.0
        turn_mag = min(abs(turn_mag), self.max_yaw_rate, abs(self.hazard_escape_turn_rate))
        cmd.angular.z = self.hazard_escape_turn_dir * turn_mag

        self.hazard_escape_accum_rad += abs(cmd.angular.z) * dt
        self.prev_cmd_w = cmd.angular.z
        self.last_turn_sign = 1.0 if cmd.angular.z > 0.0 else -1.0

        if self.hazard_escape_accum_rad >= self.hazard_escape_target_rad:
            self.hazard_escape_active = False

        return cmd

    # ---------------- Helpers ----------------
    def compute_dt(self, msg: LaserScan) -> float:
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_scan_stamp is None:
            self.last_scan_stamp = current_time
            return 0.1

        dt = current_time - self.last_scan_stamp
        self.last_scan_stamp = current_time

        if dt <= 0.0 or dt > 1.0:
            return 0.1
        return dt

    def compute_fusion_urgency(self) -> float:
        penalty_peak = max(self.fusion_penalties) if self.fusion_penalties else 0.0
        penalty_term = self.clamp(penalty_peak / 3.0, 0.0, 1.0)

        if self.fusion_speed_limit >= self.fusion_priority_speed_limit:
            speed_term = 0.0
        else:
            denom = max(1e-6, self.fusion_priority_speed_limit)
            speed_term = self.clamp((self.fusion_priority_speed_limit - self.fusion_speed_limit) / denom, 0.0, 1.0)

        return self.clamp(max(speed_term, penalty_term), 0.0, 1.0)

    def build_histogram(self, scan: LaserScan) -> Tuple[List[float], List[float], float]:
        hist = [0.0] * self.num_sectors
        # Histogram valleys are candidate directions, not verified free space.
        # Front finite-return guard above blocks motion when front data is absent.
        sector_clearance = [self.max_range_clip] * self.num_sectors

        half_fov = self.total_hist_fov_rad / 2.0
        inflate = self.robot_radius + self.safety_margin

        front_min = float('inf')

        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment

            if angle < -half_fov or angle > half_fov:
                continue

            if (r is None or not math.isfinite(r) or r <= 0.0 or
                    not scan.range_min <= r <= scan.range_max):
                continue
            r = min(r, self.max_range_clip)

            if abs(angle) <= math.radians(15.0):
                front_min = min(front_min, r)

            sector = self.angle_to_sector(angle)
            if sector < 0 or sector >= self.num_sectors:
                continue

            sector_clearance[sector] = min(sector_clearance[sector], r)

            if r < self.obstacle_influence_dist:
                mag = ((self.obstacle_influence_dist - r) / self.obstacle_influence_dist) ** 2 * 3.0
            else:
                mag = 0.0

            hist[sector] += mag

            if r > 1e-3 and r > inflate:
                widen_angle = math.asin(min(1.0, inflate / r))
            else:
                widen_angle = math.pi / 2.0

            widen_bins = max(0, int(math.ceil(widen_angle / self.sector_width_rad)))

            for k in range(1, widen_bins + 1):
                decay = max(0.2, 1.0 - (k / (widen_bins + 1)))
                if sector - k >= 0:
                    hist[sector - k] += mag * decay
                    sector_clearance[sector - k] = min(sector_clearance[sector - k], r)
                if sector + k < self.num_sectors:
                    hist[sector + k] += mag * decay
                    sector_clearance[sector + k] = min(sector_clearance[sector + k], r)

        if math.isinf(front_min):
            front_min = self.max_range_clip

        return hist, sector_clearance, front_min

    def find_open_valleys(self, open_mask: List[bool]) -> List[Tuple[int, int]]:
        valleys = []
        start = None

        for i, is_open in enumerate(open_mask):
            if is_open and start is None:
                start = i
            elif not is_open and start is not None:
                if i - start >= self.min_open_sectors:
                    valleys.append((start, i - 1))
                start = None

        if start is not None and (len(open_mask) - start) >= self.min_open_sectors:
            valleys.append((start, len(open_mask) - 1))

        return valleys

    def choose_best_sector(
        self,
        valleys: List[Tuple[int, int]],
        sector_clearance: List[float],
        front_min: float,
        fusion_urgency: float
    ) -> int:
        if not valleys:
            left_clear = sum(sector_clearance[self.center_sector + 1:]) if self.center_sector + 1 < self.num_sectors else 0.0
            right_clear = sum(sector_clearance[:self.center_sector]) if self.center_sector > 0 else 0.0
            return self.num_sectors - 1 if left_clear >= right_clear else 0

        best_sector = self.center_sector
        best_score = -1e9

        prev_sign = 1.0 if self.prev_target_angle >= 0.0 else -1.0

        forward_weight = max(0.4, self.forward_weight - self.fusion_forward_suppression * fusion_urgency)
        smooth_weight = max(0.2, self.smooth_weight - self.fusion_smooth_suppression * fusion_urgency)
        clearance_weight = self.clearance_weight + self.fusion_clearance_boost * fusion_urgency

        for start, end in valleys:
            valley_center = (start + end) // 2
            nearest_forward = self.clamp_int(self.center_sector, start, end)

            best_clear_idx = start
            best_clear_val = -1.0
            for s in range(start, end + 1):
                if sector_clearance[s] > best_clear_val:
                    best_clear_val = sector_clearance[s]
                    best_clear_idx = s

            candidates = {valley_center, nearest_forward, best_clear_idx}

            if fusion_urgency > 0.45:
                for s in range(start, end + 1):
                    if sector_clearance[s] >= (best_clear_val - 0.05):
                        candidates.add(s)

            for s in candidates:
                angle = self.sector_to_angle(s)

                forward_score = -forward_weight * abs(angle)
                smooth_score = -smooth_weight * abs(angle - self.prev_target_angle)
                clearance_score = clearance_weight * sector_clearance[s]

                fusion_penalty = self.fusion_penalties[s] if 0 <= s < len(self.fusion_penalties) else 0.0
                penalty_score = -self.fusion_penalty_weight * (1.0 + fusion_urgency) * fusion_penalty

                sign = 1.0 if angle >= 0.0 else -1.0
                commit_score = self.commitment_weight if sign == prev_sign else 0.0

                escape_bonus = 0.0
                if front_min < self.hard_turn_dist or fusion_urgency > 0.40:
                    escape_bonus = (0.8 + 0.8 * fusion_urgency) * abs(angle)

                score = (
                    forward_score +
                    smooth_score +
                    clearance_score +
                    penalty_score +
                    commit_score +
                    escape_bonus
                )

                if score > best_score:
                    best_score = score
                    best_sector = s

        return best_sector

    def angle_to_sector(self, angle: float) -> int:
        half_fov = self.total_hist_fov_rad / 2.0
        normalized = (angle + half_fov) / self.total_hist_fov_rad
        sector = int(normalized * self.num_sectors)
        return self.clamp_int(sector, 0, self.num_sectors - 1)

    def sector_to_angle(self, sector: int) -> float:
        half_fov = self.total_hist_fov_rad / 2.0
        return -half_fov + (sector + 0.5) * self.sector_width_rad

    @staticmethod
    def linear_scale(x: float, x0: float, x1: float) -> float:
        if x <= x0:
            return 0.0
        if x >= x1:
            return 1.0
        return (x - x0) / (x1 - x0)

    @staticmethod
    def clamp(x: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, x))

    @staticmethod
    def clamp_int(x: int, lo: int, hi: int) -> int:
        return max(lo, min(hi, x))

def main(args=None):
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = VFHObstacleAvoidance()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if rclpy.ok():
                node.cmd_pub.publish(Twist())
        finally:
            try:
                node.destroy_node()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()


if __name__ == '__main__':
    main()
