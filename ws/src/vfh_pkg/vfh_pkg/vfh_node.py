#!/usr/bin/env python3
import math
import time
from collections import deque
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Bool, Float32


def inspect_scan(msg, now_sec, max_age, half_fov):
    """Finite returns are evidence; no-return bins are unknown, never clear.

    This checks message structure/freshness, not physical frame alignment or
    whether a projected scan represents the full robot footprint.
    """
    if not msg.ranges:
        return False, 'empty_scan'
    values = (msg.angle_min, msg.angle_max, msg.angle_increment,
              msg.range_min, msg.range_max)
    if not all(math.isfinite(x) for x in values):
        return False, 'nonfinite_metadata'
    if msg.angle_increment <= 0 or msg.range_min < 0 or msg.range_max <= msg.range_min:
        return False, 'bad_metadata'
    end = msg.angle_min + (len(msg.ranges) - 1) * msg.angle_increment
    if abs(end - msg.angle_max) > max(1e-5, 1.5 * msg.angle_increment):
        return False, 'range_count_mismatch'
    if msg.angle_min > -half_fov + msg.angle_increment or end < half_fov - msg.angle_increment:
        return False, 'insufficient_fov'
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if stamp <= 0 or not -0.1 <= now_sec - stamp <= max_age:
        return False, 'stale_or_bad_stamp'
    front_count = 0
    for i, value in enumerate(msg.ranges):
        angle = msg.angle_min + i * msg.angle_increment
        if (abs(angle) <= math.radians(15.0) and math.isfinite(value)
                and value > 0 and msg.range_min <= value <= msg.range_max):
            front_count += 1
    if front_count == 0:
        return False, 'no_finite_front_returns'
    return True, 'ok'


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

        # LiDAR-only validation profile. Existing escape parameters are accepted
        # for launch compatibility, but command-integrated escape is disabled.
        self.declare_parameter('use_fusion', False)
        self.use_fusion = bool(self.get_parameter('use_fusion').value)
        self.declare_parameter('scan_timeout_sec', 0.25)
        self.declare_parameter('max_scan_age_sec', 0.5)
        self.declare_parameter('obstacle_hold_sec', 0.5)
        self.scan_timeout_sec = float(self.get_parameter('scan_timeout_sec').value)
        self.max_scan_age_sec = float(self.get_parameter('max_scan_age_sec').value)
        self.obstacle_hold_sec = float(self.get_parameter('obstacle_hold_sec').value)
        numeric = [self.sector_angle_deg, self.front_fov_deg, self.robot_radius,
                   self.safety_margin, self.max_range_clip, self.obstacle_influence_dist,
                   self.hist_threshold, self.forward_speed, self.min_speed,
                   self.max_yaw_rate, self.steer_gain, self.front_stop_dist,
                   self.front_slowdown_dist, self.ema_alpha, self.scan_timeout_sec,
                   self.max_scan_age_sec, self.obstacle_hold_sec]
        if (not all(math.isfinite(x) for x in numeric)
                or self.sector_angle_deg <= 0 or not 30 <= self.front_fov_deg <= 360
                or self.robot_radius < 0 or self.safety_margin < 0
                or self.max_range_clip <= 0 or self.obstacle_influence_dist <= 0
                or self.hist_threshold <= 0 or self.forward_speed < 0
                or self.max_yaw_rate <= 0 or self.steer_gain < 0
                or self.front_stop_dist < 0
                or self.front_slowdown_dist <= self.front_stop_dist
                or not 0 < self.ema_alpha <= 1
                or self.scan_timeout_sec <= 0 or self.max_scan_age_sec <= 0
                or not 0 <= self.obstacle_hold_sec <= 1.0):
            raise ValueError('Invalid VFH/scan guard configuration')
        self.last_scan_receipt = None
        self.scan_healthy = False
        self.scan_fault = 'waiting_for_scan'
        self.scan_history = deque(maxlen=128)
        self.last_message_stamp = None

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

        self.scan_watchdog = self.create_timer(0.05, self.check_scan_timeout)
        self.get_logger().info('VFH obstacle avoidance with conservative scan guard started.')
        self.get_logger().warn('min_speed is compatibility-only; no positive speed floor. '
                               'Open-loop escape rotations are disabled.')
        self.get_logger().info(f'scan_topic={scan_topic}, cmd_topic={cmd_topic}')
        self.get_logger().info(
            f'front_fov_deg={self.front_fov_deg:.1f}, sectors={self.num_sectors}, mode={self.mode}'
        )

    # ---------------- Fusion callbacks ----------------
    def on_fusion_penalties(self, msg: Float32MultiArray):
        if not self.use_fusion:
            return
        if len(msg.data) == self.num_sectors:
            data = [float(x) for x in msg.data]
            if all(math.isfinite(x) and x >= 0 for x in data):
                self.fusion_penalties = data
            else:
                self.fusion_critical_stop = True
                self.publish_stop('invalid_fusion_penalties')
        else:
            self.fusion_critical_stop = True
            self.publish_stop('bad_fusion_penalty_count')

    def on_fusion_stop(self, msg: Bool):
        if not self.use_fusion:
            return
        self.fusion_critical_stop = bool(msg.data)
        if self.fusion_critical_stop:
            self.publish_stop('fusion_critical_stop')

    def on_fusion_speed(self, msg: Float32):
        if not self.use_fusion:
            return
        value = float(msg.data)
        self.fusion_speed_limit = self.clamp(value, 0.0, 1.0) if math.isfinite(value) else 0.0
        if self.fusion_speed_limit == 0:
            self.publish_stop('fusion_speed_zero')

    # ---------------- Main scan callback ----------------
    def scan_callback(self, msg: LaserScan):
        now = time.monotonic()
        self.last_scan_receipt = now
        valid, reason = inspect_scan(msg, self.get_clock().now().nanoseconds * 1e-9,
                                     self.max_scan_age_sec, self.total_hist_fov_rad / 2)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if valid and self.last_message_stamp is not None and stamp <= self.last_message_stamp:
            valid, reason = False, 'nonadvancing_scan_stamp'
        self.scan_healthy = valid
        if not valid:
            self.scan_fault = reason
            self.publish_stop(reason)
            return
        self.last_message_stamp = stamp
        self.scan_fault = 'ok'
        self._last_stop_reason = None

        hist, sector_clearance, front_min = self.build_histogram(msg)
        observed_front_min = front_min
        # Only retain hazards: history never creates current visibility or
        # converts unknown bins into free space. No odometry-based deskew here.
        self.scan_history.append((now, hist[:], sector_clearance[:], front_min))
        while self.scan_history and now - self.scan_history[0][0] > self.obstacle_hold_sec:
            self.scan_history.popleft()
        for _, old_hist, old_clearance, old_front in self.scan_history:
            hist = [max(a, b) for a, b in zip(hist, old_hist)]
            sector_clearance = [min(a, b) for a, b in zip(sector_clearance, old_clearance)]
            front_min = min(front_min, old_front)

        if self.fusion_critical_stop or self.fusion_speed_limit <= 0:
            self.publish_stop('fusion_stop')
            return
        if front_min <= self.front_stop_dist:
            self.publish_stop(f'front_stop front_min={front_min:.2f}')
            return
        hist = [h + p for h, p in zip(hist, self.fusion_penalties)]
        open_mask = [known and h < self.hist_threshold
                     for known, h in zip(self.current_observed_sectors, hist)]
        valleys = self.find_open_valleys(open_mask)
        open_fraction = sum(open_mask) / len(open_mask)
        if not valleys:
            self.publish_stop('no_observed_open_valley')
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

        if front_min <= self.front_stop_dist and fusion_urgency < 0.85:
            cmd.linear.x = 0.0
        else:
            speed_scale = self.linear_scale(front_min, self.front_stop_dist, self.front_slowdown_dist)
            turn_scale = max(0.25, 1.0 - 0.65 * min(1.0, abs(cmd.angular.z) / self.max_yaw_rate))
            cmd.linear.x = self.forward_speed * speed_scale * turn_scale

            cmd.linear.x *= self.fusion_speed_limit

            if fusion_urgency > 0.0:
                cmd.linear.x *= max(0.25, 1.0 - 0.45 * fusion_urgency)

            # Never raise a slowdown/limit result back to a positive floor.
            cmd.linear.x = self.clamp(cmd.linear.x, 0.0, self.forward_speed)

        if (front_min < self.hard_turn_dist or fusion_urgency > 0.65) and abs(cmd.angular.z) > 0.7 * self.max_yaw_rate:
            cmd.linear.x = min(cmd.linear.x, 0.05)

        self.cmd_pub.publish(cmd)

        self.prev_target_angle = target_angle
        self.prev_cmd_w = cmd.angular.z
        if abs(cmd.angular.z) > 1e-3:
            self.last_turn_sign = 1.0 if cmd.angular.z > 0.0 else -1.0

        if self.debug_log:
            self.get_logger().info(
                f'[NORMAL] front_min={front_min:.2f} observed_front={observed_front_min:.2f} '
                f'open_frac={open_fraction:.2f} observed_sectors={sum(self.current_observed_sectors)}/{self.num_sectors} '
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

    def publish_stop(self, reason):
        self.prev_cmd_w = 0.0
        self.prev_target_angle = 0.0
        self.mode = 'normal'
        self.cmd_pub.publish(Twist())
        if reason != getattr(self, '_last_stop_reason', None):
            self.get_logger().warn(f'[STOP] {reason}')
        self._last_stop_reason = reason

    def check_scan_timeout(self):
        if (self.last_scan_receipt is None
                or time.monotonic() - self.last_scan_receipt > self.scan_timeout_sec):
            self.scan_healthy = False
            self.scan_history.clear()
            self.publish_stop('scan_timeout')
        elif not self.scan_healthy:
            self.publish_stop(self.scan_fault)

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
        sector_clearance = [self.max_range_clip] * self.num_sectors
        self.current_observed_sectors = [False] * self.num_sectors

        half_fov = self.total_hist_fov_rad / 2.0
        inflate = self.robot_radius + self.safety_margin

        front_min = float('inf')

        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment

            if angle < -half_fov or angle > half_fov:
                continue

            if (not math.isfinite(r) or r <= 0.0
                    or not scan.range_min <= r <= scan.range_max):
                continue
            r = min(r, self.max_range_clip)

            if abs(angle) <= math.radians(15.0):
                front_min = min(front_min, r)

            sector = self.angle_to_sector(angle)
            if sector < 0 or sector >= self.num_sectors:
                continue

            self.current_observed_sectors[sector] = True
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
    rclpy.init(args=args)
    node = VFHObstacleAvoidance()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
