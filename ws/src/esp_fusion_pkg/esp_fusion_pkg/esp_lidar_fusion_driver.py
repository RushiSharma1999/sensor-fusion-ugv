#!/usr/bin/env python3
import math
import time
import serial
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class VFHWithEspFusion(Node):
    def __init__(self):
        super().__init__('vfh_with_esp_fusion')

        # ----------------------------
        # Topics / serial
        # ----------------------------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('debug_log', True)

        scan_topic = self.get_parameter('scan_topic').value
        cmd_topic = self.get_parameter('cmd_topic').value
        serial_port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('baud').value)
        self.debug_log = bool(self.get_parameter('debug_log').value)

        # ----------------------------
        # Core VFH behavior
        # ----------------------------
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

        # ----------------------------
        # Escape / recovery
        # ----------------------------
        self.declare_parameter('escape_trigger_cycles', 6)
        self.declare_parameter('blocked_open_fraction', 0.10)
        self.declare_parameter('escape_turn_angle_deg', 180.0)
        self.declare_parameter('escape_turn_rate', 0.8)
        self.declare_parameter('escape_clearance_dist', 0.55)
        self.declare_parameter('escape_holdoff_cycles', 10)

        # ----------------------------
        # ESP fusion thresholds
        # ----------------------------
        self.declare_parameter('lidar_confirm_trigger_cm', 80.0)
        self.declare_parameter('confirm_window_s', 0.35)

        self.declare_parameter('us_front_stop_cm', 28.0)
        self.declare_parameter('us_front_slow_cm', 90.0)

        self.declare_parameter('us_side_near_cm', 18.0)
        self.declare_parameter('us_side_alert_cm', 55.0)

        self.declare_parameter('us_back_near_cm', 20.0)
        self.declare_parameter('us_back_alert_cm', 45.0)

        self.declare_parameter('front_far_stop_cm', 35.0)
        self.declare_parameter('front_far_slow_cm', 120.0)
        self.declare_parameter('back_far_alert_cm', 60.0)

        self.declare_parameter('recovery_time_s', 0.60)
        self.declare_parameter('recovery_linear_v', 0.08)
        self.declare_parameter('recovery_turn_w', 0.95)

        # VFH params
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

        self.escape_trigger_cycles = int(self.get_parameter('escape_trigger_cycles').value)
        self.blocked_open_fraction = float(self.get_parameter('blocked_open_fraction').value)
        self.escape_turn_angle_deg = float(self.get_parameter('escape_turn_angle_deg').value)
        self.escape_turn_rate = float(self.get_parameter('escape_turn_rate').value)
        self.escape_clearance_dist = float(self.get_parameter('escape_clearance_dist').value)
        self.escape_holdoff_cycles = int(self.get_parameter('escape_holdoff_cycles').value)

        # ESP fusion params
        self.lidar_confirm_trigger_cm = float(self.get_parameter('lidar_confirm_trigger_cm').value)
        self.confirm_window_s = float(self.get_parameter('confirm_window_s').value)

        self.us_front_stop_cm = float(self.get_parameter('us_front_stop_cm').value)
        self.us_front_slow_cm = float(self.get_parameter('us_front_slow_cm').value)

        self.us_side_near_cm = float(self.get_parameter('us_side_near_cm').value)
        self.us_side_alert_cm = float(self.get_parameter('us_side_alert_cm').value)

        self.us_back_near_cm = float(self.get_parameter('us_back_near_cm').value)
        self.us_back_alert_cm = float(self.get_parameter('us_back_alert_cm').value)

        self.front_far_stop_cm = float(self.get_parameter('front_far_stop_cm').value)
        self.front_far_slow_cm = float(self.get_parameter('front_far_slow_cm').value)
        self.back_far_alert_cm = float(self.get_parameter('back_far_alert_cm').value)

        self.recovery_time_s = float(self.get_parameter('recovery_time_s').value)
        self.recovery_linear_v = float(self.get_parameter('recovery_linear_v').value)
        self.recovery_turn_w = float(self.get_parameter('recovery_turn_w').value)

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

        # front/back confirmation timers
        self.confirm_started = {
            'front': None,
            'back': None,
        }

        # latest esp state
        self.latest_esp = None

        # serial
        self.ser = serial.Serial(serial_port, baud, timeout=0.01)
        time.sleep(2.0)
        self.ser.write(b"PI_READY\n")

        # ROS I/O
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        self.get_logger().info('VFH + ESP fusion node started.')
        self.get_logger().info(f'scan_topic={scan_topic}, cmd_topic={cmd_topic}, serial={serial_port}@{baud}')

    # =========================================================
    # Serial / ESP
    # =========================================================
    def parse_state_packet(self, line):
        if not line.startswith("STATE|"):
            return None

        fields = {}
        for part in line.strip().split("|")[1:]:
            if ":" not in part:
                continue
            key, value = part.split(":", 1)
            fields[key] = value
        return fields

    def get_bool(self, fields, key):
        return fields.get(key, "0") == "1"

    def get_float(self, fields, key, default=999.0):
        try:
            return float(fields.get(key, default))
        except Exception:
            return default

    def extract_esp_state(self, fields):
        # assumes only front/back confirms remain
        return {
            "emg": self.get_bool(fields, "EMG"),
            "far_front": self.get_float(fields, "FAR_FRONT"),
            "far_back": self.get_float(fields, "FAR_BACK"),
            "us_front": self.get_float(fields, "US_FRONT"),
            "us_left": self.get_float(fields, "US_LEFT"),
            "us_right": self.get_float(fields, "US_RIGHT"),
            "us_back": self.get_float(fields, "US_BACK"),
            "confirm_front": self.get_bool(fields, "CONF_FRONT"),
            "confirm_back": self.get_bool(fields, "CONF_BACK"),
        }

    def read_serial_state(self):
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                if self.debug_log:
                    self.get_logger().info(f"ESP RAW: {line}")

                parsed = self.parse_state_packet(line)
                if parsed:
                    self.latest_esp = self.extract_esp_state(parsed)
        except Exception as e:
            self.get_logger().error(f"Serial read failed: {e}")

    # =========================================================
    # Helpers
    # =========================================================
    @staticmethod
    def clamp(x: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, x))

    @staticmethod
    def clamp_int(x: int, lo: int, hi: int) -> int:
        return max(lo, min(hi, x))

    @staticmethod
    def linear_scale(x: float, x0: float, x1: float) -> float:
        if x <= x0:
            return 0.0
        if x >= x1:
            return 1.0
        return (x - x0) / (x1 - x0)

    @staticmethod
    def ramp_near(distance_cm, near_cm, far_cm):
        if distance_cm <= near_cm:
            return 1.0
        if distance_cm >= far_cm:
            return 0.0
        return (far_cm - distance_cm) / (far_cm - near_cm)

    def confidence_weight(self, state):
        if state == "confirmed":
            return 1.0
        if state == "lidar_only":
            return 0.85
        if state == "waiting":
            return 0.65
        return 0.0

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

    def update_confirm_state(self, sector, lidar_distance_cm, esp_confirm):
        now = time.monotonic()

        if lidar_distance_cm <= self.lidar_confirm_trigger_cm:
            if self.confirm_started[sector] is None:
                self.confirm_started[sector] = now

            elapsed = now - self.confirm_started[sector]

            if esp_confirm and elapsed <= self.confirm_window_s:
                return "confirmed"
            if elapsed > self.confirm_window_s:
                return "lidar_only"
            return "waiting"

        self.confirm_started[sector] = None
        return "clear"

    # =========================================================
    # VFH
    # =========================================================
    def scan_callback(self, msg: LaserScan):
        self.read_serial_state()

        if not msg.ranges:
            return
        if self.latest_esp is None:
            return

        dt = self.compute_dt(msg)

        hist, sector_clearance, front_min = self.build_histogram(msg)
        open_mask = [h < self.hist_threshold for h in hist]
        valleys = self.find_open_valleys(open_mask)

        open_count = sum(1 for x in open_mask if x)
        open_fraction = open_count / max(1, len(open_mask))

        # side / rear / wide sector mins from lidar for fusion
        lidar_sector = self.compute_lidar_sector_mins(msg)

        # front/back confirmation states
        front_state = self.update_confirm_state(
            'front',
            lidar_sector['front_cm'],
            self.latest_esp['confirm_front']
        )
        back_state = self.update_confirm_state(
            'back',
            lidar_sector['back_cm'],
            self.latest_esp['confirm_back']
        )

        fused = self.compute_fused_risks(front_state, back_state, lidar_sector, self.latest_esp)

        clogged = (open_fraction < self.blocked_open_fraction) and (front_min < self.escape_clearance_dist)

        # emergency from ESP
        if self.latest_esp['emg']:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            if self.debug_log:
                self.get_logger().warn('ESP emergency lockout')
            return

        if self.escape_holdoff_counter > 0:
            self.escape_holdoff_counter -= 1

        if self.mode == 'normal' and self.escape_holdoff_counter == 0:
            if clogged or fused['front_risk'] > 0.92:
                self.blocked_counter += 1
            else:
                self.blocked_counter = 0

            if self.blocked_counter >= self.escape_trigger_cycles:
                self.start_escape(sector_clearance, fused)
        else:
            if not clogged:
                self.blocked_counter = 0

        if self.mode == 'escape':
            cmd = self.run_escape(dt, fused)
            self.cmd_pub.publish(cmd)

            if self.debug_log:
                self.get_logger().info(
                    f'[ESCAPE] turned={self.escape_accum_rad:.2f}/{self.escape_target_rad:.2f} '
                    f'v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}'
                )
            return

        target_sector = self.choose_best_sector(valleys, sector_clearance, front_min)
        target_angle = self.sector_to_angle(target_sector)

        cmd = self.build_normal_command(target_angle, front_min, fused)

        self.cmd_pub.publish(cmd)

        self.prev_target_angle = target_angle
        self.prev_cmd_w = cmd.angular.z
        if abs(cmd.angular.z) > 1e-3:
            self.last_turn_sign = 1.0 if cmd.angular.z > 0.0 else -1.0

        if self.debug_log:
            self.get_logger().info(
                f'[NORMAL] '
                f'front_min={front_min:.2f} '
                f'open_frac={open_fraction:.2f} '
                f'target_sector={target_sector} '
                f'target_angle={target_angle:.2f} '
                f'v={cmd.linear.x:.2f} '
                f'w={cmd.angular.z:.2f} '
                f'front_state={front_state} '
                f'back_state={back_state} '
                f'risks='
                f'F:{fused["front_risk"]:.2f} '
                f'L:{fused["left_risk"]:.2f} '
                f'R:{fused["right_risk"]:.2f} '
                f'B:{fused["back_risk"]:.2f}'
            )

    def build_histogram(self, scan: LaserScan) -> Tuple[List[float], List[float], float]:
        hist = [0.0] * self.num_sectors
        sector_clearance = [self.max_range_clip] * self.num_sectors

        half_fov = self.total_hist_fov_rad / 2.0
        inflate = self.robot_radius + self.safety_margin

        front_min = float('inf')

        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment

            if angle < -half_fov or angle > half_fov:
                continue

            if r is None or math.isnan(r) or r <= 0.0:
                continue
            if math.isinf(r):
                r = self.max_range_clip
            else:
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
        front_min: float
    ) -> int:
        if not valleys:
            left_clear = sum(sector_clearance[self.center_sector + 1:]) if self.center_sector + 1 < self.num_sectors else 0.0
            right_clear = sum(sector_clearance[:self.center_sector]) if self.center_sector > 0 else 0.0
            return self.num_sectors - 1 if left_clear >= right_clear else 0

        best_sector = self.center_sector
        best_score = -1e9
        prev_sign = 1.0 if self.prev_target_angle >= 0.0 else -1.0

        for start, end in valleys:
            valley_center = (start + end) // 2
            nearest_forward = self.clamp_int(self.center_sector, start, end)

            candidates = {valley_center, nearest_forward}

            best_clear_idx = start
            best_clear_val = -1.0
            for s in range(start, end + 1):
                if sector_clearance[s] > best_clear_val:
                    best_clear_val = sector_clearance[s]
                    best_clear_idx = s
            candidates.add(best_clear_idx)

            for s in candidates:
                angle = self.sector_to_angle(s)

                forward_score = -self.forward_weight * abs(angle)
                smooth_score = -self.smooth_weight * abs(angle - self.prev_target_angle)
                clearance_score = self.clearance_weight * sector_clearance[s]

                sign = 1.0 if angle >= 0.0 else -1.0
                commit_score = self.commitment_weight if sign == prev_sign else 0.0

                escape_bonus = 0.0
                if front_min < self.hard_turn_dist:
                    escape_bonus = 0.8 * abs(angle)

                score = forward_score + smooth_score + clearance_score + commit_score + escape_bonus

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

    # =========================================================
    # Lidar sector mins for ESP fusion
    # =========================================================
    def compute_lidar_sector_mins(self, scan: LaserScan):
        result = {
            'front_cm': 999.0,
            'left_cm': 999.0,
            'right_cm': 999.0,
            'back_cm': 999.0,
        }

        for i, r in enumerate(scan.ranges):
            if r is None or math.isnan(r) or r <= 0.0 or math.isinf(r):
                continue

            angle_rad = scan.angle_min + i * scan.angle_increment
            angle_deg = math.degrees(angle_rad) % 360.0
            d_cm = r * 100.0

            if angle_deg >= 330.0 or angle_deg <= 30.0:
                result['front_cm'] = min(result['front_cm'], d_cm)
            elif 30.0 < angle_deg <= 150.0:
                result['left_cm'] = min(result['left_cm'], d_cm)
            elif 150.0 < angle_deg <= 210.0:
                result['back_cm'] = min(result['back_cm'], d_cm)
            else:
                result['right_cm'] = min(result['right_cm'], d_cm)

        return result

    def compute_fused_risks(self, front_state, back_state, lidar_sector, esp):
        front_lidar_risk = self.ramp_near(
            lidar_sector['front_cm'],
            self.front_far_stop_cm,
            self.front_far_slow_cm
        ) * self.confidence_weight(front_state)

        back_lidar_risk = self.ramp_near(
            lidar_sector['back_cm'],
            self.front_far_stop_cm,
            self.back_far_alert_cm
        ) * self.confidence_weight(back_state)

        left_lidar_risk = self.ramp_near(
            lidar_sector['left_cm'],
            self.front_far_stop_cm,
            100.0
        )

        right_lidar_risk = self.ramp_near(
            lidar_sector['right_cm'],
            self.front_far_stop_cm,
            100.0
        )

        us_front_risk = self.ramp_near(esp['us_front'], self.us_front_stop_cm, self.us_front_slow_cm)
        us_left_risk = self.ramp_near(esp['us_left'], self.us_side_near_cm, self.us_side_alert_cm)
        us_right_risk = self.ramp_near(esp['us_right'], self.us_side_near_cm, self.us_side_alert_cm)
        us_back_risk = self.ramp_near(esp['us_back'], self.us_back_near_cm, self.us_back_alert_cm)

        front_risk = max(front_lidar_risk, 0.90 * us_front_risk)
        back_risk = max(back_lidar_risk, 0.85 * us_back_risk)

        if esp['confirm_front']:
            front_risk = max(
                front_risk,
                0.95 * self.ramp_near(
                    esp['far_front'],
                    self.front_far_stop_cm,
                    self.front_far_slow_cm
                )
            )

        if esp['confirm_back']:
            back_risk = max(
                back_risk,
                0.85 * self.ramp_near(
                    esp['far_back'],
                    self.front_far_stop_cm,
                    self.back_far_alert_cm
                )
            )

        left_risk = max(0.80 * left_lidar_risk, us_left_risk)
        right_risk = max(0.80 * right_lidar_risk, us_right_risk)

        return {
            'front_risk': self.clamp(front_risk, 0.0, 1.0),
            'left_risk': self.clamp(left_risk, 0.0, 1.0),
            'right_risk': self.clamp(right_risk, 0.0, 1.0),
            'back_risk': self.clamp(back_risk, 0.0, 1.0),
        }

    # =========================================================
    # Command generation
    # =========================================================
    def build_normal_command(self, target_angle, front_min, fused):
        cmd = Twist()

        raw_w = self.steer_gain * target_angle

        # side ultrasonic / risk bias
        side_bias = fused['left_risk'] - fused['right_risk']

        if fused['front_risk'] > 0.0:
            if fused['left_risk'] < fused['right_risk']:
                raw_w += 0.60 * fused['front_risk']
            else:
                raw_w -= 0.60 * fused['front_risk']

        raw_w += 0.55 * side_bias
        raw_w = self.clamp(raw_w, -self.max_yaw_rate, self.max_yaw_rate)

        if front_min < self.hard_turn_dist:
            turn_dir = 1.0 if target_angle >= 0.0 else -1.0
            if abs(target_angle) < math.radians(8.0):
                turn_dir = self.last_turn_sign
            raw_w = turn_dir * max(abs(raw_w), 0.65 * self.max_yaw_rate)

        cmd.angular.z = (1.0 - self.ema_alpha) * self.prev_cmd_w + self.ema_alpha * raw_w
        cmd.angular.z = self.clamp(cmd.angular.z, -self.max_yaw_rate, self.max_yaw_rate)

        # speed from VFH front clearance
        if front_min <= self.front_stop_dist:
            base_v = 0.0
        else:
            speed_scale = self.linear_scale(front_min, self.front_stop_dist, self.front_slowdown_dist)
            turn_scale = max(0.25, 1.0 - 0.65 * min(1.0, abs(cmd.angular.z) / self.max_yaw_rate))
            base_v = self.forward_speed * speed_scale * turn_scale
            base_v = max(self.min_speed, base_v)

        # fuse in front/back safety
        front_scale = 1.0 - self.clamp(0.90 * fused['front_risk'] + 0.20 * max(fused['left_risk'], fused['right_risk']), 0.0, 1.0)
        cmd.linear.x = base_v * front_scale

        if self.latest_esp['us_front'] <= self.us_front_stop_cm:
            cmd.linear.x = 0.0

        if fused['front_risk'] >= 0.92:
            cmd.linear.x = 0.0

        # severe side close obstacle: slow down and steer away harder
        if self.latest_esp['us_left'] <= self.us_side_near_cm:
            cmd.linear.x = min(cmd.linear.x, 0.05)
            cmd.angular.z = min(cmd.angular.z, -0.45)

        if self.latest_esp['us_right'] <= self.us_side_near_cm:
            cmd.linear.x = min(cmd.linear.x, 0.05)
            cmd.angular.z = max(cmd.angular.z, 0.45)

        # back pressure only matters if front is reasonably clear
        if fused['back_risk'] > 0.5 and fused['front_risk'] < 0.4:
            cmd.linear.x = max(cmd.linear.x, 0.10)

        if front_min < self.hard_turn_dist and abs(cmd.angular.z) > 0.7 * self.max_yaw_rate:
            cmd.linear.x = min(cmd.linear.x, 0.05)

        cmd.linear.x = self.clamp(cmd.linear.x, 0.0, self.forward_speed)
        return cmd

    def start_escape(self, sector_clearance: List[float], fused):
        self.mode = 'escape'
        self.blocked_counter = 0
        self.escape_accum_rad = 0.0

        left_clear = sum(sector_clearance[self.center_sector + 1:]) if self.center_sector + 1 < self.num_sectors else 0.0
        right_clear = sum(sector_clearance[:self.center_sector]) if self.center_sector > 0 else 0.0

        # bias away from the riskier side
        left_score = left_clear - 2.0 * fused['left_risk']
        right_score = right_clear - 2.0 * fused['right_risk']

        self.escape_turn_dir = 1.0 if left_score >= right_score else -1.0

        self.get_logger().warn(
            f'Entering ESCAPE mode: turning {"left" if self.escape_turn_dir > 0 else "right"} '
            f'for {self.escape_turn_angle_deg:.1f} deg'
        )

    def run_escape(self, dt: float, fused) -> Twist:
        cmd = Twist()

        remaining = self.escape_target_rad - self.escape_accum_rad
        if remaining <= 0.0:
            self.finish_escape()
            return Twist()

        turn_mag = self.escape_turn_rate
        if remaining < math.radians(40.0):
            turn_mag = max(0.35, self.escape_turn_rate * 0.55)

        cmd.linear.x = 0.0
        cmd.angular.z = self.escape_turn_dir * turn_mag

        # side ultrasonic override while escaping
        if self.latest_esp['us_left'] <= self.us_side_near_cm:
            cmd.angular.z = -abs(turn_mag)
        if self.latest_esp['us_right'] <= self.us_side_near_cm:
            cmd.angular.z = abs(turn_mag)

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

    def destroy_node(self):
        try:
            stop = Twist()
            self.cmd_pub.publish(stop)
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VFHWithEspFusion()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
