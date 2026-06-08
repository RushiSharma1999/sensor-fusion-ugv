#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class GapFollow(Node):
    def __init__(self):
        super().__init__('gap_follow')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("GapFollow v3 started.")

        # -------- Tunables --------
        self.max_range_clip = 6.0
        self.front_fov_deg = 150.0
        self.forward_cone_deg = 30.0

        self.stop_distance = 0.55
        self.escape_distance = 0.30
        self.slow_distance = 1.50

        self.min_gap_distance = 0.80
        self.bubble_radius_m = 0.30

        self.base_speed = 0.16
        self.cruise_speed = 0.24
        self.max_speed = 0.32
        self.min_speed = 0.08

        self.k_ang = 0.90
        self.max_w = 0.70

        self.smooth_window = 5
        self.target_alpha = 0.20
        self.angle_deadband = 0.07

        self.open_space_distance = 2.2
        self.open_space_angle_deg = 12.0

        self.log_every = 10
        # --------------------------

        self.prev_target_angle = 0.0
        self.log_count = 0

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def angle_of_index(self, msg, i):
        return msg.angle_min + i * msg.angle_increment

    def moving_average(self, data, window):
        if window <= 1:
            return data[:]

        out = [0.0] * len(data)
        half = window // 2
        for i in range(len(data)):
            vals = []
            for j in range(max(0, i - half), min(len(data), i + half + 1)):
                if data[j] > 0.0:
                    vals.append(data[j])
            out[i] = sum(vals) / len(vals) if vals else 0.0
        return out

    def publish_cmd(self, v, w):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def maybe_log(self, text):
        self.log_count += 1
        if self.log_count % self.log_every == 0:
            self.get_logger().info(text)

    def scan_callback(self, msg: LaserScan):
        ranges = list(msg.ranges)
        n = len(ranges)
        if n == 0:
            return

        # 1) Clean scan
        # inf means open space, not obstacle.
        clean = [0.0] * n
        for i, r in enumerate(ranges):
            if r is None or math.isnan(r) or r <= 0.0:
                clean[i] = 0.0
            elif math.isinf(r):
                clean[i] = self.max_range_clip
            else:
                clean[i] = min(r, self.max_range_clip)

        # 2) Smooth a bit
        proc = self.moving_average(clean, self.smooth_window)

        # 3) Restrict to front FOV
        half_fov = math.radians(self.front_fov_deg / 2.0)
        start_i = max(0, int(((-half_fov) - msg.angle_min) / msg.angle_increment))
        end_i = min(n - 1, int(((+half_fov) - msg.angle_min) / msg.angle_increment))

        masked = [0.0] * n
        for i in range(start_i, end_i + 1):
            masked[i] = proc[i]

        valid_front = [i for i in range(start_i, end_i + 1) if masked[i] > 0.0]
        if not valid_front:
            self.publish_cmd(0.0, 0.25)
            return

        # 4) Basic measurements
        closest_i = min(valid_front, key=lambda i: masked[i])
        closest_d = masked[closest_i]

        half_fwd = math.radians(self.forward_cone_deg / 2.0)
        fwd_start = max(0, int(((-half_fwd) - msg.angle_min) / msg.angle_increment))
        fwd_end = min(n - 1, int(((+half_fwd) - msg.angle_min) / msg.angle_increment))
        forward_vals = [masked[i] for i in range(fwd_start, fwd_end + 1) if masked[i] > 0.0]
        forward_min = min(forward_vals) if forward_vals else float('inf')

        center_i = (start_i + end_i) // 2
        center_angle = self.angle_of_index(msg, center_i)

        # 5) Hard safety stop / escape
        if math.isfinite(forward_min) and forward_min < self.escape_distance:
            # rotate in place away from current target bias
            turn_dir = -1.0 if self.prev_target_angle > 0.0 else 1.0
            self.publish_cmd(0.0, 0.40 * turn_dir)
            self.maybe_log(
                f"ESCAPE closest_d={closest_d:.2f} fwd_min={forward_min:.2f}"
            )
            return

        if math.isfinite(forward_min) and forward_min < self.stop_distance:
            self.publish_cmd(0.0, 0.0)
            self.maybe_log(
                f"STOP closest_d={closest_d:.2f} fwd_min={forward_min:.2f}"
            )
            return

        # 6) If ahead is open, force straight
        if (not math.isfinite(forward_min)) or forward_min > self.open_space_distance:
            target_angle = 0.0
            gap_len = -1
            best_i = center_i
        else:
            # 7) Bubble out nearest obstacle
            bubble = masked[:]
            bubble_angle = math.atan2(self.bubble_radius_m, max(closest_d, 1e-3))
            bubble_pts = max(1, int(bubble_angle / abs(msg.angle_increment)))

            for i in range(max(start_i, closest_i - bubble_pts),
                           min(end_i, closest_i + bubble_pts) + 1):
                bubble[i] = 0.0

            # 8) Find gaps
            gaps = []
            cur_start = None
            for i in range(start_i, end_i + 2):
                is_open = (i <= end_i) and (bubble[i] > self.min_gap_distance)
                if is_open and cur_start is None:
                    cur_start = i
                elif (not is_open) and cur_start is not None:
                    gaps.append((cur_start, i - 1))
                    cur_start = None

            if not gaps:
                best_i = center_i
                gap_len = 0
            else:
                # Pick widest gap, with a mild preference for center
                best_gap = None
                best_score = -1e9
                gap_len = 0

                for gs, ge in gaps:
                    width = ge - gs + 1
                    gi = (gs + ge) // 2
                    ang = self.angle_of_index(msg, gi)

                    # prefer wide gaps near straight ahead
                    score = 2.5 * width - 1.0 * abs(ang)
                    if score > best_score:
                        best_score = score
                        best_gap = (gs, ge)
                        gap_len = width

                gs, ge = best_gap

                # Target center of best gap
                best_i = (gs + ge) // 2

                # Gentle refinement near center only
                local_radius = max(2, gap_len // 8)
                local_start = max(gs, best_i - local_radius)
                local_end = min(ge, best_i + local_radius)

                refine_i = best_i
                refine_score = -1e9
                for i in range(local_start, local_end + 1):
                    d = bubble[i]
                    if d <= 0.0:
                        continue
                    ang = self.angle_of_index(msg, i)
                    score = 1.5 * d - 1.2 * abs(ang)
                    if score > refine_score:
                        refine_score = score
                        refine_i = i

                best_i = refine_i

            target_angle = self.angle_of_index(msg, best_i)
            target_angle = math.atan2(math.sin(target_angle), math.cos(target_angle))

            # If nearly centered and reasonably clear, go straight
            if abs(target_angle) < math.radians(self.open_space_angle_deg) and \
               ((math.isfinite(forward_min) and forward_min > 1.2) or closest_d > 1.2):
                target_angle = 0.0

        # 9) Light smoothing only
        target_angle = (
            self.target_alpha * self.prev_target_angle
            + (1.0 - self.target_alpha) * target_angle
        )

        if abs(target_angle) < self.angle_deadband:
            target_angle = 0.0

        self.prev_target_angle = target_angle

        # 10) Steering
        # If robot turns the wrong way, flip the sign here.
        w = self.k_ang * target_angle
        w = self.clamp(w, -self.max_w, self.max_w)

        # 11) Speed control
        if not math.isfinite(forward_min):
            clearance_scale = 1.0
        else:
            clearance_scale = self.clamp(
                (forward_min - self.stop_distance) / (self.slow_distance - self.stop_distance),
                0.0,
                1.0
            )

        # Reduce speed during turning, but do not kill it
        turn_scale = max(0.50, 1.0 - abs(w) / self.max_w)

        v = self.base_speed + (self.cruise_speed - self.base_speed) * clearance_scale
        v *= turn_scale

        # Extra reward for being aligned
        if abs(target_angle) < 0.10 and clearance_scale > 0.7:
            v = max(v, 0.22)

        v = self.clamp(v, self.min_speed, self.max_speed)

        # 12) Publish
        self.publish_cmd(v, w)

        self.maybe_log(
            f"closest_d={closest_d:.2f} "
            f"gap_len={gap_len} "
            f"best_i={best_i} "
            f"ang={target_angle:+.2f} "
            f"fwd_min={forward_min:.2f} "
            f"v={v:.2f} "
            f"w={w:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GapFollow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
