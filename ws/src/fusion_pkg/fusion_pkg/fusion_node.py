#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32, Float32MultiArray


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        # ---------- Parameters ----------
        self.declare_parameter('tof_features_topic', '/tof/features')
        self.declare_parameter('ultrasonic_features_topic', '/ultrasonic/features')

        # Match VFH sectorization
        self.declare_parameter('front_fov_deg', 160.0)
        self.declare_parameter('sector_angle_deg', 5.0)

        # Penalty spread for front-facing ToF
        self.declare_parameter('tof_center_bearing_rad', 0.0)
        self.declare_parameter('tof_spread_deg', 30.0)

        # ToF weighting
        self.declare_parameter('tof_floor_weight', 1.2)
        self.declare_parameter('tof_dropoff_weight', 2.2)

        # ToF stop / speed logic
        self.declare_parameter('dropoff_stop_threshold', 0.75)
        self.declare_parameter('floor_stop_threshold', 0.90)
        self.declare_parameter('speed_limit_strong_threshold', 0.65)
        self.declare_parameter('speed_limit_medium_threshold', 0.40)

        # Ultrasonic directional bearings
        self.declare_parameter('ultra_left_bearing_rad', 0.52)     # +30 deg
        self.declare_parameter('ultra_center_bearing_rad', 0.0)
        self.declare_parameter('ultra_right_bearing_rad', -0.52)   # -30 deg

        # Ultrasonic spreads
        self.declare_parameter('ultra_side_spread_deg', 20.0)
        self.declare_parameter('ultra_center_spread_deg', 25.0)

        # Ultrasonic weighting
        self.declare_parameter('ultra_weight', 2.5)
        self.declare_parameter('ultra_center_bias', 1.25)

        # Ultrasonic stop / speed thresholds
        self.declare_parameter('ultra_stop_cm', 12.0)
        self.declare_parameter('ultra_slow_cm', 25.0)
        self.declare_parameter('ultra_medium_cm', 35.0)

        self.tof_features_topic = self.get_parameter('tof_features_topic').value
        self.ultrasonic_features_topic = self.get_parameter('ultrasonic_features_topic').value

        self.front_fov_deg = float(self.get_parameter('front_fov_deg').value)
        self.sector_angle_deg = float(self.get_parameter('sector_angle_deg').value)

        self.tof_center_bearing_rad = float(self.get_parameter('tof_center_bearing_rad').value)
        self.tof_spread_deg = float(self.get_parameter('tof_spread_deg').value)

        self.tof_floor_weight = float(self.get_parameter('tof_floor_weight').value)
        self.tof_dropoff_weight = float(self.get_parameter('tof_dropoff_weight').value)

        self.dropoff_stop_threshold = float(self.get_parameter('dropoff_stop_threshold').value)
        self.floor_stop_threshold = float(self.get_parameter('floor_stop_threshold').value)
        self.speed_limit_strong_threshold = float(self.get_parameter('speed_limit_strong_threshold').value)
        self.speed_limit_medium_threshold = float(self.get_parameter('speed_limit_medium_threshold').value)

        self.ultra_left_bearing_rad = float(self.get_parameter('ultra_left_bearing_rad').value)
        self.ultra_center_bearing_rad = float(self.get_parameter('ultra_center_bearing_rad').value)
        self.ultra_right_bearing_rad = float(self.get_parameter('ultra_right_bearing_rad').value)

        self.ultra_side_spread_deg = float(self.get_parameter('ultra_side_spread_deg').value)
        self.ultra_center_spread_deg = float(self.get_parameter('ultra_center_spread_deg').value)

        self.ultra_weight = float(self.get_parameter('ultra_weight').value)
        self.ultra_center_bias = float(self.get_parameter('ultra_center_bias').value)

        self.ultra_stop_cm = float(self.get_parameter('ultra_stop_cm').value)
        self.ultra_slow_cm = float(self.get_parameter('ultra_slow_cm').value)
        self.ultra_medium_cm = float(self.get_parameter('ultra_medium_cm').value)

        # ---------- Sectorization ----------
        self.num_sectors = max(5, int(round(self.front_fov_deg / self.sector_angle_deg)))
        if self.num_sectors % 2 == 0:
            self.num_sectors += 1

        self.total_hist_fov_rad = math.radians(self.front_fov_deg)
        self.sector_width_rad = self.total_hist_fov_rad / self.num_sectors

        # ---------- Latest sensor states ----------
        self.tof_data = None
        self.ultra_data = None
        self.camera_data = None

        # ---------- ROS ----------
        self.sub_tof = self.create_subscription(
            String,
            self.tof_features_topic,
            self.on_tof_features,
            10
        )

        self.sub_ultra = self.create_subscription(
            String,
            self.ultrasonic_features_topic,
            self.on_ultrasonic_features,
            10
        )

        self.penalty_pub = self.create_publisher(Float32MultiArray, '/fusion/sector_penalties', 10)
        self.stop_pub = self.create_publisher(Bool, '/fusion/critical_stop', 10)
        self.speed_pub = self.create_publisher(Float32, '/fusion/speed_limit', 10)
        self.debug_pub = self.create_publisher(String, '/fusion/debug_state', 10)

        self.timer = self.create_timer(0.1, self.publish_fusion)  # 10 Hz

        self.get_logger().info(
            f'Fusion node started. sectors={self.num_sectors}, '
            f'tof_topic={self.tof_features_topic}, ultra_topic={self.ultrasonic_features_topic}'
        )

    # ---------- Parsing ----------
    def parse_fields(self, text: str):
        fields = {}
        for part in text.split('|'):
            if ':' not in part:
                continue
            key, value = part.split(':', 1)
            fields[key] = value
        return fields

    def on_tof_features(self, msg: String):
        self.tof_data = self.parse_fields(msg.data)

    def on_ultrasonic_features(self, msg: String):
        self.ultra_data = self.parse_fields(msg.data)

    # ---------- Helpers ----------
    @staticmethod
    def clamp(x: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, x))

    def sector_center_angle(self, idx: int) -> float:
        half_fov = self.total_hist_fov_rad / 2.0
        return -half_fov + (idx + 0.5) * self.sector_width_rad

    def add_directional_penalty(self, penalties, center_bearing, spread_deg, magnitude):
        if magnitude <= 0.0:
            return

        spread_rad = math.radians(spread_deg)

        for i in range(self.num_sectors):
            ang = self.sector_center_angle(i)
            d = abs(ang - center_bearing)
            if d > spread_rad:
                continue

            w = max(0.0, 1.0 - d / spread_rad)
            penalties[i] += magnitude * w

    # ---------- Core fusion ----------
    def publish_fusion(self):
        penalties = [0.0] * self.num_sectors
        critical_stop = False
        speed_limit = 1.0
        debug_parts = []

        # ---------------- ToF contribution ----------------
        if self.tof_data is not None:
            try:
                valid = int(self.tof_data.get('VALID', '0'))
                state = self.tof_data.get('STATE', 'invalid')

                floor_hazard = float(self.tof_data.get('FLOOR', '0.0'))
                dropoff_hazard = float(self.tof_data.get('DROPOFF', '0.0'))

                near_min = float(self.tof_data.get('NEAR_MIN', '300.0'))
                near_avg = float(self.tof_data.get('NEAR_AVG', '300.0'))
                far_avg = float(self.tof_data.get('FAR_AVG', '300.0'))

                bearing = float(self.tof_data.get('BEARING', f'{self.tof_center_bearing_rad:.2f}'))
            except Exception as e:
                self.get_logger().warn(f'Bad /tof/features fields: {e}')
                valid = 0
                state = 'invalid'
                floor_hazard = 0.0
                dropoff_hazard = 0.0
                near_min = 300.0
                near_avg = 300.0
                far_avg = 300.0
                bearing = self.tof_center_bearing_rad

            if valid == 1:
                tof_penalty = (
                    self.tof_floor_weight * floor_hazard +
                    self.tof_dropoff_weight * dropoff_hazard
                )

                self.add_directional_penalty(
                    penalties,
                    center_bearing=bearing,
                    spread_deg=self.tof_spread_deg,
                    magnitude=tof_penalty
                )

                if dropoff_hazard >= self.dropoff_stop_threshold:
                    critical_stop = True
                if floor_hazard >= self.floor_stop_threshold and near_min <= 25.0:
                    critical_stop = True

                strongest = max(floor_hazard, dropoff_hazard)

                if strongest >= self.speed_limit_strong_threshold:
                    speed_limit = min(speed_limit, 0.35)
                elif strongest >= self.speed_limit_medium_threshold:
                    speed_limit = min(speed_limit, 0.60)
                elif strongest > 0.0:
                    speed_limit = min(speed_limit, 0.80)

                debug_parts.append(
                    f"tof(state={state},floor={floor_hazard:.2f},drop={dropoff_hazard:.2f},"
                    f"near_min={near_min:.1f},near_avg={near_avg:.1f},far_avg={far_avg:.1f})"
                )
            else:
                debug_parts.append("tof(invalid)")

        # ---------------- Ultrasonic contribution ----------------
        if self.ultra_data is not None:
            try:
                valid = int(self.ultra_data.get('VALID', '0'))
                state = self.ultra_data.get('STATE', 'invalid')

                left_cm = float(self.ultra_data.get('LEFT_CM', '999.0'))
                center_cm = float(self.ultra_data.get('CENTER_CM', '999.0'))
                right_cm = float(self.ultra_data.get('RIGHT_CM', '999.0'))

                left_hazard = float(self.ultra_data.get('LEFT_HAZARD', '0.0'))
                center_hazard = float(self.ultra_data.get('CENTER_HAZARD', '0.0'))
                right_hazard = float(self.ultra_data.get('RIGHT_HAZARD', '0.0'))

                min_cm = float(self.ultra_data.get('MIN_CM', '999.0'))
                avg_cm = float(self.ultra_data.get('AVG_CM', '999.0'))
                strongest_name = self.ultra_data.get('STRONGEST', 'none')
            except Exception as e:
                self.get_logger().warn(f'Bad /ultrasonic/features fields: {e}')
                valid = 0
                state = 'invalid'
                left_cm = 999.0
                center_cm = 999.0
                right_cm = 999.0
                left_hazard = 0.0
                center_hazard = 0.0
                right_hazard = 0.0
                min_cm = 999.0
                avg_cm = 999.0
                strongest_name = 'none'

            if valid == 1:
                self.add_directional_penalty(
                    penalties,
                    center_bearing=self.ultra_left_bearing_rad,
                    spread_deg=self.ultra_side_spread_deg,
                    magnitude=self.ultra_weight * left_hazard
                )

                self.add_directional_penalty(
                    penalties,
                    center_bearing=self.ultra_center_bearing_rad,
                    spread_deg=self.ultra_center_spread_deg,
                    magnitude=self.ultra_weight * self.ultra_center_bias * center_hazard
                )

                self.add_directional_penalty(
                    penalties,
                    center_bearing=self.ultra_right_bearing_rad,
                    spread_deg=self.ultra_side_spread_deg,
                    magnitude=self.ultra_weight * right_hazard
                )

                # Very close object in front => immediate stop
                if center_cm <= self.ultra_stop_cm:
                    critical_stop = True

                # If anything is extremely close, also stop
                if min_cm <= max(6.0, self.ultra_stop_cm * 0.75):
                    critical_stop = True

                # Speed limiting from ultrasonic proximity
                if center_cm <= self.ultra_stop_cm:
                    speed_limit = min(speed_limit, 0.0)
                elif center_cm <= self.ultra_slow_cm or min_cm <= self.ultra_slow_cm:
                    speed_limit = min(speed_limit, 0.30)
                elif center_cm <= self.ultra_medium_cm or min_cm <= self.ultra_medium_cm:
                    speed_limit = min(speed_limit, 0.55)
                elif max(left_hazard, center_hazard, right_hazard) > 0.0:
                    speed_limit = min(speed_limit, 0.75)

                debug_parts.append(
                    f"ultra(state={state},left={left_cm:.1f},center={center_cm:.1f},"
                    f"right={right_cm:.1f},min={min_cm:.1f},avg={avg_cm:.1f},strongest={strongest_name})"
                )
            else:
                debug_parts.append(f"ultra({state})")

        # ---------------- Future camera contribution ----------------
        if self.camera_data is not None:
            debug_parts.append("camera(connected_later)")

        penalties = [self.clamp(p, 0.0, 3.0) for p in penalties]
        speed_limit = self.clamp(speed_limit, 0.0, 1.0)

        pen_msg = Float32MultiArray()
        pen_msg.data = penalties
        self.penalty_pub.publish(pen_msg)

        stop_msg = Bool()
        stop_msg.data = critical_stop
        self.stop_pub.publish(stop_msg)

        speed_msg = Float32()
        speed_msg.data = speed_limit
        self.speed_pub.publish(speed_msg)

        dbg_msg = String()
        dbg_msg.data = (
            f"critical_stop={int(critical_stop)}|speed_limit={speed_limit:.2f}|"
            f"penalty_max={max(penalties) if penalties else 0.0:.2f}|"
            + "|".join(debug_parts)
        )
        self.debug_pub.publish(dbg_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
