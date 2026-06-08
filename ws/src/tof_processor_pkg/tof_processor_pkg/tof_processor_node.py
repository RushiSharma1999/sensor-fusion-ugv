#!/usr/bin/env python3
import collections

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TofProcessorNode(Node):
    def __init__(self):
        super().__init__('tof_processor_node')

        # ---------- Parameters ----------
        self.declare_parameter('summary_topic', '/tof/front/summary')
        self.declare_parameter('features_topic', '/tof/features')
        self.declare_parameter('publish_hz', 15.0)

        # Invalid handling
        self.declare_parameter('far_avg_invalid_high', 280.0)
        self.declare_parameter('near_avg_invalid_high', 280.0)
        self.declare_parameter('center_invalid_low', 5.0)

        # Baseline learning
        self.declare_parameter('baseline_window', 30)
        self.declare_parameter('baseline_min_samples', 10)

        # Floor obstacle thresholds
        self.declare_parameter('floor_nearmin_rise_threshold', 8.0)
        self.declare_parameter('floor_nearavg_rise_threshold', 10.0)

        # Edge thresholds (fast path: absolute geometry)
        self.declare_parameter('edge_center_gap_threshold', 80.0)          # center - near_min
        self.declare_parameter('edge_center_gap_strong', 100.0)
        self.declare_parameter('edge_near_min_max', 70.0)                  # near_min still "close"
        self.declare_parameter('edge_center_absolute_min', 110.0)

        # Edge thresholds (support path: baseline-relative)
        self.declare_parameter('edge_center_delta_threshold', 10.0)
        self.declare_parameter('edge_far_delta_threshold', 10.0)
        self.declare_parameter('edge_center_nearavg_gap_threshold', 28.0)  # center - near_avg

        # Persistence
        self.declare_parameter('obstacle_set_frames', 2)
        self.declare_parameter('edge_set_frames', 1)

        summary_topic = self.get_parameter('summary_topic').value
        features_topic = self.get_parameter('features_topic').value
        publish_hz = float(self.get_parameter('publish_hz').value)

        self.far_avg_invalid_high = float(self.get_parameter('far_avg_invalid_high').value)
        self.near_avg_invalid_high = float(self.get_parameter('near_avg_invalid_high').value)
        self.center_invalid_low = float(self.get_parameter('center_invalid_low').value)

        self.baseline_window = int(self.get_parameter('baseline_window').value)
        self.baseline_min_samples = int(self.get_parameter('baseline_min_samples').value)

        self.floor_nearmin_rise_threshold = float(self.get_parameter('floor_nearmin_rise_threshold').value)
        self.floor_nearavg_rise_threshold = float(self.get_parameter('floor_nearavg_rise_threshold').value)

        self.edge_center_gap_threshold = float(self.get_parameter('edge_center_gap_threshold').value)
        self.edge_center_gap_strong = float(self.get_parameter('edge_center_gap_strong').value)
        self.edge_near_min_max = float(self.get_parameter('edge_near_min_max').value)
        self.edge_center_absolute_min = float(self.get_parameter('edge_center_absolute_min').value)

        self.edge_center_delta_threshold = float(self.get_parameter('edge_center_delta_threshold').value)
        self.edge_far_delta_threshold = float(self.get_parameter('edge_far_delta_threshold').value)
        self.edge_center_nearavg_gap_threshold = float(self.get_parameter('edge_center_nearavg_gap_threshold').value)

        self.obstacle_set_frames = int(self.get_parameter('obstacle_set_frames').value)
        self.edge_set_frames = int(self.get_parameter('edge_set_frames').value)

        # ---------- State ----------
        self.front_data = None

        self.baseline_center = collections.deque(maxlen=self.baseline_window)
        self.baseline_near_min = collections.deque(maxlen=self.baseline_window)
        self.baseline_near_avg = collections.deque(maxlen=self.baseline_window)
        self.baseline_far_avg = collections.deque(maxlen=self.baseline_window)

        self.obstacle_counter = 0
        self.edge_counter = 0

        # ---------- ROS ----------
        self.sub_front = self.create_subscription(
            String,
            summary_topic,
            self.on_front,
            10
        )

        self.features_pub = self.create_publisher(String, features_topic, 10)
        self.timer = self.create_timer(1.0 / publish_hz, self.publish_features)

        self.get_logger().info('ToF processor node started')
        self.get_logger().info(f'summary_topic={summary_topic}, features_topic={features_topic}')

    def parse_summary(self, text: str):
        fields = {}
        for part in text.split('|'):
            if ':' not in part:
                continue
            key, value = part.split(':', 1)
            fields[key] = value
        return fields

    def on_front(self, msg: String):
        self.front_data = self.parse_summary(msg.data)

    @staticmethod
    def clamp(x: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, x))

    @staticmethod
    def mean(values):
        if not values:
            return None
        return sum(values) / len(values)

    def is_valid_frame(self, valid: int, center: float, near_min: float, near_avg: float, far_avg: float) -> bool:
        if valid != 1:
            return False
        if far_avg >= self.far_avg_invalid_high:
            return False
        if near_avg >= self.near_avg_invalid_high:
            return False
        if center <= self.center_invalid_low:
            return False
        if near_min <= 0.0:
            return False
        return True

    def baseline_ready(self) -> bool:
        return (
            len(self.baseline_center) >= self.baseline_min_samples and
            len(self.baseline_near_min) >= self.baseline_min_samples and
            len(self.baseline_near_avg) >= self.baseline_min_samples and
            len(self.baseline_far_avg) >= self.baseline_min_samples
        )

    def maybe_update_baseline(
        self,
        center: float,
        near_min: float,
        near_avg: float,
        far_avg: float,
        obstacle_raw: bool,
        edge_raw: bool
    ):
        # Only learn baseline while clear
        if not obstacle_raw and not edge_raw:
            self.baseline_center.append(center)
            self.baseline_near_min.append(near_min)
            self.baseline_near_avg.append(near_avg)
            self.baseline_far_avg.append(far_avg)

    def publish_features(self):
        if self.front_data is None:
            return

        try:
            sensor_id = self.front_data.get('ID', 'FRONT')
            timestamp_ms = int(self.front_data.get('T', '0'))
            valid = int(self.front_data.get('VALID', '0'))
            center = float(self.front_data.get('CENTER', '0.0'))
            near_min = float(self.front_data.get('NEAR_MIN', '300.0'))
            near_avg = float(self.front_data.get('NEAR_AVG', '300.0'))
            far_avg = float(self.front_data.get('FAR_AVG', '300.0'))
        except Exception as e:
            self.get_logger().warn(f'Bad summary fields: {e}')
            return

        frame_valid = self.is_valid_frame(valid, center, near_min, near_avg, far_avg)

        if not frame_valid:
            out = String()
            out.data = (
                f"ID:{sensor_id}|T:{timestamp_ms}|VALID:0|BEARING:0.00|"
                f"CENTER:{center:.1f}|NEAR_MIN:{near_min:.1f}|NEAR_AVG:{near_avg:.1f}|FAR_AVG:{far_avg:.1f}|"
                f"BASE_CENTER:-1.0|BASE_NEAR_MIN:-1.0|BASE_NEAR_AVG:-1.0|BASE_FAR_AVG:-1.0|"
                f"D_CENTER:0.0|D_NEAR_MIN:0.0|D_NEAR_AVG:0.0|D_FAR_AVG:0.0|"
                f"CENTER_GAP:0.0|CENTER_NEARAVG_GAP:0.0|"
                f"FLOOR:0.00|DROPOFF:0.00|STATE:invalid"
            )
            self.features_pub.publish(out)
            return

        if not self.baseline_ready():
            self.baseline_center.append(center)
            self.baseline_near_min.append(near_min)
            self.baseline_near_avg.append(near_avg)
            self.baseline_far_avg.append(far_avg)

            base_center = self.mean(self.baseline_center)
            base_near_min = self.mean(self.baseline_near_min)
            base_near_avg = self.mean(self.baseline_near_avg)
            base_far_avg = self.mean(self.baseline_far_avg)

            out = String()
            out.data = (
                f"ID:{sensor_id}|T:{timestamp_ms}|VALID:1|BEARING:0.00|"
                f"CENTER:{center:.1f}|NEAR_MIN:{near_min:.1f}|NEAR_AVG:{near_avg:.1f}|FAR_AVG:{far_avg:.1f}|"
                f"BASE_CENTER:{base_center:.1f}|BASE_NEAR_MIN:{base_near_min:.1f}|"
                f"BASE_NEAR_AVG:{base_near_avg:.1f}|BASE_FAR_AVG:{base_far_avg:.1f}|"
                f"D_CENTER:0.0|D_NEAR_MIN:0.0|D_NEAR_AVG:0.0|D_FAR_AVG:0.0|"
                f"CENTER_GAP:0.0|CENTER_NEARAVG_GAP:0.0|"
                f"FLOOR:0.00|DROPOFF:0.00|STATE:calibrating"
            )
            self.features_pub.publish(out)
            return

        # ---------- Baselines ----------
        base_center = self.mean(self.baseline_center)
        base_near_min = self.mean(self.baseline_near_min)
        base_near_avg = self.mean(self.baseline_near_avg)
        base_far_avg = self.mean(self.baseline_far_avg)

        # ---------- Deltas ----------
        d_center = center - base_center
        d_near_min = near_min - base_near_min
        d_near_avg = near_avg - base_near_avg
        d_far_avg = far_avg - base_far_avg

        # ---------- Absolute geometry cues ----------
        center_gap = center - near_min
        center_nearavg_gap = center - near_avg

        # ---------- Edge logic ----------
        # Fast path: center opens up hard while near_min still looks close.
        strong_absolute_edge_raw = (
            center_gap >= self.edge_center_gap_strong and
            near_min <= self.edge_near_min_max
        )

        moderate_absolute_edge_raw = (
            center_gap >= self.edge_center_gap_threshold and
            center >= self.edge_center_absolute_min and
            near_min <= self.edge_near_min_max
        )

        # Support path: baseline-relative change, but lighter-weight than before.
        baseline_edge_raw = (
            d_center >= self.edge_center_delta_threshold and
            (
                d_far_avg >= self.edge_far_delta_threshold or
                center_nearavg_gap >= self.edge_center_nearavg_gap_threshold
            )
        )

        edge_raw = (
            strong_absolute_edge_raw or
            moderate_absolute_edge_raw or
            baseline_edge_raw
        )

        # ---------- Floor obstacle logic ----------
        # Floor obstacle should usually look like near region getting closer / larger,
        # but not the big center-gap signature of a ledge.
        obstacle_raw = (
            not edge_raw and
            (
                d_near_min >= self.floor_nearmin_rise_threshold or
                d_near_avg >= self.floor_nearavg_rise_threshold
            )
        )

        # ---------- Persistence ----------
        self.edge_counter = self.edge_counter + 1 if edge_raw else 0
        self.obstacle_counter = self.obstacle_counter + 1 if obstacle_raw else 0

        edge_detected = self.edge_counter >= self.edge_set_frames
        obstacle_detected = self.obstacle_counter >= self.obstacle_set_frames

        # ---------- Hazard scores ----------
        floor_hazard = 0.0
        dropoff_hazard = 0.0

        # Floor hazard
        if d_near_min > 0.0:
            floor_hazard += 0.65 * self.clamp(d_near_min / 18.0, 0.0, 1.0)
        if d_near_avg > 0.0:
            floor_hazard += 0.35 * self.clamp(d_near_avg / 22.0, 0.0, 1.0)
        if obstacle_detected:
            floor_hazard = max(floor_hazard, 0.70)
        floor_hazard = self.clamp(floor_hazard, 0.0, 1.0)

        # Dropoff hazard: center contrast is primary
        if center_gap > 0.0:
            dropoff_hazard += 0.60 * self.clamp(center_gap / 120.0, 0.0, 1.0)
        if center_nearavg_gap > 0.0:
            dropoff_hazard += 0.20 * self.clamp(center_nearavg_gap / 50.0, 0.0, 1.0)
        if d_center > 0.0:
            dropoff_hazard += 0.10 * self.clamp(d_center / 30.0, 0.0, 1.0)
        if d_far_avg > 0.0:
            dropoff_hazard += 0.10 * self.clamp(d_far_avg / 40.0, 0.0, 1.0)
        if edge_detected:
            dropoff_hazard = max(dropoff_hazard, 0.80)
        dropoff_hazard = self.clamp(dropoff_hazard, 0.0, 1.0)

        # ---------- Update baseline ----------
        self.maybe_update_baseline(
            center=center,
            near_min=near_min,
            near_avg=near_avg,
            far_avg=far_avg,
            obstacle_raw=obstacle_raw,
            edge_raw=edge_raw
        )

        # ---------- Final state ----------
        if edge_detected:
            state = 'dropoff'
        elif obstacle_detected:
            state = 'floor_obstacle'
        else:
            state = 'clear'

        out = String()
        out.data = (
            f"ID:{sensor_id}|T:{timestamp_ms}|VALID:1|BEARING:0.00|"
            f"CENTER:{center:.1f}|NEAR_MIN:{near_min:.1f}|NEAR_AVG:{near_avg:.1f}|FAR_AVG:{far_avg:.1f}|"
            f"BASE_CENTER:{base_center:.1f}|BASE_NEAR_MIN:{base_near_min:.1f}|"
            f"BASE_NEAR_AVG:{base_near_avg:.1f}|BASE_FAR_AVG:{base_far_avg:.1f}|"
            f"D_CENTER:{d_center:.1f}|D_NEAR_MIN:{d_near_min:.1f}|D_NEAR_AVG:{d_near_avg:.1f}|D_FAR_AVG:{d_far_avg:.1f}|"
            f"CENTER_GAP:{center_gap:.1f}|CENTER_NEARAVG_GAP:{center_nearavg_gap:.1f}|"
            f"FLOOR:{floor_hazard:.2f}|DROPOFF:{dropoff_hazard:.2f}|STATE:{state}"
        )
        self.features_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TofProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
