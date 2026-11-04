#!/usr/bin/env python3
"""Read-only cloud/scan comparison. No publishers, serial access, or motor commands.

Run under ROS 2 Humble after sourcing /opt/ros/humble/setup.bash.
Defaults match the converter parameters reported on 2026-11-04.
Messages are paired by exact source header stamp, not arrival time. Comparison
requires matching frame IDs; this tool does not transform clouds. Its projection
is diagnostic, not a replacement perception or safety controller.
"""
import argparse
from collections import Counter, OrderedDict
import json
import math
import struct
import time


def emit(record):
    print(json.dumps(record, allow_nan=False, sort_keys=True), flush=True)


def xyz_points(msg):
    """Honor PointCloud2 offsets, row padding, byte order and float precision."""
    if msg.width == 0 or msg.height == 0:
        return
    formats = {7: 'f', 8: 'd'}  # PointField FLOAT32 / FLOAT64
    fields = {field.name: field for field in msg.fields}
    readers = []
    for name in ('x', 'y', 'z'):
        if name not in fields:
            raise ValueError('PointCloud2 missing ' + name)
        field = fields[name]
        if field.datatype not in formats or field.count != 1:
            raise ValueError('Expected scalar float32/float64 field: ' + name)
        reader = struct.Struct(('>' if msg.is_bigendian else '<') + formats[field.datatype])
        if field.offset < 0 or field.offset + reader.size > msg.point_step:
            raise ValueError('Field exceeds point_step: ' + name)
        readers.append((field.offset, reader))
    if msg.point_step <= 0 or msg.row_step < msg.width * msg.point_step:
        raise ValueError('Invalid PointCloud2 strides')
    data = memoryview(msg.data)
    if len(data) < msg.row_step * msg.height:
        raise ValueError('Truncated PointCloud2 data')
    for row in range(msg.height):
        for column in range(msg.width):
            offset = row * msg.row_step + column * msg.point_step
            point = tuple(reader.unpack_from(data, offset + delta)[0] for delta, reader in readers)
            if all(math.isfinite(value) for value in point):
                yield point


def finite_front(scan, half_angle):
    return [float(value) for i, value in enumerate(scan.ranges)
            if abs(scan.angle_min + i * scan.angle_increment) <= half_angle
            and math.isfinite(value) and value > 0
            and scan.range_min <= value <= scan.range_max]


def compare(cloud, scan, min_height, max_height, half_angle):
    metadata = (scan.angle_min, scan.angle_max, scan.angle_increment, scan.range_min, scan.range_max)
    if (not all(math.isfinite(value) for value in metadata) or scan.angle_increment <= 0
            or scan.angle_max <= scan.angle_min or scan.range_max <= scan.range_min):
        raise ValueError('Invalid LaserScan geometry or range limits')
    observed = finite_front(scan, half_angle)
    base = {'stamp': [scan.header.stamp.sec, scan.header.stamp.nanosec],
            'cloud_frame': cloud.header.frame_id, 'scan_frame': scan.header.frame_id,
            'scan_front_valid_bins': len(observed),
            'scan_front_min_m': min(observed) if observed else None}
    if cloud.header.frame_id != scan.header.frame_id:
        return dict(base, comparable=False, reason='frame_mismatch_requires_transform')
    below = inside = above = 0
    z_values, near_z_values, predicted_bins = [], [], {}
    nearest = []
    for x, y, z in xyz_points(cloud):
        distance = math.hypot(x, y)
        if not scan.range_min <= distance <= scan.range_max or distance <= 0:
            continue
        angle = math.atan2(y, x)
        if not scan.angle_min <= angle <= scan.angle_max:
            continue
        index = int((angle - scan.angle_min) / scan.angle_increment)
        if not 0 <= index < len(scan.ranges):
            continue
        # Use the same bin-angle convention as the current VFH front check.
        if abs(scan.angle_min + index * scan.angle_increment) > half_angle:
            continue
        z_values.append(z)
        if distance <= 1.2:
            near_z_values.append(z)
        nearest.append((distance, z))
        if z < min_height:
            below += 1
        elif z > max_height:
            above += 1
        else:
            inside += 1
            predicted_bins[index] = min(distance, predicted_bins.get(index, math.inf))
    raw_count = below + inside + above
    classification = 'front_present'
    if not observed:
        if raw_count == 0:
            classification = 'raw_cloud_has_no_front_points_in_scan_range'
        elif inside == 0:
            classification = 'configured_height_band_removes_all_front_points'
        else:
            classification = 'predicted_front_points_exist_but_scan_front_empty'
    return dict(base, comparable=True, classification=classification,
                raw_front_points=raw_count, below_height=below, within_height=inside, above_height=above,
                predicted_front_bins=len(predicted_bins),
                raw_front_z_min_m=min(z_values) if z_values else None,
                raw_front_z_max_m=max(z_values) if z_values else None,
                near_front_z_min_m=min(near_z_values) if near_z_values else None,
                near_front_z_max_m=max(near_z_values) if near_z_values else None,
                nearest_front_range_z_m=[list(pair) for pair in sorted(nearest)[:5]])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=20.)
    parser.add_argument('--cloud-topic', default='/unilidar/cloud')
    parser.add_argument('--scan-topic', default='/scan')
    parser.add_argument('--min-height', type=float, default=-0.1)
    parser.add_argument('--max-height', type=float, default=1.0)
    parser.add_argument('--front-half-angle-deg', type=float, default=15.)
    args = parser.parse_args()
    if (not all(math.isfinite(v) for v in (args.seconds, args.min_height, args.max_height,
                                          args.front_half_angle_deg)) or args.seconds <= 0
            or args.max_height < args.min_height or not 0 < args.front_half_angle_deg <= 180):
        parser.error('Invalid duration, height band, or front angle')
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.executors import ExternalShutdownException
        from sensor_msgs.msg import PointCloud2, LaserScan
    except ImportError as exc:
        parser.exit(2, f'ROS imports unavailable: {exc}. Source /opt/ros/humble/setup.bash first.\n')

    class Diagnostic(Node):
        def __init__(self):
            super().__init__('ugv_scan_diagnostic')
            self.clouds, self.scans = OrderedDict(), OrderedDict()
            self.counts = Counter()
            self.first_report = True
            self.last_print = -math.inf
            self.subscriptions_kept = [
                self.create_subscription(PointCloud2, args.cloud_topic, self.on_cloud, qos_profile_sensor_data),
                self.create_subscription(LaserScan, args.scan_topic, self.on_scan, qos_profile_sensor_data)]

        def on_cloud(self, msg):
            self.receive(msg, self.clouds, self.scans, 'cloud')

        def on_scan(self, msg):
            self.receive(msg, self.scans, self.clouds, 'scan')

        def receive(self, msg, own, other, kind):
            self.counts[kind + '_received'] += 1
            key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            if key == (0, 0):
                self.counts['zero_stamp_' + kind] += 1
                return
            if key not in other:
                if key in own:
                    self.counts['duplicate_pending_stamp_' + kind] += 1
                own[key] = msg
                if len(own) > 64:
                    own.popitem(last=False)
                    self.counts['unmatched_evicted_' + kind] += 1
                return
            partner = other.pop(key)
            cloud, scan = (msg, partner) if kind == 'cloud' else (partner, msg)
            self.counts['stamp_pairs'] += 1
            try:
                row = compare(cloud, scan, args.min_height, args.max_height,
                              math.radians(args.front_half_angle_deg))
            except (ValueError, TypeError, struct.error) as exc:
                self.counts['comparison_errors'] += 1
                if self.counts['comparison_errors'] <= 3:
                    emit({'event': 'comparison_error', 'detail': str(exc)})
                return
            self.counts[row.get('classification', row.get('reason'))] += 1
            if row['scan_front_valid_bins'] == 0:
                self.counts['paired_scan_front_empty'] += 1
            elapsed = time.monotonic() - started
            if self.first_report or elapsed - self.last_print >= 1.0 or row['scan_front_valid_bins'] == 0:
                emit(dict(row, event='pair', elapsed_s=round(elapsed, 3)))
                self.first_report = False
                self.last_print = elapsed

    rclpy.init(args=[])
    node = Diagnostic()
    started = time.monotonic()
    emit({'event': 'start', 'seconds': args.seconds, 'cloud_topic': args.cloud_topic,
          'scan_topic': args.scan_topic, 'height_band_m': [args.min_height, args.max_height],
          'front_half_angle_deg': args.front_half_angle_deg,
          'note': 'Read only. Height band must match converter. Exact-stamp pairing; no TF transform.'})
    interrupted = False
    try:
        while rclpy.ok() and time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        interrupted = True
    finally:
        emit({'event': 'summary', 'elapsed_s': round(time.monotonic() - started, 3),
              'interrupted': interrupted, 'counts': dict(node.counts),
              'pending_unmatched_clouds': len(node.clouds), 'pending_unmatched_scans': len(node.scans),
              'note': 'Only exact-stamp pairs are compared. Missing messages/pairs limit conclusions; counts are observations, not proof of physical clearance.'})
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if node.counts['stamp_pairs'] and not node.counts['comparison_errors'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
