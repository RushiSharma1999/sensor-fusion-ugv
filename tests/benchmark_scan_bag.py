"""
Compare cached/uncached core decisions on an actual ROS bag without hardware.

Accepts rosbag2 SQLite with little-endian CDR LaserScan and Twist topics.
Times are local CPU measurements; acquisition/tick times come from the bag.
The ROS adapter's wall-clock deadline is not executed. No motors are driven.
"""

import argparse
import json
from pathlib import Path
import sqlite3
import struct
import time
from types import SimpleNamespace as NS

import numpy as np
from vfh_pkg.motion_core import MotionController


class Reader:
    """Diagnostic reader for the specific little-endian CDR scan layout."""

    def __init__(self, data):
        if data[:4] != b'\0\1\0\0':
            raise ValueError('Expected little-endian CDR serialization')
        self.data, self.position = data, 4

    def take(self, fmt):
        self.position += (-(self.position - 4)) % 4
        result = struct.unpack_from('<' + fmt, self.data, self.position)
        self.position += struct.calcsize('<' + fmt)
        return result

    def string(self):
        length, = self.take('I')
        value = self.data[self.position:self.position + length]
        self.position += length
        if value[-1:] != b'\0':
            raise ValueError('Invalid CDR string')
        return value[:-1].decode()

    def floats(self):
        length, = self.take('I')
        return self.take(str(length) + 'f') if length else ()


def decode_scan(data):
    reader = Reader(data)
    sec, nanosec = reader.take('iI')
    frame = reader.string()
    names = ('angle_min', 'angle_max', 'angle_increment', 'time_increment',
             'scan_time', 'range_min', 'range_max')
    values = dict(zip(names, reader.take('7f')))
    ranges, intensities = reader.floats(), reader.floats()
    if reader.position != len(data):
        raise ValueError('Unexpected serialized scan length')
    return NS(header=NS(stamp=NS(sec=sec, nanosec=nanosec), frame_id=frame),
              ranges=ranges, intensities=intensities, **values)


def replay(events, cached):
    core = MotionController()
    if not cached:
        core._swept_geometry = core._build_swept_geometry
    costs, decisions = [], []
    begin = events[0][0]
    for stamp, topic, msg in events:
        now = (stamp - begin) / 1e9
        if topic == '/scan':
            core.observe(msg, now, stamp)
        else:
            started = time.perf_counter()
            result = core.tick(now, stamp)
            costs.append((time.perf_counter() - started) * 1000)
            decisions.append((result.command, result.selected, result.reason,
                              result.state, result.passing_side))
    percentiles = np.percentile(costs, [0, 50, 95, 100]).tolist()
    summary = {'compute_ms': dict(zip(('min', 'p50', 'p95', 'max'), percentiles)),
               'max_requested_v': max(row[0].v for row in decisions)}
    if cached:
        summary['cache'] = core._swept_geometry.cache_info()._asdict()
    return summary, decisions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    args = parser.parse_args()
    with sqlite3.connect(args.bag.resolve().as_uri() + '?mode=ro', uri=True) as database:
        topics = {row[0]: row[1:] for row in database.execute('select id,name,type from topics')}
        expected = {'/scan': 'sensor_msgs/msg/LaserScan',
                    '/diagnostic/cmd_vel_vfh': 'geometry_msgs/msg/Twist'}
        events = []
        for stamp, topic_id, data in database.execute(
                'select timestamp,topic_id,data from messages order by timestamp,id'):
            name, kind = topics[topic_id]
            if name not in expected:
                continue
            if kind != expected[name]:
                raise ValueError('Unexpected topic type: ' + kind)
            events.append((stamp, name, decode_scan(data) if name == '/scan' else None))
    if not any(row[1] == '/scan' for row in events) or not any(
            row[1] == '/diagnostic/cmd_vel_vfh' for row in events):
        raise ValueError('Both /scan and /diagnostic/cmd_vel_vfh are required')
    before, original = replay(events, False)
    after, optimized = replay(events, True)
    if original != optimized:
        raise AssertionError('Cached and uncached core decisions differ')
    print(json.dumps({'uncached': before, 'cached': after,
                      'identical_decisions': len(original),
                      'core_only_no_adapter_deadline': True}, indent=2))


if __name__ == '__main__':
    main()
