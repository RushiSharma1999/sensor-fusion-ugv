#!/usr/bin/env python3
import math
import time
import serial

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String


def yaw_to_quat(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class CmdVelToSerial(Node):
    def __init__(self, **node_kwargs):
        super().__init__('cmd_vel_to_serial', **node_kwargs)

        from .speed_profiles import declare_speed_profile
        profile = declare_speed_profile(self)

        # Params
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('rx_hz', 50.0)
        self.declare_parameter('debug_serial', False)
        self.declare_parameter('trace_commands', True)
        self.declare_parameter('firmware_debug', True)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('cmd_timeout_sec', 0.25)
        self.declare_parameter('max_linear_speed', profile.linear)
        self.declare_parameter('max_angular_speed', profile.angular)
        self.declare_parameter('allow_reverse', False)
        self.declare_parameter('ultra_timeout_sec', 0.6)

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('tof_front_topic', '/tof/front/summary')
        self.declare_parameter('ultra_raw_topic', '/esp/ultrasonic_raw')

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.rx_hz = float(self.get_parameter('rx_hz').value)
        self.debug_serial = bool(self.get_parameter('debug_serial').value)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        tof_front_topic = self.get_parameter('tof_front_topic').value
        ultra_raw_topic = self.get_parameter('ultra_raw_topic').value

        for name in ('trace_commands', 'firmware_debug', 'dry_run', 'allow_reverse'):
            setattr(self, name, bool(self.get_parameter(name).value))
        for name in ('cmd_timeout_sec', 'max_linear_speed', 'max_angular_speed', 'ultra_timeout_sec'):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
            setattr(self, name, value)
        if not (math.isfinite(self.rate_hz) and self.rate_hz > 0 and
                math.isfinite(self.rx_hz) and self.rx_hz > 0):
            raise ValueError('rate_hz and rx_hz must be positive and finite')
        if (abs(self.max_linear_speed - profile.linear) > 1e-9 or
                abs(self.max_angular_speed - profile.angular) > 1e-9):
            raise ValueError('Bridge limits must match speed_profile; '
                             'select the same profile in both nodes')
        self.started_at = time.monotonic()
        self.startup_deadline = self.started_at + 5.0
        self.stop_acknowledged = False
        self.ever_armed = False
        self.serial_fault = False
        self.last_stop_sent = -math.inf
        self.last_trace_time = -math.inf
        self.last_trace_key = None
        self.tx_sequence = 0
        self.ultra_samples = {}
        self.last_ultra_publish = -math.inf
        self.last_ultra_timestamp = 0
        # UGV_SERIAL_OPEN_FIX_V1: match the stopped-state probe's line controls.
        self.ser = serial.Serial(port=None, baudrate=baud, timeout=0.0,
                                 write_timeout=0.1, exclusive=True)
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.port = port
        self.ser.open()
        self.get_logger().info('Serial line controls: DTR=0 RTS=0; STOP handshake required')
        self.get_logger().info(f"Serial open: {port} @ {baud}")

        # cmd_vel in
        self.last_twist = Twist()
        self.last_rx_time = None
        self.sub = self.create_subscription(Twist, cmd_vel_topic, self.on_cmd_vel, 10)

        # odom out + TF
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # sensor outputs
        self.tof_front_pub = self.create_publisher(String, tof_front_topic, 10)
        self.ultra_raw_pub = self.create_publisher(String, ultra_raw_topic, 10)

        # Serial RX line buffer
        self._rx_buf = ""

        # Timers
        self.cmd_timer = self.create_timer(1.0 / self.rate_hz, self.on_cmd_timer)
        self.rx_timer = self.create_timer(1.0 / self.rx_hz, self.on_rx_timer)

        self.get_logger().warn(
            f'dry_run={self.dry_run}; limits v={self.max_linear_speed:.3f} '
            f'w={self.max_angular_speed:.3f}; waiting for firmware ACK STOP'
        )
        self.send_stop()
        self.get_logger().info(
            f"Bridge running: {cmd_vel_topic} -> serial, serial -> {odom_topic}, "
            f"{tof_front_topic}, {ultra_raw_topic}"
        )

    def on_cmd_vel(self, msg: Twist):
        self.last_twist = msg
        self.last_rx_time = time.monotonic()

    def write_serial(self, text):
        try:
            data = text.encode('ascii')
            if self.ser.write(data) != len(data):
                raise IOError('short serial write')
            return True
        except Exception as e:
            self.serial_fault = True
            self.get_logger().error(f'Serial write failed; restart bridge to re-arm: {e}')
            return False

    def send_stop(self):
        self.last_stop_sent = time.monotonic()
        self.write_serial('STOP\n')

    def on_cmd_timer(self):
        now = time.monotonic()
        self.publish_ultra_samples(now)
        if not self.stop_acknowledged or self.serial_fault:
            if not self.stop_acknowledged and now > self.startup_deadline and not self.serial_fault:
                self.serial_fault = True
                self.get_logger().error('No ACK STOP within 5 seconds; motion disabled until restart')
            if now - self.last_stop_sent >= 0.2:
                self.send_stop()
            return
        age = math.inf if self.last_rx_time is None else now - self.last_rx_time
        v_in, w_in = float(self.last_twist.linear.x), float(self.last_twist.angular.z)
        reason = 'command'
        if age > self.cmd_timeout_sec:
            v = w = 0.0
            reason = 'cmd_timeout'
        elif not math.isfinite(v_in) or not math.isfinite(w_in):
            v = w = 0.0
            reason = 'nonfinite_cmd'
        else:
            v = max(-self.max_linear_speed if self.allow_reverse else 0.0,
                    min(self.max_linear_speed, v_in))
            w = max(-self.max_angular_speed, min(self.max_angular_speed, w_in))
            if (v, w) != (v_in, w_in):
                reason = 'limited'
        if self.dry_run:
            reason = 'dry_run/' + reason
            v_wire = w_wire = 0.0
        else:
            v_wire, w_wire = v, w
        line = f'CMD v {v_wire:.3f} w {w_wire:.3f}\n'
        self.tx_sequence += 1
        sent = self.write_serial(line)
        key = (line, reason, sent)
        if self.trace_commands and (key != self.last_trace_key or now - self.last_trace_time >= 0.5):
            self.get_logger().info(
                f'TX seq={self.tx_sequence} elapsed={now-self.started_at:.3f} '
                f'age={age:.3f} in_v={v_in:.4f} in_w={w_in:.4f} '
                f'planned_v={v:.4f} planned_w={w:.4f} wire="{line.strip()}" '
                f'reason={reason} sent={int(sent)}'
            )
            self.last_trace_time, self.last_trace_key = now, key

    def on_rx_timer(self):
        try:
            data = self.ser.read(4096)
            if not data:
                return
            self._rx_buf += data.decode('ascii', errors='ignore')
        except Exception as e:
            self.serial_fault = True
            self.get_logger().error(f"Serial read failed; motion disabled until restart: {e}")
            return

        while '\n' in self._rx_buf:
            line, self._rx_buf = self._rx_buf.split('\n', 1)
            line = line.strip()
            if not line:
                continue

            if self.debug_serial:
                self.get_logger().info(f"SERIAL: {line}")

            if line == 'ACK STOP':
                if not self.stop_acknowledged and not self.serial_fault:
                    self.stop_acknowledged = True
                    self.ever_armed = True
                    # Discard commands received before the stopped-state handshake.
                    self.last_rx_time = None
                    self.get_logger().info('RX ACK STOP; accepting fresh ROS commands')
                    if self.firmware_debug:
                        self.write_serial('DEBUG 1\n')
            elif line.startswith('READY'):
                self.ultra_samples.clear()
                self.stop_acknowledged = False
                self.last_rx_time = None
                if self.ever_armed:
                    self.serial_fault = True
                    self.get_logger().error('Firmware reboot detected; restart bridge to re-arm')
                self.send_stop()
            elif line.startswith('DBG|') or line.startswith('ACK'):
                if self.firmware_debug and not self.debug_serial:
                    self.get_logger().info(f'RX elapsed={time.monotonic()-self.started_at:.3f} {line}')
            elif line.startswith('US|'):
                self.handle_us_line(line)
            elif line.startswith("ODOM"):
                self.handle_odom_line(line)
            elif line.startswith("TOF|"):
                self.handle_tof_line(line)
            elif line.startswith("ULTRA|"):
                self.handle_ultra_line(line)
            # Legacy firmware formats are retained below.
        if len(self._rx_buf) > 16384:
            self._rx_buf = ''
            self.serial_fault = True
            self.get_logger().error('Serial line exceeded buffer limit; motion disabled until restart')

    def handle_us_line(self, line):
        try:
            fields = dict(part.split(':', 1) for part in line.split('|')[1:] if ':' in part)
            sensor = fields['ID']
            if sensor not in ('FRONT', 'LEFT', 'RIGHT', 'BACK'):
                return
            cm = float(fields['CM'])
            valid = fields['VALID'] == '1' and math.isfinite(cm) and cm > 0
            timestamp = int(fields['T'])
            self.ultra_samples[sensor] = (cm if valid else 400.0, valid, time.monotonic())
            self.last_ultra_timestamp = timestamp
        except (ValueError, KeyError):
            self.get_logger().warn(f'Bad US line: {line}')

    def publish_ultra_samples(self, now):
        if now - self.last_ultra_publish < 0.1:
            return
        self.last_ultra_publish = now
        values, validity = [], []
        for sensor in ('FRONT', 'LEFT', 'RIGHT', 'BACK'):
            cm, valid, received = self.ultra_samples.get(sensor, (400.0, False, -math.inf))
            valid = valid and now - received <= self.ultra_timeout_sec
            values.append(f'{sensor}:{cm if valid else 400.0:.1f}')
            validity.append(f'VALID_{sensor}:{int(valid)}')
        msg = String()
        # 400.0 with VALID=0 is unknown, never a measured clear distance.
        msg.data = '|'.join([f'T:{self.last_ultra_timestamp}'] + values + validity)
        self.ultra_raw_pub.publish(msg)

    def close(self):
        # No further CMD writes after this point; STOP can be superseded by CMD.
        self.cmd_timer.cancel()
        self.rx_timer.cancel()
        self.write_serial('STOP\n')
        deadline = time.monotonic() + 0.5
        acknowledged = False
        pending = self._rx_buf
        try:
            while time.monotonic() < deadline:
                pending += self.ser.read(4096).decode('ascii', errors='ignore')
                lines = pending.split('\n')
                pending = lines.pop()
                if any(line.strip() == 'ACK STOP' for line in lines):
                    acknowledged = True
                    break
                time.sleep(0.01)
        except Exception as exc:
            self.serial_fault = True
            self.get_logger().error(f'Final STOP read failed: {exc}')
        finally:
            try:
                self.ser.close()
            except Exception as exc:
                self.serial_fault = True
                self.get_logger().error(f'Serial close failed: {exc}')
            finally:
                self.get_logger().info(f'Final STOP acknowledgement observed={int(acknowledged)}')

    def handle_odom_line(self, line: str):
        try:
            parts = line.split()

            def get_val(label: str) -> float:
                i = parts.index(label)
                return float(parts[i + 1])

            x = get_val("x")
            y = get_val("y")
            th = get_val("th")
            v = get_val("v")
            w = get_val("w")
        except Exception:
            self.get_logger().warn(f"Bad ODOM line: {line}")
            return

        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quat(th)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def handle_tof_line(self, line: str):
        try:
            parts = line.split('|')
            if parts[0] != "TOF":
                self.get_logger().warn(f"Bad TOF prefix: {line}")
                return

            fields = {}
            for part in parts[1:]:
                if ':' not in part:
                    continue
                key, value = part.split(':', 1)
                fields[key] = value

            sensor_id = fields.get("ID", "")
            if sensor_id != "FRONT":
                return

            timestamp_ms = int(fields.get("T", "0"))
            valid = int(fields.get("VALID", "0"))
            center = float(fields.get("CENTER", "0.0"))
            near_min = float(fields.get("NEAR_MIN", "0.0"))
            near_avg = float(fields.get("NEAR_AVG", "0.0"))
            far_avg = float(fields.get("FAR_AVG", "0.0"))
        except Exception:
            self.get_logger().warn(f"Bad TOF line: {line}")
            return

        msg = String()
        msg.data = (
            f"ID:{sensor_id}|T:{timestamp_ms}|VALID:{valid}|"
            f"CENTER:{center:.1f}|NEAR_MIN:{near_min:.1f}|"
            f"NEAR_AVG:{near_avg:.1f}|FAR_AVG:{far_avg:.1f}"
        )
        self.tof_front_pub.publish(msg)

    def handle_ultra_line(self, line: str):
        """Legacy aggregate firmware record; use the same expiry rules as US."""
        try:
            fields = dict(part.split(':', 1) for part in line.split('|')[1:] if ':' in part)
            timestamp = int(fields.get('T', '0'))
            samples = {}
            for sensor in ('FRONT', 'LEFT', 'RIGHT', 'BACK'):
                cm = float(fields.get(sensor, '400.0'))
                valid = (fields.get('VALID_' + sensor, '0') == '1'
                         and math.isfinite(cm) and cm > 0)
                samples[sensor] = (cm if valid else 400.0, valid, time.monotonic())
            self.ultra_samples.update(samples)
            self.last_ultra_timestamp = timestamp
        except ValueError:
            self.get_logger().warn(f'Bad ULTRA line: {line}')


def main():
    rclpy.init()
    node = CmdVelToSerial()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.close()
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
