"""Deterministic LiDAR-only selection, freshness, transitions and output limiting."""

from dataclasses import dataclass, field
import math

from esp_bridge.speed_profiles import speed_limits

from .motion_geometry import (check_polygon, inflate, make_footprint, minimum_clearance, Mount,
                              path_clearance, swept_polygon, TAU)


@dataclass(frozen=True)
class Config:
    speed_profile: str = 'conservative'
    control_hz: float = 25.0
    control_compute_budget_sec: float = 0.04
    scan_timeout_sec: float = 0.35
    future_tolerance_sec: float = 0.02
    scan_frame: str = 'unilidar_lidar'
    base_frame: str = 'base_link'
    geometry_mode: str = 'legacy_envelope'
    footprint_xy: tuple = ()
    robot_radius: float = 0.20
    safety_margin: float = 0.16
    transform_mode: str = 'configured'
    mounting_calibrated: bool = False
    lidar_x: float = 0.0
    lidar_y: float = 0.0
    lidar_yaw: float = 0.0
    transform_timeout_sec: float = 0.10
    enable_curves: bool = False
    motion_model_calibrated: bool = False
    linear_model_scale: float = 1.0
    angular_model_scale: float = 1.0
    linear_model_error: float = 0.0
    angular_model_error: float = 0.0
    horizon_sec: float = 4.0
    sweep_step_sec: float = 0.25
    coverage_max_gap_deg: float = 10.0
    min_front_returns: int = 3
    front_stop_dist: float = 0.65
    hard_turn_dist: float = 0.85
    front_slowdown_dist: float = 1.20
    high_speed_clearance: float = 1.80
    high_speed_footprint_clearance: float = 0.30
    release_margin: float = 0.03
    front_release_margin: float = 0.10
    confirm_sec: float = 0.20
    confirm_scans: int = 2
    linear_accel: float = 0.08
    linear_decel: float = 0.15
    angular_accel: float = 0.50
    angular_decel: float = 0.75
    recovery_budget_sec: float = 20.0
    recovery_clear_sec: float = 1.0
    recovery_clear_scans: int = 4
    side_blocked_sec: float = 1.0
    progress_timeout_sec: float = 3.0
    progress_delta: float = 0.15
    side_switch_cooldown_sec: float = 2.0
    max_side_switches: int = 2
    forward_weight: float = 3.0
    clearance_weight: float = 1.5
    continuity_weight: float = 0.25
    commitment_weight: float = 0.40

    def __post_init__(self):
        speed_limits(self.speed_profile)
        for name, value in vars(self).items():
            if isinstance(value, (int, float)) and not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        positive = (
            'control_hz',
            'control_compute_budget_sec',
            'scan_timeout_sec',
            'robot_radius',
            'horizon_sec',
            'sweep_step_sec',
            'coverage_max_gap_deg',
            'transform_timeout_sec',
            'linear_model_scale',
            'angular_model_scale',
            'linear_accel',
            'linear_decel',
            'angular_accel',
            'angular_decel',
            'confirm_sec',
            'recovery_budget_sec',
            'recovery_clear_sec',
            'side_blocked_sec',
            'progress_timeout_sec',
            'progress_delta',
            'side_switch_cooldown_sec')
        if any(getattr(self, n) <= 0 for n in positive):
            raise ValueError('Timing, model scales and rate limits must be positive')
        nonnegative = ('safety_margin', 'release_margin', 'front_release_margin',
                       'future_tolerance_sec', 'max_side_switches', 'linear_model_error',
                       'angular_model_error', 'high_speed_footprint_clearance', 'forward_weight',
                       'clearance_weight', 'continuity_weight', 'commitment_weight')
        if (not 20 <= self.control_hz <= 30 or not 0 < self.coverage_max_gap_deg <= 15
                or self.sweep_step_sec > self.horizon_sec
                or not 0 < self.front_stop_dist < self.hard_turn_dist < self.front_slowdown_dist
                or self.high_speed_clearance < self.front_slowdown_dist
                or any(getattr(self, n) < 0 for n in nonnegative)
                or type(self.max_side_switches) is not int
                or any(type(getattr(self, n)) is not int or getattr(self, n) < 1
                       for n in ('min_front_returns', 'confirm_scans', 'recovery_clear_scans'))):
            raise ValueError('Invalid enhanced controller configuration')
        if self.transform_mode not in ('configured', 'tf'):
            raise ValueError('transform_mode must be configured or tf')
        if not self.scan_frame or not self.base_frame:
            raise ValueError('Frame names cannot be empty')
        if self.geometry_mode == 'legacy_envelope':
            if self.transform_mode != 'configured' or any(
                    (self.lidar_x, self.lidar_y, self.lidar_yaw)):
                raise ValueError(
                    'Legacy envelope is scan-anchored; select polygon for mounting transforms')
        elif (self.geometry_mode == 'polygon' and self.transform_mode == 'configured'
              and not self.mounting_calibrated):
            raise ValueError('Polygon configured transform requires mounting_calibrated')
        if (self.geometry_mode == 'polygon' and self.transform_mode == 'configured'
                and self.scan_frame == self.base_frame
                and any((self.lidar_x, self.lidar_y, self.lidar_yaw))):
            raise ValueError(
                'Scan already in base_frame: configured mounting offset would be applied twice')
        if self.enable_curves and (
                self.geometry_mode != 'polygon' or not self.motion_model_calibrated):
            raise ValueError('Curves require polygon geometry and calibrated motion-model bounds')
        make_footprint(
            self.geometry_mode,
            self.footprint_xy,
            self.robot_radius,
            self.safety_margin)


@dataclass(frozen=True)
class Scan:
    stamp_ns: int
    received: float
    source_age: float
    sequence: int
    points: tuple
    beams: tuple
    mount: Mount
    raw_front: float
    front_count: int
    side_left: float
    side_right: float


def parse_scan(msg, received, ros_now_ns, sequence, config, mount=None):
    """
    Validate system-time headers and LaserScan metadata before accepting data.

    The active Unitree driver defaults use_system_timestamp=true, and the
    converter copies that header. Zero stamps/clock mismatch are not supported.
    pointcloud_to_laserscan may allocate ceil(span/increment) bins; accept its
    final bin up to one increment below angle_max, with float32 rounding slack.
    """
    stamp = msg.header.stamp
    if (type(stamp.sec) is not int or type(stamp.nanosec) is not int
            or not 0 <= stamp.nanosec < 1_000_000_000):
        raise ValueError('invalid_stamp')
    stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
    age = (ros_now_ns - stamp_ns) * 1e-9
    if stamp_ns <= 0 or age > config.scan_timeout_sec or age < -config.future_tolerance_sec:
        raise ValueError('source_stamp_stale_or_invalid')
    if msg.header.frame_id != config.scan_frame:
        raise ValueError('scan_frame_mismatch')
    fields = (msg.angle_min, msg.angle_max, msg.angle_increment, msg.range_min,
              msg.range_max, msg.time_increment, msg.scan_time)
    if not all(math.isfinite(x) for x in fields):
        raise ValueError('invalid_scan_metadata')
    n = len(msg.ranges)
    span = msg.angle_max - msg.angle_min
    step = msg.angle_increment
    if (not 2 <= n <= 20000 or step <= 0 or not 0 < span <= TAU + step
            or msg.range_min < 0 or msg.range_max <= msg.range_min
            or msg.time_increment < 0 or msg.scan_time < 0
            or abs(msg.angle_min + (n - 1) * step - msg.angle_max) > step * 1.01 + 1e-6):
        raise ValueError('invalid_scan_metadata')
    if config.geometry_mode == 'legacy_envelope':
        mount = Mount()
    elif config.transform_mode == 'configured':
        mount = Mount(config.lidar_x, config.lidar_y, config.lidar_yaw)
    elif mount is None:
        raise ValueError('transform_unavailable_or_stale')
    if not all(math.isfinite(v) for v in vars(mount).values()):
        raise ValueError('invalid_transform')
    points, front, left, right, beams = [], [], [], [], {}
    for i, raw in enumerate(msg.ranges):
        r = float(raw)
        if math.isfinite(r) and r < msg.range_min:
            raise ValueError('return_below_declared_minimum')
        if not math.isfinite(r) or r <= 0 or not msg.range_min <= r <= msg.range_max:
            continue
        angle = msg.angle_min + i * step
        point = mount.apply((r * math.cos(angle), r * math.sin(angle)))
        if not all(math.isfinite(value) for value in point):
            raise ValueError('nonfinite_transformed_point')
        points.append(point)
        a = math.atan2(point[1], point[0])
        if abs(a) <= math.radians(15):
            front.append(r)
        if math.radians(20) <= a <= math.radians(100):
            left.append(r)
        if math.radians(-100) <= a <= math.radians(-20):
            right.append(r)
        key = round(angle % TAU, 10)
        beams[key] = min(r, beams.get(key, math.inf))
    return Scan(stamp_ns, received, age, sequence, tuple(points), tuple(sorted(beams.items())),
                mount, min(front, default=math.nan), len(front),
                min(left, default=math.nan), min(right, default=math.nan))


@dataclass
class Confirmation:
    since: float = None
    count: int = 0
    sequence: int = -1

    def reset(self):
        self.since, self.count, self.sequence = None, 0, -1

    def update(self, scan, condition, seconds, observations):
        if not condition:
            self.reset()
            return False
        if scan.sequence != self.sequence:
            self.since = scan.received if self.since is None else self.since
            self.count += 1
            self.sequence = scan.sequence
        # Time is measured between accepted observations, never control ticks.
        return self.count >= observations and scan.received - self.since >= seconds - 1e-9


@dataclass(frozen=True)
class Command:
    v: float = 0.0
    w: float = 0.0


@dataclass
class Decision:
    command: Command = field(default_factory=Command)
    selected: Command = field(default_factory=Command)
    reason: str = 'waiting_for_scan'
    state: str = 'stopped'
    raw_front: float = math.nan
    front_returns: int = 0
    path_clearance: float = math.nan
    footprint_clearance: float = math.nan
    side_left: float = math.nan
    side_right: float = math.nan
    side_left_clearance: float = math.nan
    side_right_clearance: float = math.nan
    scan_age: float = math.inf
    coverage: float = 0.0
    swept_clearance: float = math.nan
    passing_side: int = 0
    recovery_age: float = 0.0
    observation: int = 0
    observation_status: str = 'none'
    confirm_count: int = 0


def limit_axis(previous, target, dt, accel, decel):
    """Decelerate through zero before reversing angular direction."""
    if previous * target < 0:
        target = 0.0
    rate = accel if abs(target) > abs(previous) else decel
    return previous + max(-rate * dt, min(rate * dt, target - previous))


def wire_resolution(previous, limited):
    """Quantize toward the previous output so rounding cannot exceed rate limits."""
    if limited >= previous:
        return math.floor(limited * 1000 + 1e-9) / 1000
    return math.ceil(limited * 1000 - 1e-9) / 1000


class MotionController:
    """All decisions use LiDAR evidence; no command integration claims progress."""

    def __init__(self, config=Config()):
        self.cfg = config
        self.limits = speed_limits(config.speed_profile)
        self.conservative_speed = speed_limits('conservative').linear
        self.footprint = make_footprint(config.geometry_mode, config.footprint_xy,
                                        config.robot_radius, config.safety_margin)
        self.scan = None
        self.last_arrival = None
        self.last_accepted = None
        self.last_stamp_ns = 0
        self.sequence = 0
        self.input_fault = 'waiting_for_scan'
        self.observation_status = 'none'
        self.previous = Command()
        self.last_tick = None
        self.last_publication = None
        self.gate = Confirmation()
        self.linear_release = Confirmation()
        self.route_clear = Confirmation()
        self.motion_released = False
        self.linear_latched = True
        self.side = 0
        self.side_blocked_since = None
        self.last_side_switch = -math.inf
        self.side_switches = 0
        self.recovery_started = None
        self.progress_highwater = 0.0
        self.last_progress_evidence = None
        self.latched = False
        self.cached_sequence = -1
        self.cached_candidates = []
        self.check_age = 0.0
        self.metrics_sequence = -1
        self.metrics = None

    def invalidate(self, reason):
        self.input_fault = reason
        self.motion_released = False
        self.linear_latched = True
        self.gate.reset()
        self.linear_release.reset()
        self.route_clear.reset()
        self.previous = Command()

    def observe(self, msg, now, ros_now_ns, mount=None):
        self.last_arrival = now
        try:
            scan = parse_scan(msg, now, ros_now_ns, self.sequence + 1, self.cfg, mount)
            if scan.stamp_ns == self.last_stamp_ns:
                if (scan.points != self.scan.points or scan.beams != self.scan.beams
                        or scan.mount != self.scan.mount):
                    raise ValueError('conflicting_duplicate_observation')
                # Duplicate delivery does not refresh acceptance, confirmation or age.
                self.observation_status = 'duplicate_ignored'
                return False
            if scan.stamp_ns < self.last_stamp_ns:
                raise ValueError('out_of_order_observation')
        except (ValueError, TypeError, AttributeError, OverflowError) as exc:
            self.observation_status = str(exc)
            self.invalidate(str(exc))
            return False
        self.scan = scan
        self.sequence = scan.sequence
        self.last_stamp_ns = scan.stamp_ns
        self.last_accepted = now
        self.input_fault = None
        self.observation_status = 'accepted'
        return True

    def check(self, command):
        swept = swept_polygon(self.footprint, command.v, command.w, self.cfg.horizon_sec,
                              self.cfg.sweep_step_sec, self.cfg.linear_model_scale,
                              self.cfg.angular_model_scale, self.cfg.linear_model_error,
                              self.cfg.angular_model_error)
        # The robot can have moved since acquisition. Bound that displacement;
        # this is uncertainty padding, never claimed measured progress.
        radius = max(math.hypot(*p) for p in self.footprint)
        age_speed = (self.limits.linear *
                     self.cfg.linear_model_scale +
                     self.cfg.linear_model_error +
                     radius *
                     (self.limits.angular *
                      self.cfg.angular_model_scale +
                      self.cfg.angular_model_error))
        swept = inflate(swept, (self.check_age + self.cfg.control_compute_budget_sec) * age_speed)
        return check_polygon(self.scan, swept, math.radians(self.cfg.coverage_max_gap_deg))

    def begin_recovery(self, now):
        if self.recovery_started is None:
            self.recovery_started = now
            self.last_progress_evidence = now
            self.progress_highwater = 0.0

    def stop(self, decision, reason, now):
        self.previous = Command()
        decision.command = Command()
        decision.reason = reason
        decision.state = 'latched' if self.latched else 'stopped'
        decision.passing_side = self.side
        decision.recovery_age = 0 if self.recovery_started is None else now - self.recovery_started
        decision.confirm_count = self.gate.count
        self.last_publication = now
        return decision

    def candidates(self, path, current_clearance):
        cfg = self.cfg
        # These are requested velocities. Full speed needs open straight evidence.
        front_position = max(p[0] for p in self.footprint)
        front_equivalent = path + front_position
        distance = min(self.scan.raw_front, front_equivalent)
        open_space = (distance >= cfg.high_speed_clearance and
                      current_clearance >= cfg.high_speed_footprint_clearance)
        straight_cap = self.limits.linear if open_space else min(
            self.conservative_speed, self.limits.linear)
        scale = max(0.0, min(1.0, (distance - cfg.front_stop_dist) /
                             (cfg.front_slowdown_dist - cfg.front_stop_dist)))
        v = straight_cap * scale
        motions = []
        if not self.linear_latched and v >= 0.001:
            motions.append(Command(v, 0.0))
            if cfg.enable_curves:
                for w in (0.06, -0.06, 0.12, -0.12, 0.20, -0.20):
                    turn_v = min(self.conservative_speed, v) / \
                        (1 + 2 * abs(w) / self.limits.angular)
                    motions.append(Command(turn_v, w))
        motions.extend((Command(0, self.limits.angular), Command(0, -self.limits.angular),
                        Command(0, self.limits.angular / 2), Command(0, -self.limits.angular / 2)))
        return [(motion, self.check(motion)) for motion in motions]

    def score(self, item):
        motion, checked = item
        cfg = self.cfg
        # Clearance is capped, so distant walls do not outweigh all progress.
        progress = motion.v / self.limits.linear
        turn_progress = 0.4 * abs(motion.w) / self.limits.angular if motion.v == 0 else 0.0
        continuity = abs(motion.w - self.previous.w) / self.limits.angular
        commit = self.side != 0 and motion.w * self.side > 0
        # Begin gentle steering before the straight motion becomes impossible.
        # Only checked curves receive this preference, and its magnitude grows
        # toward hard_turn_dist. Clearance and continuity select the gentle side.
        front = self.current_front
        anticipation = 0.0
        if cfg.enable_curves and motion.v > 0 and motion.w:
            anticipation = 2.0 * max(0.0, min(1.0, (cfg.front_slowdown_dist - front) /
                                              (cfg.front_slowdown_dist - cfg.hard_turn_dist)))
        return (cfg.forward_weight * progress
                + cfg.clearance_weight * min(1.0, checked.clearance)
                - cfg.continuity_weight * continuity
                + cfg.commitment_weight * commit + anticipation + turn_progress)

    def tick(self, now, ros_now_ns):
        cfg = self.cfg
        dt = (1 / cfg.control_hz if self.last_tick is None
              else max(0.0, min(2 / cfg.control_hz, now - self.last_tick)))
        self.last_tick = now
        d = Decision(passing_side=self.side, observation=self.sequence,
                     observation_status=self.observation_status)
        if (self.recovery_started is not None
                and now - self.recovery_started >= cfg.recovery_budget_sec):
            self.latched = True
        if self.latched:
            return self.stop(d, 'recovery_budget_exhausted_restart_required', now)
        if self.scan is None or self.input_fault:
            if self.last_accepted is not None:
                self.begin_recovery(now)
            return self.stop(d, self.input_fault or 'waiting_for_scan', now)
        scan = self.scan
        d.raw_front, d.side_left, d.side_right = scan.raw_front, scan.side_left, scan.side_right
        d.front_returns = scan.front_count
        d.scan_age = max(now - scan.received + max(0, scan.source_age),
                         (ros_now_ns - scan.stamp_ns) * 1e-9)
        self.check_age = max(0.0, d.scan_age)
        if d.scan_age > cfg.scan_timeout_sec or (
                ros_now_ns - scan.stamp_ns) * 1e-9 < -cfg.future_tolerance_sec:
            self.invalidate('scan_timeout_or_clock_jump')
            self.begin_recovery(now)
            return self.stop(d, self.input_fault, now)
        if scan.front_count < cfg.min_front_returns:
            self.invalidate('front_evidence_missing')
            self.begin_recovery(now)
            return self.stop(d, self.input_fault, now)
        if self.metrics_sequence != scan.sequence:
            side_points = [[p for p in scan.points
                            if math.radians(20) <= sign * math.atan2(p[1], p[0])
                            <= math.radians(100)] for sign in (1, -1)]
            self.metrics = (minimum_clearance(scan.points, self.footprint),
                            path_clearance(scan.points, self.footprint),
                            *(minimum_clearance(points, self.footprint) if points else math.nan
                              for points in side_points))
            self.metrics_sequence = scan.sequence
        current, path, d.side_left_clearance, d.side_right_clearance = self.metrics
        d.path_clearance, d.footprint_clearance = path, current
        if current <= 0:
            self.invalidate('observed_hazard_inside_footprint_envelope')
            self.begin_recovery(now)
            return self.stop(d, self.input_fault, now)
        stationary_check = self.check(Command())
        d.coverage = stationary_check.coverage
        if not stationary_check.usable:
            self.invalidate(stationary_check.reason)
            self.begin_recovery(now)
            return self.stop(d, self.input_fault, now)
        front_position = max(p[0] for p in self.footprint)
        front = min(scan.raw_front, path + front_position)
        self.current_front = front
        if front <= cfg.front_stop_dist:
            self.linear_latched = True
            self.linear_release.reset()
            self.previous = Command(0, self.previous.w)
        elif self.linear_latched:
            if self.linear_release.update(
                    scan,
                    front >= cfg.front_stop_dist +
                    cfg.front_release_margin,
                    cfg.confirm_sec,
                    cfg.confirm_scans):
                self.linear_latched = False
        if not self.motion_released:
            self.motion_released = self.gate.update(scan, current >= cfg.release_margin,
                                                    cfg.confirm_sec, cfg.confirm_scans)
            if not self.motion_released:
                return self.stop(d, 'confirming_usable_perception', now)
        if scan.sequence != self.cached_sequence:
            self.cached_candidates = self.candidates(path, current)
            self.cached_sequence = scan.sequence
        viable = [item for item in self.cached_candidates if item[1].usable
                  and (not self.linear_latched or item[0].v == 0)]
        # Perception-space progress evidence only. Ignore tiny range improvements
        # and never let them extend the absolute recovery deadline.
        if self.recovery_started is not None and scan.sequence != getattr(
                self, 'progress_sequence', -1):
            metric = min(cfg.front_slowdown_dist, front)
            if metric >= self.progress_highwater + cfg.progress_delta:
                self.progress_highwater = metric
                self.last_progress_evidence = now
            self.progress_sequence = scan.sequence
        committed = [item for item in viable if not self.side or item[0].w * self.side >= 0]
        if self.side:
            if committed:
                self.side_blocked_since = None
            elif self.side_blocked_since is None:
                self.side_blocked_since = now
            blocked = self.side_blocked_since is not None and now - \
                self.side_blocked_since >= cfg.side_blocked_sec
            stalled = (self.last_progress_evidence is not None and
                       now - self.last_progress_evidence >= cfg.progress_timeout_sec)
            alternative = [item for item in viable if item[0].w * self.side < 0]
            materially_better = alternative and (
                not committed or max(
                    self.score(x) for x in alternative) > max(
                    self.score(x) for x in committed) + 0.2)
            if ((blocked or stalled) and materially_better
                    and now - self.last_side_switch >= cfg.side_switch_cooldown_sec
                    and self.side_switches < cfg.max_side_switches):
                self.side *= -1
                self.side_switches += 1
                self.last_side_switch = now
                self.side_blocked_since = None
                committed = [x for x in viable if x[0].w * self.side >= 0]
        if not committed:
            self.begin_recovery(now)
            reason = ('committed_side_blocked' if viable
                      else 'no_usable_motion_check_coverage_and_clearance')
            return self.stop(d, reason, now)
        selected, checked = max(committed, key=self.score)
        constrained = front < cfg.front_slowdown_dist or selected.w != 0
        if constrained:
            self.begin_recovery(now)
            self.route_clear.reset()
            if self.side == 0 and selected.w:
                self.side = 1 if selected.w > 0 else -1
        elif self.recovery_started is not None:
            # Route availability is observable; traversed distance is not.
            if self.route_clear.update(scan, current >= cfg.release_margin,
                                       cfg.recovery_clear_sec, cfg.recovery_clear_scans):
                self.recovery_started = None
                self.side = self.side_switches = 0
                self.side_blocked_since = None
        limited = Command(
            limit_axis(
                self.previous.v,
                selected.v,
                dt,
                cfg.linear_accel,
                cfg.linear_decel),
            limit_axis(
                self.previous.w,
                selected.w,
                dt,
                cfg.angular_accel,
                cfg.angular_decel))
        # Match the bridge's three-decimal wire resolution before the final check.
        bounded_v = max(0, min(self.limits.linear, limited.v))
        bounded_w = max(-self.limits.angular, min(self.limits.angular, limited.w))
        limited = Command(wire_resolution(self.previous.v, bounded_v),
                          wire_resolution(self.previous.w, bounded_w))
        if (limited.w or front < cfg.high_speed_clearance
                or current < cfg.high_speed_footprint_clearance):
            limited = Command(min(self.conservative_speed, limited.v), limited.w)
        if not cfg.enable_curves and limited.v > 0 and limited.w != 0:
            # Brake angular motion before uncalibrated straight travel.
            limited = Command(0, limited.w)
        final = self.check(limited)
        d.selected, d.coverage, d.swept_clearance = selected, final.coverage, final.clearance
        if not final.usable:
            self.begin_recovery(now)
            return self.stop(d, 'limited_command_' + final.reason, now)
        self.previous = limited
        self.last_publication = now
        d.command = limited
        d.state = 'recovering' if self.recovery_started is not None else 'cruise'
        d.reason = (
            'curve' if limited.v and limited.w else 'straight' if limited.v else 'stationary_turn')
        if self.recovery_started is not None and not constrained:
            d.reason = 'route_confirming_motion_checked'
        d.passing_side = self.side
        d.recovery_age = 0 if self.recovery_started is None else now - self.recovery_started
        d.confirm_count = self.gate.count
        return d
