"""Synthetic geometry/state tests. Fixture dimensions are not chassis measurements."""

from dataclasses import replace
import math
import random
from types import SimpleNamespace as NS
import unittest

from esp_bridge.speed_profiles import speed_limits
from vfh_pkg.motion_core import (Command, Config, Confirmation, limit_axis, MotionController,
                                 parse_scan)
from vfh_pkg.motion_geometry import (check_polygon, make_footprint, minimum_clearance, Mount,
                                     path_clearance, pose, signed_clearance, swept_polygon)


def scan(t=100.0, distance=4.0, count=361):
    ns = round(t * 1e9)
    return NS(header=NS(stamp=NS(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000),
                        frame_id='unilidar_lidar'), angle_min=-math.pi, angle_max=math.pi,
              angle_increment=2 * math.pi / (count - 1), range_min=0.15, range_max=20.0,
              time_increment=0.0, scan_time=0.0, ranges=[distance] * count)


def obstacle(msg, x, y, half_width=0):
    index = round((math.atan2(y, x) - msg.angle_min) / msg.angle_increment)
    for i in range(max(0, index - half_width), min(len(msg.ranges), index + half_width + 1)):
        msg.ranges[i] = math.hypot(x, y)
    return msg


def polygon_config(**kwargs):
    return Config(geometry_mode='polygon', mounting_calibrated=True, safety_margin=0.05,
                  footprint_xy=((-0.25, -0.18), (0.25, -0.18), (0.25, 0.18), (-0.25, 0.18)),
                  **kwargs)


def feed(core, t, msg=None):
    msg = scan(t) if msg is None else msg
    core.observe(msg, t, round(t * 1e9))
    return core.tick(t, round(t * 1e9))


def release(core, t=100.0, factory=scan):
    result = None
    for i in range(5):
        now = t + i * 0.16
        result = feed(core, now, factory(now))
    return result


class GeometryTests(unittest.TestCase):
    def test_vectorized_clearance_matches_scalar_signed_distances(self):
        shape = make_footprint('legacy_envelope', (), .2, .16)
        rng = random.Random(731)
        for _ in range(100):
            points = [(rng.uniform(-1, 1), rng.uniform(-1, 1)) for _ in range(5)]
            self.assertAlmostEqual(minimum_clearance(points, shape),
                                   min(signed_clearance(p, shape) for p in points))

    def test_projection_regression_and_real_hazard(self):
        cfg = Config()
        footprint = make_footprint(cfg.geometry_mode, (), cfg.robot_radius, cfg.safety_margin)
        beside = (0.001, 0.38)
        self.assertGreater(signed_clearance(beside, footprint), 0)
        self.assertTrue(math.isinf(path_clearance([beside], footprint)))
        self.assertAlmostEqual(path_clearance(
            [(1, 0)], footprint), 1 - max(p[0] for p in footprint))
        core = MotionController()
        release(core)
        d = feed(core, 101, obstacle(scan(101), 0.001, 0.35))
        self.assertEqual(d.command, Command())
        self.assertEqual(d.reason, 'observed_hazard_inside_footprint_envelope')
        self.assertGreater(d.raw_front, 3)

    def test_straight_left_right_complete_sweeps(self):
        cfg = polygon_config(enable_curves=True, motion_model_calibrated=True)
        core = MotionController(cfg)
        feed(core, 100)
        for command in (Command(.05, 0), Command(.025, .12), Command(.025, -.12), Command(0, .25)):
            self.assertTrue(core.check(command).usable, command)
        for side in (1, -1):
            turn = Command(.025, side * .12)
            swept = swept_polygon(core.footprint, turn.v, turn.w, 4, .25)
            # A corner late in the curve, outside the initial footprint.
            late = Mount(*pose(turn.v, turn.w, 4)).apply((.29, side * .22))
            self.assertGreater(signed_clearance(late, core.footprint), 0)
            self.assertLessEqual(signed_clearance(late, swept), 0)
            t = 101 if side == 1 else 102
            feed(core, t, obstacle(scan(t), *late))
            self.assertFalse(core.check(turn).usable)

    def test_rear_corner_during_stationary_turn(self):
        core = MotionController(polygon_config())
        msg = obstacle(scan(), -.28, .25)
        feed(core, 100, msg)
        self.assertGreater(signed_clearance(core.scan.points[round(
            (math.atan2(.25, -.28) + math.pi) / (math.pi / 180))], core.footprint), 0)
        self.assertFalse(core.check(Command(0, -.25)).usable)

    def test_between_samples_covered_even_with_coarse_steps(self):
        shape = ((-.2, -.1), (.2, -.1), (.2, .1), (-.2, .1))
        for w in (-.25, 0, .25):
            swept = swept_polygon(shape, .08, w, 4, 1)
            for i in range(401):
                transform = Mount(*pose(.08, w, i * .01))
                for p in shape:
                    self.assertLessEqual(signed_clearance(transform.apply(p), swept), 1e-8)

    def test_model_error_padding_covers_perturbed_motion(self):
        shape = ((-.2, -.1), (.2, -.1), (.2, .1), (-.2, .1))
        swept = swept_polygon(shape, .05, .12, 4, .25, linear_error=.01, angular_error=.03)
        for v in (.04, .06):
            for w in (.09, .15):
                for i in range(81):
                    for p in shape:
                        self.assertLessEqual(signed_clearance(
                            Mount(*pose(v, w, i * .05)).apply(p), swept), 1e-8)

    def test_mount_transform_applied_once_in_both_directions(self):
        cfg = polygon_config(lidar_x=.12, lidar_y=-.07, lidar_yaw=.4)
        raw = scan()
        data = parse_scan(raw, 100, 100_000_000_000, 1, cfg)
        expected = Mount(.12, -.07, .4).apply((-4, 0))
        self.assertAlmostEqual(data.points[0][0], expected[0])
        self.assertAlmostEqual(data.points[0][1], expected[1])
        self.assertAlmostEqual(data.mount.inverse(data.points[0])[0], -4)
        self.assertTrue(MotionController(cfg).footprint)
        self.assertTrue(
            check_polygon(
                data, make_footprint(
                    'polygon', cfg.footprint_xy, .2, .05), math.radians(10)).usable)

    def test_tf_required_and_legacy_does_not_apply_offset(self):
        cfg = polygon_config(transform_mode='tf')
        core = MotionController(cfg)
        self.assertFalse(core.observe(scan(), 100, 100_000_000_000))
        self.assertEqual(core.tick(100, 100_000_000_000).command, Command())
        self.assertTrue(core.observe(scan(101), 101, 101_000_000_000, Mount(.1, .02, .3)))
        with self.assertRaises(ValueError):
            Config(lidar_x=.1)
        with self.assertRaises(ValueError):
            Config(enable_curves=True)
        with self.assertRaises(ValueError):
            polygon_config(scan_frame='base_link', lidar_x=.1)


class ObservationTests(unittest.TestCase):
    def test_changed_data_reusing_a_stamp_is_rejected_and_stops(self):
        core = MotionController()
        release(core)
        last = core.scan.received
        conflicting = obstacle(scan(last), .25, 0)
        self.assertFalse(core.observe(conflicting, last + .04, round((last + .04) * 1e9)))
        self.assertEqual(core.last_accepted, last)
        self.assertEqual(core.tick(last + .04, round((last + .04) * 1e9)).command, Command())
        self.assertEqual(core.input_fault, 'conflicting_duplicate_observation')

    def test_empty_infinite_nan_sparse_and_front_unknown(self):
        for kind in ('empty', 'infinite', 'nan', 'sparse', 'front_unknown', 'side_unknown'):
            with self.subTest(kind=kind):
                core = MotionController()
                msg = scan()
                if kind == 'empty':
                    msg.ranges = []
                if kind == 'infinite':
                    msg.ranges = [math.inf] * 361
                if kind == 'nan':
                    msg.ranges = [math.nan] * 361
                if kind == 'sparse':
                    msg = scan(count=13)
                if kind == 'front_unknown':
                    msg.ranges[165:196] = [math.inf] * 31
                if kind == 'side_unknown':
                    msg.ranges[250:280] = [math.inf] * 30
                d = release(core, factory=lambda t: stamp_at(msg, t))
                self.assertEqual(d.command, Command())
                self.assertIn(d.state, ('stopped', 'latched'))

    def test_malformed_metadata_and_untrusted_close_returns(self):
        variants = {'angle_increment': 0, 'angle_min': math.nan, 'angle_max': 0,
                    'range_max': .1, 'time_increment': -.1, 'scan_time': math.inf}
        for key, value in variants.items():
            core = MotionController()
            msg = scan()
            setattr(msg, key, value)
            self.assertFalse(core.observe(msg, 100, 100_000_000_000), key)
        for value in (0, .1, 'bad'):
            core = MotionController()
            msg = scan()
            msg.ranges[220] = value
            self.assertFalse(core.observe(msg, 100, 100_000_000_000))
        msg = scan()
        msg.header.frame_id = 'wrong'
        self.assertFalse(MotionController().observe(msg, 100, 100_000_000_000))

    def test_converter_ceil_bin_contract(self):
        msg = scan()
        msg.ranges.pop()
        self.assertTrue(MotionController().observe(msg, 100, 100_000_000_000))
        msg.ranges.pop()
        self.assertFalse(MotionController().observe(msg, 100, 100_000_000_000))

    def test_duplicate_delay_out_of_order_zero_and_future(self):
        core = MotionController()
        self.assertTrue(core.observe(scan(), 100, 100_000_000_000))
        self.assertFalse(core.observe(scan(), 100.1, 100_100_000_000))
        self.assertEqual(core.last_accepted, 100)
        self.assertEqual(core.sequence, 1)
        self.assertFalse(core.observe(scan(99.9), 100.15, 100_150_000_000))
        self.assertEqual(core.tick(100.15, 100_150_000_000).command, Command())
        for t in (0, 99, 101):
            self.assertFalse(core.observe(scan(t), 100.2, 100_200_000_000))

    def test_control_ticks_and_duplicate_traffic_do_not_refresh_scan(self):
        core = MotionController()
        release(core)
        stamp = core.scan.stamp_ns
        last = core.scan.received
        for i in range(1, 16):
            now = last + i * .04
            core.observe(scan(stamp * 1e-9), now, round(now * 1e9))
            d = core.tick(now, round(now * 1e9))
            if now - last > .35:
                self.assertEqual(d.command, Command())
        self.assertEqual(core.last_accepted, last)
        self.assertGreater(core.last_publication, core.last_accepted)

    def test_confirmation_needs_time_and_distinct_fresh_observations(self):
        cfg = Config(confirm_sec=.2, confirm_scans=3)
        core = MotionController(cfg)
        first = scan()
        core.observe(first, 100, 100_000_000_000)
        for t in (100, 100.04, 100.08, 100.24):
            self.assertEqual(core.tick(t, round(t * 1e9)).command, Command())
        self.assertEqual(core.gate.count, 1)
        self.assertEqual(feed(core, 100.25).command, Command())
        self.assertGreater(feed(core, 100.30).command.v, 0)
        c = Confirmation()
        observation = core.scan
        for _ in range(20):
            self.assertFalse(c.update(observation, True, .2, 2))
        self.assertEqual(c.count, 1)


def stamp_at(msg, t):
    ns = round(t * 1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)
    return msg


class DecisionTests(unittest.TestCase):
    def test_wire_resolution_preserves_normal_deceleration_limit(self):
        core = MotionController()
        for i in range(30):
            feed(core, 100 + i * .04)
        self.assertEqual(core.previous.v, .05)
        previous = core.previous.v
        for i in range(10):
            t = 101.2 + i * .04
            d = feed(core, t, obstacle(scan(t), 1.0, 0, 3))
            self.assertGreater(d.command.v, 0)
            self.assertLessEqual(previous - d.command.v, .15 * .04 + 1e-9)
            previous = d.command.v

    def test_smoothing_and_immediate_hazard_precedence(self):
        core = MotionController()
        release(core)
        previous = core.previous
        for i in range(1, 6):
            t = 100.64 + i * .04
            d = core.tick(t, round(t * 1e9))
            self.assertLessEqual(d.command.v - previous.v, .08 * .04 + 1e-9)
            previous = d.command
        d = feed(core, 100.9, obstacle(scan(100.9), .25, 0))
        self.assertEqual(d.command, Command())
        self.assertEqual(core.previous, Command())
        self.assertAlmostEqual(limit_axis(.05, 0, .04, .08, .15), .044)
        self.assertAlmostEqual(limit_axis(.2, -.25, .04, .5, .75), .17)

    def test_final_smoothed_motion_is_collision_checked(self):
        core = MotionController(polygon_config(enable_curves=True, motion_model_calibrated=True))
        release(core)
        # Actual prior turn plus a newly appearing obstacle: a straight candidate
        # is clear, but residual angular velocity after limiting sweeps a corner.
        core.previous = Command(.04, -.25)
        msg = obstacle(scan(101), -.28, .25)
        core.observe(msg, 101, 101_000_000_000)
        self.assertTrue(core.check(Command(.04, 0)).usable)
        core.cached_sequence = core.scan.sequence
        straight = Command(.04, 0)
        core.cached_candidates = [(straight, core.check(straight))]
        d = core.tick(101, 101_000_000_000)
        self.assertEqual(d.command, Command())
        self.assertTrue(d.reason.startswith('limited_command_'), d)

    def test_boundary_chatter_never_releases_or_changes_side(self):
        core = MotionController()
        release(core)
        core.side = 1
        for i, distance in enumerate((.35, .37, .35, .38, .36, .37)):
            now = 101 + i * .16
            d = feed(core, now, obstacle(scan(now), .001, distance))
            self.assertEqual(d.command, Command())
            self.assertEqual(core.side, 1)
        self.assertEqual(core.side_switches, 0)

    def test_side_persists_across_brief_data_interruption(self):
        core = MotionController()
        release(core)
        core.side = -1
        core.tick(101.1, 101_100_000_000)
        self.assertEqual(core.previous, Command())
        release(core, 101.2)
        self.assertEqual(core.side, -1)

    def test_bounded_recovery_not_reset_by_noise_or_reconfirmation(self):
        core = MotionController(replace(Config(), recovery_budget_sec=2))
        release(core)
        for i in range(20):
            now = 101 + i * .16
            distance = .35 if i % 2 else .37
            d = feed(core, now, obstacle(scan(now), .001, distance))
        self.assertTrue(core.latched)
        self.assertEqual(d.command, Command())
        self.assertIn('restart_required', d.reason)
        self.assertEqual(feed(core, 110).command, Command())

    def test_side_reassessment_requires_sustained_block_and_clear_alternative(self):
        core = MotionController()
        release(core)
        core.side = 1
        for i in range(9):
            t = 101 + i * .16
            core.observe(scan(t), t, round(t * 1e9))
            core.cached_sequence = core.scan.sequence
            right = Command(0, -.25)
            core.cached_candidates = [(right, core.check(right))]
            core.tick(t, round(t * 1e9))
            if i < 7:
                self.assertEqual(core.side, 1)
        self.assertEqual(core.side, -1)
        self.assertEqual(core.side_switches, 1)

    def test_hazard_wins_over_route_completion(self):
        core = MotionController()
        release(core)
        core.begin_recovery(100.7)
        core.route_clear.count = 100
        core.route_clear.since = 99
        d = feed(core, 101, obstacle(scan(101), 0, .3))
        self.assertEqual(d.command, Command())
        self.assertIsNotNone(core.recovery_started)

    def test_profiles_reach_matching_ceiling_only_in_open_space(self):
        for profile in ('conservative', 'open_space'):
            core = MotionController(Config(speed_profile=profile))
            for i in range(80):
                t = 100 + i * .04
                d = feed(core, t)
            self.assertAlmostEqual(d.command.v, speed_limits(profile).linear)
            self.assertEqual(core.limits.angular, .25)
            d = feed(core, 104, obstacle(scan(104), 1.4, 0, 3))
            self.assertLessEqual(d.command.v, .05)

    def test_anticipatory_curves_both_sides_slow_down(self):
        cfg = polygon_config(enable_curves=True, motion_model_calibrated=True)
        for side in (1, -1):
            core = MotionController(cfg)
            release(core)
            core.side = side
            for i in range(8):
                t = 101 + i * .16
                d = feed(core, t, obstacle(scan(t), .95, 0, 4))
            self.assertGreater(d.command.v, 0, d)
            self.assertGreater(d.command.w * side, 0, d)
            self.assertLess(d.command.v, .05)
            self.assertTrue(core.check(d.command).usable)


if __name__ == '__main__':
    unittest.main()
