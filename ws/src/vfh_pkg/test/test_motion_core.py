"""Synthetic geometry/state tests. Fixture dimensions are not chassis measurements."""

from dataclasses import replace
import math
import random
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from esp_bridge.speed_profiles import speed_limits
from vfh_pkg.motion_core import (Command, Config, Confirmation, limit_axis, MotionController,
                                 parse_scan)
from vfh_pkg.motion_geometry import (check_polygon, make_footprint, minimum_clearance, Mount,
                                     path_clearance, pose, signed_clearance, swept_polygon,
                                     wedge_radii, wedge_radius)


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
    def test_heading_alignment_is_explicit_and_cannot_invent_offsets(self):
        with self.assertRaises(ValueError):
            Config(lidar_yaw=math.pi)
        for options in ({'lidar_x': .1}, {'lidar_y': .1}, {'transform_mode': 'tf'},
                        {'scan_frame': 'base_link'}):
            with self.assertRaises(ValueError):
                Config(align_legacy_heading=True, lidar_yaw=math.pi, **options)
        with self.assertRaises(ValueError):
            polygon_config(align_legacy_heading=True)
        with self.assertRaises(ValueError):
            Config(align_legacy_heading=True, lidar_yaw=math.nan)

    def test_heading_alignment_rotates_points_once_and_preserves_raw_scan(self):
        cfg = Config(align_legacy_heading=True, lidar_yaw=math.radians(-165))
        msg = obstacle(scan(), math.cos(math.radians(165)), math.sin(math.radians(165)), 4)
        original = tuple(msg.ranges)
        data = parse_scan(msg, 100, 100_000_000_000, 1, cfg)
        marker = data.points[345]
        self.assertAlmostEqual(marker[0], 1)
        self.assertAlmostEqual(marker[1], 0)
        self.assertAlmostEqual(data.raw_front, 1)
        raw = data.mount.inverse(marker)
        self.assertAlmostEqual(math.atan2(raw[1], raw[0]), math.radians(165))
        self.assertEqual(tuple(msg.ranges), original)
        self.assertEqual(msg.header.frame_id, 'unilidar_lidar')

    def test_heading_alignment_changes_checked_motion_direction(self):
        msg = obstacle(scan(), .5 * math.cos(math.radians(165)),
                       .5 * math.sin(math.radians(165)), 3)
        unaligned = MotionController()
        aligned = MotionController(Config(align_legacy_heading=True,
                                          lidar_yaw=math.radians(-165)))
        for core in (unaligned, aligned):
            core.observe(msg, 100, 100_000_000_000)
        self.assertTrue(unaligned.check(Command(.05, 0)).usable)
        self.assertFalse(aligned.check(Command(.05, 0)).usable)
        self.assertTrue(aligned.check(Command(0, .25)).usable)
        self.assertTrue(aligned.check(Command(0, -.25)).usable)

    def test_heading_alignment_does_not_hide_any_envelope_hazard(self):
        cfg = Config(align_legacy_heading=True, lidar_yaw=math.radians(-165))
        for degrees in (-175, -90, 0, 90, 165):
            core = MotionController(cfg)
            angle = math.radians(degrees)
            result = feed(core, 100, obstacle(scan(), .3 * math.cos(angle), .3 * math.sin(angle)))
            self.assertEqual(result.command, Command())
            self.assertEqual(result.reason, 'observed_hazard_inside_footprint_envelope')
            self.assertAlmostEqual(result.last_hazard['range_m'], .3)
            self.assertAlmostEqual(result.last_hazard['bearing_deg'], degrees)

    def test_vectorized_clearance_matches_scalar_signed_distances(self):
        shape = make_footprint('legacy_envelope', (), .2, .16)
        rng = random.Random(731)
        for _ in range(100):
            points = [(rng.uniform(-1, 1), rng.uniform(-1, 1)) for _ in range(5)]
            self.assertAlmostEqual(minimum_clearance(points, shape),
                                   min(signed_clearance(p, shape) for p in points))

    def test_batched_wedges_match_scalar_across_wrap_edges_and_chunks(self):
        rng = random.Random(8401)
        for mount in (Mount(), Mount(.4, -.2, .7), Mount(-.7, .3, -2.8)):
            poly = tuple(mount.apply(p) for p in ((-.3, -.2), (.3, -.2), (.3, .2), (-.3, .2)))
            starts = [0, math.pi / 2, -math.pi, 2 * math.pi - .01]
            widths = [0, math.pi, 2 * math.pi, .02]
            starts += [rng.uniform(-2 * math.pi, 2 * math.pi) for _ in range(513)]
            widths += [rng.uniform(0, 2 * math.pi) for _ in range(513)]
            actual = wedge_radii(poly, starts, widths)
            self.assertEqual(len(actual), len(starts))
            for value, a, width in zip(actual, starts, widths):
                self.assertAlmostEqual(value, wedge_radius(poly, a, width), places=12)

    def test_batched_visibility_matches_scalar_for_offset_motion_and_sparse_scans(self):
        # Finite endpoint returns can lie outside a rectangle yet fail to
        # establish range support at its corner between adjacent beams.
        poly = ((-.2, -.1), (.2, -.1), (.2, .1), (-.2, .1))
        msg = scan(count=37)
        msg.range_min = .01
        msg.ranges = [wedge_radius(poly, msg.angle_min + i * msg.angle_increment, 0) + .001
                      for i in range(37)]
        data = parse_scan(msg, 100, 100_000_000_000, 1, Config())
        unknown = check_polygon(data, poly, math.radians(10))
        self.assertGreater(unknown.clearance, 0)
        self.assertFalse(unknown.usable)
        self.assertEqual(unknown.coverage_gap.cause, 'insufficient_range_support')
        self.assertGreater(unknown.coverage_gap.required_radius_m,
                           unknown.coverage_gap.supported_radius_m)
        rng = random.Random(620)
        for config in (Config(), polygon_config(lidar_x=.1, lidar_y=-.02, lidar_yaw=.3)):
            core = MotionController(config)
            for t in range(100, 110):
                msg = scan(t)
                msg.ranges = [rng.uniform(.38, 3) if rng.random() > .25 else math.inf
                              for _ in msg.ranges]
                core.observe(msg, t, t * 1_000_000_000)
                for command in (Command(), Command(.05, 0), Command(.025, .12),
                                Command(.025, -.12), Command(0, -.25)):
                    actual = core.check(command)
                    with patch('vfh_pkg.motion_geometry.wedge_radii',
                               side_effect=lambda poly, starts, widths: [
                                   wedge_radius(poly, a, w) for a, w in zip(starts, widths)]):
                        scalar = core.check(command)
                    self.assertEqual(actual.usable, scalar.usable)
                    self.assertEqual(actual.reason, scalar.reason)
                    self.assertEqual(actual.coverage, scalar.coverage)
                    self.assertEqual(actual.clearance, scalar.clearance)
                    if actual.coverage_gap is not None:
                        self.assertEqual(actual.coverage_gap.cause, scalar.coverage_gap.cause)
                        self.assertAlmostEqual(actual.coverage_gap.required_radius_m,
                                               scalar.coverage_gap.required_radius_m, places=12)

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
    def test_aligned_front_still_requires_its_own_finite_evidence(self):
        core = MotionController(Config(align_legacy_heading=True,
                                       lidar_yaw=math.radians(-165)))
        msg = scan()
        for i in range(len(msg.ranges)):
            angle = msg.angle_min + i * msg.angle_increment - math.radians(165)
            if abs(math.atan2(math.sin(angle), math.cos(angle))) < math.radians(16):
                msg.ranges[i] = math.inf
        result = feed(core, 100, msg)
        self.assertEqual(result.command, Command())
        self.assertEqual(result.reason, 'front_evidence_missing')

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
    def test_missing_wedge_reports_raw_sector_and_still_stops_immediately(self):
        core = MotionController(Config(align_legacy_heading=True,
                                       lidar_yaw=math.radians(-165)))
        release(core)
        msg = scan(101)
        msg.ranges[90:111] = [math.inf] * 21
        result = feed(core, 101, msg)
        self.assertEqual(result.command, Command())
        self.assertEqual(result.reason, 'coverage_unknown')
        self.assertEqual(result.finite_beams, 339)
        self.assertAlmostEqual(result.max_finite_gap_deg, 22, places=6)
        gap = result.coverage_gap
        self.assertEqual(gap.cause, 'angular_gap')
        self.assertAlmostEqual(gap.start_deg, -91, places=6)
        self.assertAlmostEqual(gap.width_deg, 22, places=6)
        self.assertGreater(gap.required_radius_m, .36)
        self.assertGreater(gap.supported_radius_m, gap.required_radius_m)
        # Plenty of range at the bounding rays cannot verify the missing wedge.
        self.assertLess(result.coverage, 1)

    def test_aligned_front_obstacle_selects_checked_stationary_turn(self):
        core = MotionController(Config(align_legacy_heading=True,
                                       lidar_yaw=math.radians(-165)))

        def scene(t):
            angle = math.radians(165)
            return obstacle(scan(t), .6 * math.cos(angle), .6 * math.sin(angle), 4)

        result = release(core, factory=scene)
        self.assertEqual(result.command.v, 0)
        self.assertGreater(abs(result.command.w), 0)
        self.assertEqual(result.reason, 'stationary_turn')
        self.assertTrue(core.check(result.command).usable)

    def test_aligned_turn_resumes_forward_when_observed_route_opens(self):
        core = MotionController(Config(align_legacy_heading=True,
                                       lidar_yaw=math.radians(-165)))

        def blocked(t):
            angle = math.radians(165)
            return obstacle(scan(t), .6 * math.cos(angle), .6 * math.sin(angle), 4)

        turning = release(core, factory=blocked)
        self.assertGreater(abs(turning.command.w), 0)
        for i in range(15):
            result = feed(core, 101 + i * .16)
            self.assertTrue(core.check(result.command).usable)
        self.assertGreater(result.command.v, 0)
        self.assertEqual(result.command.w, 0)

    def test_cached_geometry_does_not_cache_collision_coverage_or_age(self):
        core = MotionController()
        feed(core, 100)
        motion = Command(.05, 0)
        self.assertTrue(core.check(motion).usable)
        self.assertTrue(core.check(motion).usable)
        self.assertGreater(core._swept_geometry.cache_info().hits, 0)
        core.observe(obstacle(scan(101), .25, 0), 101, 101_000_000_000)
        self.assertFalse(core.check(motion).usable)
        core.observe(scan(102, distance=math.inf), 102, 102_000_000_000)
        self.assertFalse(core.check(motion).usable)
        core.observe(scan(103, distance=1.1), 103, 103_000_000_000)
        fresh = core.check(motion)
        core.check_age = .35
        older = core.check(motion)
        self.assertTrue(older.usable)
        self.assertLess(older.clearance, fresh.clearance)

    def test_cached_and_uncached_spatial_results_match(self):
        for cfg in (Config(), polygon_config()):
            cached = MotionController(cfg)
            reference = MotionController(cfg)
            reference._swept_geometry = reference._build_swept_geometry
            for i, distance in enumerate((4, 1.5, .7, .25, math.inf)):
                now = 100 + i
                msg = scan(now, distance)
                for core in (cached, reference):
                    core.observe(msg, now, round(now * 1e9))
                for age in (0, .2, .35):
                    cached.check_age = reference.check_age = age
                    for motion in (Command(), Command(.05, 0), Command(.025, .12),
                                   Command(0, .25), Command(0, -.25)):
                        self.assertEqual(cached.check(motion), reference.check(motion))

    def test_hazard_measurement_survives_repeated_stopped_ticks(self):
        core = MotionController()
        release(core)
        first = feed(core, 101, obstacle(scan(101), .25, 0))
        repeated = core.tick(101.04, 101_040_000_000)
        self.assertEqual(first.command, Command())
        self.assertEqual(repeated.command, Command())
        self.assertEqual(first.last_hazard, repeated.last_hazard)
        self.assertAlmostEqual(repeated.last_hazard['range_m'], .25)
        self.assertAlmostEqual(repeated.last_hazard['bearing_deg'], 0)
        self.assertEqual(repeated.last_hazard['stamp_ns'], 101_000_000_000)

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

    def test_intermittent_distant_coverage_loss_does_not_produce_restart_pulses(self):
        core = MotionController()
        release(core)
        core.side = 1
        for i in range(24):
            now = 101 + i * .16
            msg = scan(now, 3)
            if i % 4 == 0:
                msg.ranges[90:103] = [math.inf] * 13
            result = feed(core, now, msg)
            self.assertEqual(result.command, Command())
            self.assertIn(result.reason, ('coverage_unknown', 'perception_stability_confirming'))
            self.assertEqual(core.side, 1)
        self.assertFalse(core.latched)

    def test_perception_release_needs_time_and_distinct_scans_not_ticks(self):
        core = MotionController()
        release(core)
        bad = scan(101)
        bad.ranges[90:103] = [math.inf] * 13
        feed(core, 101, bad)
        feed(core, 101.16)
        for now in (101.20, 101.24, 101.32):
            result = core.tick(now, round(now * 1e9))
            self.assertEqual(result.command, Command())
            self.assertEqual(result.perception_confirm_count, 1)
        for now in (101.34, 101.42, 101.50, 101.58):
            result = feed(core, now)
            self.assertEqual(result.command, Command())
        self.assertEqual(result.perception_confirm_count, 5)
        for now in (101.66, 101.74, 101.82, 101.90):
            self.assertEqual(feed(core, now).command, Command())
        result = feed(core, 101.98)
        self.assertGreater(result.command.v, 0)
        self.assertTrue(core.check(result.command).usable)

    def test_perception_stability_does_not_delay_hazards_or_extend_recovery(self):
        core = MotionController(Config(recovery_budget_sec=2))
        release(core)
        started = None
        for i in range(16):
            now = 101 + i * .16
            msg = scan(now)
            if i % 4 == 0:
                msg.ranges[90:103] = [math.inf] * 13
            result = feed(core, now, msg)
            started = core.recovery_started if started is None else started
            self.assertEqual(core.recovery_started, started)
            self.assertEqual(result.command, Command())
        self.assertTrue(core.latched)
        self.assertIn('restart_required', result.reason)
        core = MotionController()
        release(core)
        core.tick(101.1, 101_100_000_000)
        result = feed(core, 101.2, obstacle(scan(101.2), .25, 0))
        self.assertEqual(result.command, Command())
        self.assertEqual(result.reason, 'observed_hazard_inside_footprint_envelope')

    def test_optional_curve_caps_reject_invalid_configuration_and_bound_candidates(self):
        for options in ({'curve_linear_speed_cap': .08}, {'curve_max_yaw_rate': .03},
                        {'curve_max_yaw_rate': .26}, {'perception_release_scans': 0}):
            with self.assertRaises(ValueError):
                Config(**options)
        cfg = polygon_config(enable_curves=True, motion_model_calibrated=True,
                             curve_linear_speed_cap=.02, curve_max_yaw_rate=.06)
        core = MotionController(cfg)
        release(core)
        motions = core.candidates(1.5, 1)
        curves = [c for c, checked in motions if c.v > 0 and c.w and checked.usable]
        self.assertEqual({c.w for c in curves}, {-.06, .06})
        self.assertTrue(all(c.v <= .02 for c in curves))
        with self.assertRaises(ValueError):
            Config(enable_curves=True, curve_linear_speed_cap=.02, curve_max_yaw_rate=.06)

    def test_final_curved_output_obeys_optional_caps_during_transitions(self):
        cfg = polygon_config(enable_curves=True, motion_model_calibrated=True,
                             curve_linear_speed_cap=.02, curve_max_yaw_rate=.06)
        for initial, target in ((Command(.05, 0), Command(.013, .06)),
                                (Command(0, .25), Command(.013, .06)),
                                (Command(.013, .06), Command(0, .25))):
            core = MotionController(cfg)
            release(core)
            core.previous = initial
            core.last_tick = 100.96
            for i in range(18):
                now = 101 + i * .04
                core.observe(scan(now), now, round(now * 1e9))
                core.cached_sequence = core.scan.sequence
                core.cached_candidates = [(target, core.check(target))]
                previous = core.previous
                result = core.tick(now, round(now * 1e9))
                self.assertLessEqual(previous.v - result.command.v, .15 * .04 + 1e-9)
                if result.command.v > 0 and result.command.w:
                    self.assertLessEqual(result.command.v, .02)
                    self.assertLessEqual(abs(result.command.w), .06)
                self.assertTrue(core.check(result.command).usable)
            if target.v:
                self.assertGreater(result.command.v, 0)
            else:
                self.assertEqual(result.command.v, 0)
            self.assertGreater(result.command.w, 0)

    def test_both_caps_hold_on_the_wire_when_a_tick_crosses_both(self):
        cfg = polygon_config(enable_curves=True, motion_model_calibrated=True,
                             curve_linear_speed_cap=.0207, curve_max_yaw_rate=.0617,
                             angular_accel=3.0)
        for sign in (-1, 1):
            for initial, target in ((Command(.05, 0), Command(0, sign * .25)),
                                    (Command(.020, sign * .06), Command(.05, 0)),
                                    (Command(.013, sign * .06), Command(0, sign * .25))):
                core = MotionController(cfg)
                release(core)
                core.previous = initial
                core.last_tick = 100.96
                now = 101.0
                core.observe(scan(now), now, round(now * 1e9))
                core.cached_candidates = [(target, core.check(target))]
                core.cached_sequence = core.sequence
                with patch.object(core, 'check', wraps=core.check) as checks:
                    output = core.tick(now, round(now * 1e9)).command
                wire = Command(float(f'{output.v:.3f}'), float(f'{output.w:.3f}'))
                self.assertEqual(checks.call_args.args, (wire,))
                self.assertEqual(output, wire)
                if initial.w == 0:
                    self.assertGreater(output.v, cfg.curve_linear_speed_cap)
                    self.assertEqual(output.w, 0)
                else:
                    self.assertGreater(wire.v, 0)
                    self.assertGreater(wire.w * sign, 0)
                    self.assertLessEqual(wire.v, cfg.curve_linear_speed_cap)
                    self.assertLessEqual(abs(wire.w), cfg.curve_max_yaw_rate)

    def test_preview_checks_both_directions_without_selecting_either(self):
        cfg = Config(preview_curves=True, curve_linear_speed_cap=.02, curve_max_yaw_rate=.06)
        core = MotionController(cfg)
        baseline = MotionController(replace(cfg, preview_curves=False))
        for i in range(16):
            t = 100 + .12 * i
            msg = obstacle(scan(t), .95, 0, 3)
            d = feed(core, t, msg)
            normal = feed(baseline, t, msg)
            self.assertEqual((d.command, d.selected), (normal.command, normal.selected))
            self.assertFalse(any(m.v and m.w for m, _ in core.cached_candidates))
        self.assertEqual({(r['w'], r['result']) for r in d.curve_checks},
                         {(-.06, 'clear'), (.06, 'clear')})
        self.assertEqual(normal.curve_checks, ())
        stopped = feed(core, 102, obstacle(scan(102), .25, 0))
        self.assertEqual(stopped.curve_checks, ())
        self.assertEqual(stopped.command, Command())
        with self.assertRaises(ValueError):
            polygon_config(enable_curves=True, motion_model_calibrated=True, preview_curves=True)

    def test_mirrored_approaches_select_curves_then_stop_on_loss_or_hazard(self):
        cfg = polygon_config(enable_curves=True, motion_model_calibrated=True,
                             curve_linear_speed_cap=.02, curve_max_yaw_rate=.06)
        for side, fault in ((1, 'coverage_unknown'), (-1, 'scan_timeout_or_clock_jump'),
                            (1, 'observed_hazard_inside_footprint_envelope')):
            core = MotionController(cfg)
            for i in range(32):
                t = 100 + .04 * i
                d = feed(core, t)
            self.assertEqual(d.command, Command(.05, 0))
            for i in range(12):
                t += .04
                msg = obstacle(obstacle(scan(t), .95, 0, 3), .55, -.38 * side, 2)
                d = feed(core, t, msg)
                if d.command.w:
                    self.assertGreater(d.command.w * side, 0)
                    self.assertLessEqual(d.command.v, .02)
                    self.assertLessEqual(abs(d.command.w), .06)
            self.assertEqual(d.reason, 'curve')
            self.assertGreater(d.command.v, 0)
            if fault == 'scan_timeout_or_clock_jump':
                d = core.tick(t + .36, round((t + .36) * 1e9))
            else:
                msg = scan(t + .04)
                if fault == 'coverage_unknown':
                    msg.ranges[90:105] = [math.inf] * 15
                else:
                    obstacle(msg, .25, 0)
                d = feed(core, t + .04, msg)
            self.assertEqual((d.reason, d.command), (fault, Command()))

    def test_normal_straight_to_turn_brakes_translation_before_rotating(self):
        core = MotionController()
        release(core)
        core.previous = Command(.05, 0.0)
        core.last_tick = 100.96
        previous = core.previous
        braking = False
        for i in range(12):
            now = 101 + i * .04
            core.observe(scan(now), now, round(now * 1e9))
            turn = Command(0, .25)
            core.cached_sequence = core.scan.sequence
            core.cached_candidates = [(turn, core.check(turn))]
            result = core.tick(now, round(now * 1e9))
            self.assertLessEqual(previous.v - result.command.v, .15 * .04 + 1e-9)
            self.assertLessEqual(abs(result.command.w - previous.w), .5 * .04 + 1e-9)
            self.assertTrue(core.check(result.command).usable)
            self.assertFalse(result.command.v > 0 and result.command.w != 0)
            if result.command.v > 0:
                self.assertEqual(result.reason, 'braking_before_stationary_turn')
                braking = True
            previous = result.command
        self.assertTrue(braking)
        self.assertEqual(result.command.v, 0)
        self.assertGreater(result.command.w, 0)

    def test_unsafe_translation_during_turn_braking_stops_immediately(self):
        core = MotionController(Config(horizon_sec=10))
        release(core)
        core.previous = Command(.05, 0.0)
        core.last_tick = 100.96
        core.observe(obstacle(scan(101), .8, 0, 3), 101, 101_000_000_000)
        turn = Command(0, .25)
        self.assertTrue(core.check(turn).usable)
        self.assertFalse(core.check(Command(.044, 0)).usable)
        core.cached_sequence = core.scan.sequence
        core.cached_candidates = [(turn, core.check(turn))]
        result = core.tick(101, 101_000_000_000)
        self.assertEqual(result.command, Command())
        self.assertEqual(result.reason, 'limited_command_swept_footprint_collision')

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
