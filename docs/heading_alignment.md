# Heading alignment from the confirmed front marker

The user confirmed that the robot was stationary and the marker box was centred
directly ahead in the robot's physical forward direction, about one metre from
the LiDAR. In the uploaded `ugv-front-marker.yaml`, the likely marker returns
are at 1.0406–1.0832 m and raw bearings 158–173 degrees. Their centre is near
165 degrees, rather than the controller's assumed zero-degree forward direction.
This explains how an approaching physical front obstacle could remain outside
the controller's front cone until the all-around envelope stopped motion.

The source stamp is 1794611472.251048326, frame `unilidar_lidar`. The capture has
360 bins, 81 finite returns and no finite return within 0.36 m. This identifies
a coarse heading estimate, not a precise mount pose or physical footprint.

## Selectable correction

`ws/src/vfh_pkg/config/heading_marker.yaml` enables `align_legacy_heading` and
sets `lidar_yaw=-2.8797932657906435` rad (the conversion of an initial -165-degree
estimate). The relation is:

`heading-aligned bearing = raw LiDAR bearing + lidar_yaw`

The original scan/cloud messages, frame IDs and driver/projection parameters
remain unchanged. Only the enhanced core's internal points rotate. Consequently
raw `/scan` still reports the marker near 165 degrees; it is the controller's
front/path/side diagnostics that should now follow the robot's heading.
Visibility checking applies the inverse transform once, back to the original
LiDAR rays. The cached swept geometry is in the same aligned calculation frame
as the transformed points. No offset is applied twice.

The legacy envelope remains centred at the LiDAR origin with robot_radius=0.20
and safety_margin=0.16 m. Zero internal translation is not a measurement of the
LiDAR's position relative to the chassis or its rotation pivot. The new option
does not assert `mounting_calibrated` or `motion_model_calibrated`, and curves
remain disabled. Translations, TF and the polygon mode cannot be combined with
this heading-only option. Full measured polygon/mount geometry still uses the
existing configured/TF path with `align_legacy_heading=false`.

The normal default keeps heading alignment disabled, and the baseline controller
is unchanged. The supplied heading estimate is specific to the confirmed marker
setup; verify/refine it rather than copying it to other robot mountings.
The uncorrected baseline is not a workaround for this known heading mismatch in
motor-enabled floor trials.

## Offline results

Applying the correction to the actual uploaded scan moves the five marker
bearings to approximately -7, -5, 0, +2 and +8 degrees in the calculation frame.
The controller's `raw_front` changes from 2.1171 m to 1.0406 m. Both configurations
pass spatial checks at that one-metre distance. This is offline scan geometry,
not a physical avoidance demonstration or a test of message freshness today.

Seven additional synthetic core tests cover explicit opt-in/configuration
rejection, point rotation exactly once with unchanged raw scan/frame, inverse
visibility coordinates and changed swept-motion direction, close returns at all
bearings, missing aligned-front evidence, stationary turning for a 0.60 m aligned
front obstacle, and resuming straight travel after fresh route-clear evidence.
The existing obstacle, coverage, freshness, smoothing, recovery and bridge
regressions remain applicable. A ROS middleware test loads the actual shipped
YAML and checks nonzero angular/zero linear commands for the synthetic aligned
front obstacle, without opening a serial device or running hardware.

Executed for this update: all 50 offline functional checks and all seven ROS
integration checks passed; both packages built in the available ROS 2 Jazzy
container. The four changed Python files passed targeted Flake8/PEP257 and
Python 3.10 grammar checks. The complete package suites reported 54 tests:
50 passed, two pre-existing Flake8 failures (28 findings in vfh_pkg and 54 in
esp_bridge), and two copyright skips. Jetson/Humble execution and physical
heading/turning validation remain to be performed on the robot.

## Jetson update and motors-off verification

Keep motor power off. Select the existing review branch and update without force
(Git will refuse conflicting local edits rather than overwrite them):

```bash
bash ~/sensor-fusion-ugv-old-migration/run_cloud36_test.sh --stop
cd ~/sensor-fusion-ugv-old-migration && git switch improve/lidar-swept-motion && git pull --ff-only origin improve/lidar-swept-motion
cd ~/sensor-fusion-ugv-old-migration/ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select esp_bridge vfh_pkg --executor sequential
cd ~/sensor-fusion-ugv-old-migration && bash tests/run_offline.sh
cd ~/sensor-fusion-ugv-old-migration && bash run_cloud36_test.sh --dry-run --behavior enhanced --speed-profile conservative --params "$PWD/ws/src/vfh_pkg/config/heading_marker.yaml"
```

Use the same `--params` argument in subsequent runs to select the correction.
The bare launcher defaults do not enable it. With the stationary front marker
again about one metre away, verify:

- Startup reports `align_legacy_heading=True`, yaw near -2.879793 rad, and
  `curves=False`. JSON reports `geometry_reference=lidar_origin_heading_aligned`.
- `scan_to_geometry_yaw_rad` reports the actual applied yaw. `raw_front` should
  follow the front box's measured range, around one metre, with finite front
  evidence. At one metre the controller can still request forward travel; the
  configured translation stop is 0.65 m.
- Repeat with the box physically on the left and then right. The corresponding
  aligned `side_left`/`side_right` should respond. Raw LiDAR bearings would be
  approximately -105 degrees for physical left and +75 degrees for physical
  right under this estimate. This also checks handedness; yaw alone cannot fix
  an upside-down/nonplanar mounting or reflected axes.
- With motor power still off, a box directly ahead at about 0.60 m from the LiDAR
  (where actual chassis clearance permits) should cause zero translation and
  `stationary_turn` with a nonzero angular request if all required coverage and
  swept clearance are usable. Keep the box outside the configured 0.36 m
  envelope. Inside-envelope returns must stop both axes, and unknown coverage
  or stale scans must also stop. Do not bypass a guard to obtain a turn.
- The bridge is in dry-run: all wire velocities stay zero. Check startup and
  final STOP acknowledgements and retain the vfh/bridge logs.

If the heading checks pass, follow the existing wheels-raised and conservative
floor comparison stages. At the wheels-raised motor stage, retain the profile:

```bash
cd ~/sensor-fusion-ugv-old-migration && bash run_cloud36_test.sh --motors --behavior enhanced --speed-profile conservative --params "$PWD/ws/src/vfh_pkg/config/heading_marker.yaml"
```

Stop with Ctrl+C or the existing `--stop` command. A stationary wheels-raised
robot cannot demonstrate route-opening through actual body rotation, and can
eventually reach the recovery latch; stop/restart between stages. Do not treat
the supplied yaw estimate as calibrated motion/footprint clearance or enable
open_space/curves based solely on this change. The previous 0.15 m returns remain
unexplained and are still treated as hazards; this correction does not mask them.
