# Jetson LiDAR-only avoidance: implementation and comparison

The preserved baseline is still the default. Enhanced mode corrects clearance
semantics, checks the full swept footprint, publishes at 25 Hz, limits normal
acceleration, uses time plus distinct scans for confirmation, and maintains a
bounded passing-side recovery. Curved avoidance is implemented and tested but
disabled until footprint, mounting and motion calibration are supplied. Requested
straight speed can explicitly opt into 0.08 m/s; the conservative profile remains
0.05 m/s and both retain 0.25 rad/s. These are command values, not measured speeds.

The source baseline is main commit `ec5e89f1474392d5fa1074ceeafa30492372b955`.
Changes live on `improve/lidar-swept-motion`. No motors, firmware flashing,
deployment, encoder tuning, camera following, sonar decisions or safety mux are
part of this implementation or its automated validation.

## Actual command path

Both shell entry points now use one implementation in `run_continuous_bot.sh`,
rooted at the script's checkout rather than a hardcoded home directory:

`unitree_lidar_ros2_node` → `/unilidar/cloud` → `pointcloud_to_laserscan_node` →
`/scan` → `vfh_pkg/vfh_node` → `/diagnostic/cmd_vel_vfh` →
`esp_bridge/cmd_vel_to_serial` → ESP32 serial.

The driver still uses UDP 192.168.1.62:6101 / 192.168.1.2:6201 and
`cloud_scan_num=36`. Projection remains `target_frame=unilidar_lidar`,
height [-0.10, 1.0] m, range [0.15, 20] m and `use_inf=true`. No projection,
driver or firmware source was changed. The old general-stack `start_bot.sh`
launches other nodes and is not the comparison launcher. `.bak` and generated
`build`/`install` copies are not implementation sources.

Previously `run_continuous_bot.sh` omitted `cloud_scan_num`, so it inherited the
driver's 18 default; the ride-log `run_cloud36_test.sh` explicitly selected 36.
Consolidation retains that tested 36 setting for both entry points.

The launcher passes the same `speed_profile` to controller and bridge. Their
ceilings come from `esp_bridge/speed_profiles.py`; the bridge rejects conflicting
explicit limits, including tighter limits that could change a checked curve's
curvature. Baseline uses its original continuous state machine with launcher
thresholds 0.65/0.85/1.2 m and minimum requested speed 0.02 m/s. Enhanced mode
has only a scan subscription: fusion, camera, sonar and odometry do not decide
its motions. The bridge still publishes existing telemetry.

## Geometry and motion assumptions

`raw_front` is the minimum original LiDAR radial range for returns whose
transformed point lies in the robot's central ±15-degree cone. `side_left` and
`side_right` are radial ranges in the transformed 20–100-degree side cones;
they are diagnostics, not turn-clearance certificates. `path_clearance` is
distance the configured inflated footprint could translate along +x before
contact. `footprint_clearance` is signed Euclidean clearance to that footprint;
zero or negative stops both axes immediately. None of these fields is the old
near-90-degree forward projection minimum.

`side_left_clearance` and `side_right_clearance` separately give signed footprint
clearance for returns in those side cones. The global footprint/swept checks
also include rear points and all other bearings.

`geometry_mode=legacy_envelope` retains the configured 0.20+0.16 m envelope
around the scan reference, conservatively circumscribed by a polygon. This
**does not assert that the LiDAR is centred or that the chassis fits that disk**.
It is the existing unverified configuration fallback, with stricter coverage
checks. Straight and stationary-turn sweeps use that reference model; accurate
corner/pivot certification requires the measured footprint and mount. Curves
cannot be enabled in this fallback. No close return is masked or discarded as
chassis. Finite returns below declared range_min invalidate the observation.

For accurate geometry select `polygon`, enter the full physical footprint
including protrusions as strictly convex CCW JSON vertices in `base_frame`,
and retain the safety margin. The origin must lie inside the polygon. Concave
footprints should be represented by their enclosing convex hull. For a measured
fixed mount choose `configured`, set x/y/yaw for the scan origin **in base_frame**
and set `mounting_calibrated=true`. Zero offsets in polygon mode must also be
measured. For TF choose `tf`: the adapter looks up base←scan at the source scan
time, rejects unavailable/stale/nonplanar transforms, and never also applies the
configured offset. Static transforms are timeless. A scan already projected
into base_frame must have an identity configured mount.

Candidate motions include straight, six forward curves (±0.06, ±0.12, ±0.20
rad/s), and stationary turns at half/full angular ceiling. Speeds decrease with
front/path clearance and curvature. Scores combine forward progress, capped
clearance, steering continuity, a gradual anticipatory curve preference and
passing-side commitment. Tie order is deterministic. The entire footprint is
transformed at trajectory interval midpoints. A displacement bound
`(|v| + footprint_radius*|w|)*interval/2` covers the space between samples;
the convex hull and outward offsets also cover corners. Conservative convex
hulls can reject arcs that an exact nonconvex sweep might permit.

The model is planar constant-Twist motion. `linear_model_scale` and
`angular_model_scale` are nominal measured command-to-motion gains. Absolute
physical velocity errors are padded over the full horizon, including centre
trajectory changes from angular error. Time since acquisition and computation
budget also add reachability padding. No commanded-motion integration is called
measured progress. Model defaults of 1 and zero error are **uncalibrated legacy
assumptions**, not physical bounds. Enable curves only with polygon geometry,
a valid mount and `motion_model_calibrated=true` after measuring gains/error
bounds, turn pivot, slip and stopping. The included YAML deliberately does not
invent calibration. Hardware-independent synthetic fixture dimensions are not
values to copy to the robot.

Coverage is explicit: finite returns must bracket every angular wedge occupied
by the swept footprint, gaps must be <= `coverage_max_gap_deg` (10 degrees),
and the whole wedge must lie nearer than the smaller finite range times
cos(gap/2). Missing bins can be interpolated only within this bounded gap;
infinite ranges never extend the observed distance and empty sectors remain
unknown. At least three central-front finite returns are separately required.
Because the current footprint occupies directions around the sensor, the default
fallback generally needs finite support around the robot, including the rear.
This is deliberately more conservative than the old two-side-minima guard and
can stop on sparse real scans. It is not proof that no thin object exists
between LiDAR rays. Check actual coverage before enabling motors; do not raise
the gap threshold or add a chassis mask to conceal missing evidence.

The final limited, wire-resolution command is collision/coverage checked again,
with reachability padding, after smoothing. With curves disabled, residual
angular motion is brought to zero before starting straight travel. Immediate
footprint hazards, invalid perception and freshness failures bypass smoothing.
The 0.65 m front threshold forbids translation; separately verified stationary
turns remain available outside the footprint envelope. Higher speed needs at
least 1.8 m front/path evidence, 0.30 m footprint clearance and zero angular
command; turns and nearby obstacles cap requests at the conservative ceiling.
Safety caps can bypass normal deceleration limits.

## Time, confirmation and bounded recovery

The active Unitree header default is `use_system_timestamp=true`; SDK parsing
sets cloud time to system time minus the packet scan period, and the converter
preserves the header. Both source age and monotonic accepted age must satisfy
0.35 s. This may be stricter than an arrival-only check when acquisition or
network delay is significant. There is no assumption of a guaranteed 6 Hz rate.
Zero/future/stale headers, clock mismatch, invalid metadata and out-of-order
observations stop conservatively. Exact duplicates do not refresh acceptance or
confirmation; a sufficiently old duplicate also fails source freshness.
Changed data reusing a previously accepted stamp is a conflicting duplicate and
stops rather than silently overriding or refreshing that observation.
Timestamp regressions may require driver/controller restart. Simulated time
must match the producer; the control timer itself uses a steady clock.

Startup/release needs both 0.20 s between accepted observations and at least two
distinct fresh scans. Repeated ticks do not count. Envelope hazards stop
immediately; release needs an additional 0.03 m, so crossing 0.36 m does not
chatter between movement and stops. Translation has its own 0.10 m front-release
margin. Maneuver completion can confirm a clear route while an independently
checked motion continues; there is no mandatory completion brake.

The passing side survives data interruptions. Reassessment requires either a
continuously blocked committed side for 1 s or 3 s without material observable
clearance improvement, an independently usable opposite candidate scoring at
least 0.2 better, a 2 s switch cooldown, and no more than two switches. Progress
evidence means >=0.15 m improvement in the observed front/path metric, not robot
displacement. It does not extend the absolute 20 s recovery budget. Only a
usable straight route sustained for 1 s and four observations ends that recovery
episode. Tiny range changes and failed confirmations cannot reset its deadline.
Budget exhaustion latches zero until controller restart. On startup with no
accepted observation, the node can wait stopped indefinitely with its reason.
LiDAR route availability cannot establish that a mechanically stalled robot
actually travelled through an unchanged open scene; independent measured motion
would be needed to diagnose that failure.

The adapter checks source age again immediately before publication and stops
if computation exceeded its 0.04 s budget. The bridge independently expires
commands after 0.25 s; the firmware's existing 500 ms watchdog remains unchanged.
No timeout was increased.

## Installation and exact Jetson commands

These single-line commands assume the existing checkout is
`~/sensor-fusion-ugv-old-migration`, as in the ride log; substitute its directory
if yours differs. Stop any existing stack before rebuilding. Review/fetch the
implementation branch into the checkout without overwriting local changes.
The launcher derives its root from its own path, so a separate checkout works.
ROS Humble must already be installed on Ubuntu 22.04.

For a first checkout of this branch in the existing repository, after reviewing
and safely saving any local changes (the command uses no force):

```bash
cd ~/sensor-fusion-ugv-old-migration && git fetch origin refs/heads/improve/lidar-swept-motion && git switch -c improve/lidar-swept-motion FETCH_HEAD
```

If the local branch already exists, switch to it instead of creating it again.

Install package/test dependencies:

```bash
sudo apt-get install python3-numpy python3-serial python3-pytest python3-colcon-common-extensions ros-humble-pointcloud-to-laserscan ros-humble-tf2-ros ros-humble-nav-msgs ros-humble-std-msgs
```

Build the changed packages, leaving the already-built driver alone:

```bash
cd ~/sensor-fusion-ugv-old-migration/ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select esp_bridge vfh_pkg --executor sequential
```

Offline functional, protocol, launcher and syntax checks (no ROS or hardware):

```bash
cd ~/sensor-fusion-ugv-old-migration && bash tests/run_offline.sh
```

Actual ROS middleware tests with synthetic scans and a fake serial transport:

```bash
cd ~/sensor-fusion-ugv-old-migration && source /opt/ros/humble/setup.bash && source ws/install/local_setup.bash && ROS_DOMAIN_ID=87 python3 -m unittest discover -s tests -p test_ros_integration.py -v
```

Working baseline, motors OFF / dry-run (wire CMD velocities are always zero):

```bash
cd ~/sensor-fusion-ugv-old-migration && bash run_cloud36_test.sh --dry-run --behavior baseline --speed-profile conservative
```

Enhanced fallback, motors OFF / dry-run:

```bash
cd ~/sensor-fusion-ugv-old-migration && bash run_cloud36_test.sh --dry-run --behavior enhanced --speed-profile conservative
```

Enhanced with your measured configuration, motors OFF / dry-run:

```bash
cd ~/sensor-fusion-ugv-old-migration && bash run_cloud36_test.sh --dry-run --behavior enhanced --speed-profile conservative --params "$PWD/ws/src/vfh_pkg/config/enhanced.yaml"
```

Opt-in higher requested speed, initially still motors OFF / dry-run:

```bash
cd ~/sensor-fusion-ugv-old-migration && bash run_cloud36_test.sh --dry-run --behavior enhanced --speed-profile open_space --params "$PWD/ws/src/vfh_pkg/config/enhanced.yaml"
```

Only after the corresponding stage below passes, explicit motor-enabled commands:

```bash
cd ~/sensor-fusion-ugv-old-migration && bash run_cloud36_test.sh --motors --behavior baseline --speed-profile conservative
cd ~/sensor-fusion-ugv-old-migration && bash run_cloud36_test.sh --motors --behavior enhanced --speed-profile conservative --params "$PWD/ws/src/vfh_pkg/config/enhanced.yaml"
cd ~/sensor-fusion-ugv-old-migration && bash run_cloud36_test.sh --motors --behavior enhanced --speed-profile open_space --params "$PWD/ws/src/vfh_pkg/config/enhanced.yaml"
```

Stop from another terminal, or use Ctrl+C in the launcher:

```bash
cd ~/sensor-fusion-ugv-old-migration && bash run_cloud36_test.sh --stop
```

The launcher sends SIGINT to nodes; bridge cleanup sends STOP and reports its
final ACK observation. If final ACK is missing or serial fails, use the physical
motor-power cutoff and investigate. `--stop` also stops the existing general
robot stack processes, as the prior script did. Keep other serial clients shut.
The port remains `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A84021142-if00`,
115200 baud. DTR/RTS=false **before open**, exclusive access, mandatory startup
ACK STOP, stale-command expiry, reboot/error latching, shutdown STOP, existing
wire/telemetry formats, dry-run and disabled reverse are retained.

## Logs and physical comparison

Logs go to `continuous-YYYYMMDD-HHMMSS-PID` under the checkout. Enhanced JSON
records show raw front range/count, side ranges, path and footprint clearance,
scan/source age, arrival and accepted ages, coverage [0,1], observation status,
selected and final `command` v/w, swept clearance, passing side, recovery age,
state and reason. Swept clearance saturates at 1 m for scoring; current footprint
clearance does not. JSON null means unknown/not evaluated or an unbounded path,
never verified free space. Records are limited to 2 Hz for unchanged states and
5 Hz for transitions. Bridge TX logs show input, planned and exact wire values.

1. **Motors physically off:** build, run offline/ROS checks, then baseline and
   enhanced dry-runs with live LiDAR. Verify source paths from the preflight,
   startup ACK STOP, capability flags, zero wire commands, finite front evidence,
   source age and coverage. Observe deliberate obstacles on all sides. Interrupt
   LiDAR and confirm controller zeros despite continuing timer publications;
   terminate controller and confirm independent bridge expiry. Never infer a
   clearance measurement from the test fixtures or the terminal log.
2. **Wheels securely raised:** keep a power cutoff available, explicitly select
   `--motors`, begin with the conservative profile, and observe wheel direction,
   startup/stop, gentle acceleration, stationary turn direction, scan-loss stop,
   controller-loss stop and shutdown ACK. Verify real stopping and firmware
   watchdog behavior here. A software assertion about a fake port does not verify
   physical braking or the installed firmware. Preserve the known-good firmware.
3. **Same floor course at conservative speed:** measure footprint/mount first
   where possible; compare baseline and enhanced fallback with identical obstacle
   positions and LiDAR settings. Then compare measured polygon/model curves if
   calibrated. Keep an operator at cutoff. Record course completion time, pauses,
   side changes, requested commands, measured speed, minimum observed physical
   gaps, freshness/coverage failures and stopping distance. Repeat each setting
   in both directions; accept no guard bypass to finish a course.
4. **Opt-in open_space:** only after earlier stages pass, compare the same course
   at `open_space`. Verify straight requests never exceed 0.08, turning/nearby
   requests never exceed conservative speed, angular requests stay <=0.25,
   actual speed and stopping remain within measured model bounds, and sensor
   gaps still cause immediate zero requests. Return to conservative if any
   physical or data-quality result is worse. No improved ride result is claimed
   by this change; [the log analysis](ride_log_findings.md) is historical only.

## Measurements still needed and rollback

| Measurement | Enables / validates |
| --- | --- |
| Full footprint and protrusions relative to chosen rotation reference | Accurate polygon corner and swept-footprint checks; otherwise legacy envelope remains unverified |
| LiDAR x/y/yaw and frame convention relative to that reference | Consistent point/footprint frame and real pivot; otherwise curves stay disabled |
| Actual straight/turn speed, gains, slip and error bounds at both requested profiles | Valid curved-motion prediction and reachability padding; do not equate Twist to physical motion |
| Stopping time/distance under load, latency and worst acquisition ages | Physical adequacy of margin/horizon and the optional speed profile; no timeout increase or margin reduction is implied |

Rollback is immediately selectable without editing/restoring files: stop the
stack, then run the baseline conservative command above. Its old hesitation and
geometry semantics remain visible for comparison. To remove the change entirely,
with a clean/safely saved working tree select the pre-change commit and rebuild
the same two packages:

```bash
cd ~/sensor-fusion-ugv-old-migration && git switch --detach ec5e89f1474392d5fa1074ceeafa30492372b955 && source /opt/ros/humble/setup.bash && cd ws && colcon build --symlink-install --packages-select esp_bridge vfh_pkg --executor sequential
```

That commit's original launcher expects the historical
`~/sensor-fusion-ugv-old-migration` path. Return to the implementation branch and
rebuild to re-enable selection. Never rebuild or change checkouts with motors on.

All enhanced defaults, units and parameter meanings are listed in
[enhanced_parameters.md](enhanced_parameters.md). Actual executed checks and
environment limits are recorded in [validation.md](validation.md).
