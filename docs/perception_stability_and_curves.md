# Distant restart chatter and curved-drive commissioning

## Latest run and the software change

The uploaded terminal excerpt for `continuous-20261113-163330-5098` shows the
user's build and all then-current 67 functional checks passing on Humble.
Its motor segment spans about 129.57 s, with 320 throttled controller records
and 958 unique TX traces after removing repeated shutdown-tail lines.
There are 26 `coverage_unknown` reports, three scan-expiry reports, one initial
58.06 ms planning overrun, one swept-envelope collision report and 12 observed
inside-envelope hazard reports. These are report counts, not independent
physical incidents. The sole bridge `cmd_timeout` is a startup zero before any
fresh command (age=inf); later traces carry controller commands. The process
was interrupted by the user and final STOP acknowledgement was observed.

The new wedge diagnostics establish that the distant coverage stops arise
from 13–15-degree finite-ray gaps exceeding the existing 10-degree limit.
At about 36.01 s, central front range is 2.214 m, but a required raw wedge
starting near +61 degrees spans 15 degrees. Its required radius is about
0.396 m; bounding returns support about 6.16 m. Plenty of range at two rays
cannot rule out an unseen object between them. Front range alone cannot
certify the complete motion. Raw bearings are in the LiDAR frame, before
the heading correction; they are not physical body-left bearings.

Short good-data intervals previously allowed a restart after normal 0.20 s
confirmation, followed by another coverage stop. Enhanced mode now holds zero
until usable observations span 0.75 s and include at least five distinct fresh
scans after a perception/compute fault. A failure resets that release gate;
repeated control ticks do not advance it. It runs concurrently with normal
confirmation, preserves the passing side and never extends the absolute
20-second recovery budget. Persistent gaps can therefore produce a sustained
stop or latched failure, rather than repeated short drive pulses.

`perception_stability_confirming`, `perception_confirm_count` and
`perception_confirm_elapsed_sec` expose the wait. Initial startup and routine
obstacle release retain their existing confirmation if perception remained
usable. Actual hazards, invalid data, freshness failure and unsafe final
commands still request zero immediately. This change suppresses restart
chatter; it cannot promise physically jerk-free motion or permit driving
through unknown space. Coverage thresholds, source expiry, collision margins,
projection and cloud_scan_num=36 are unchanged. Curved candidates obey the same
coverage/freshness guards and would not eliminate these missing-data stops.

A synthetic intermittent-gap comparison (24 observations, a gap every fourth
scan) produces six moving observations in the previous controller and none
in the new controller until sustained usable data arrives. This is a synthetic
regression, not a replay of the supplied terminal text or a new physical result.

## Update and verify with motors off

Use the existing branch without forcing away local edits:

```bash
bash ~/sensor-fusion-ugv-old-migration/run_cloud36_test.sh --stop
cd ~/sensor-fusion-ugv-old-migration && git switch improve/lidar-swept-motion && git pull --ff-only origin improve/lidar-swept-motion
cd ~/sensor-fusion-ugv-old-migration/ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select esp_bridge vfh_pkg --executor sequential
cd ~/sensor-fusion-ugv-old-migration && bash tests/run_offline.sh
cd ~/sensor-fusion-ugv-old-migration && source /opt/ros/humble/setup.bash && source ws/install/local_setup.bash && ROS_DOMAIN_ID=87 python3 -m unittest discover -s tests -p test_ros_integration.py -v
cd ~/sensor-fusion-ugv-old-migration && bash run_cloud36_test.sh --dry-run --behavior enhanced --speed-profile conservative --params "$PWD/ws/src/vfh_pkg/config/heading_marker.yaml"
```

Dry-run wire velocities must remain zero. Check pending stability counts after
coverage loss, fresh/distinct release, immediate obstacle/scan-loss stops and
startup/final ACK STOP. Repeat the earlier wheels-raised and conservative
floor stages for normal avoidance only after those checks pass. Its command
uses `--motors` instead of `--dry-run` and the same heading file.

To investigate the actual ray gaps, record `/scan`, `/diagnostic/cmd_vel_vfh`
and `/rosout` alongside the printed vfh/bridge logs using the
[room-run capture command](room_run_findings.md#update-verify-capture). No complete
scan series for this encounter was supplied; terminal ranges do not establish
that the unsupported wedges are free. Keep guards active while diagnosing it.

## Next motor-on curve-request test: all driven wheels raised

This is available before physical measurements: it verifies that combined
forward/angular requests reach the firmware and both signed requests return
to STOP. It is **not a floor-curve or autonomous-avoidance test**. The helper
has no LiDAR planner and must only be used with the robot securely supported
and every driven wheel off the floor. Keep a motor-power cutoff accessible.

Stop the ROS stack and all serial monitors first; exclusive serial access is
retained. After physically securing the robot, run this explicit motor test:

```bash
bash ~/sensor-fusion-ugv-old-migration/run_cloud36_test.sh --stop
cd ~/sensor-fusion-ugv-old-migration && python3 ugv_motor_check.py --test wheels-curves --motors --wheels-raised --angular-command-sign -1
```

It uses the existing by-id port and 115200 baud, DTR/RTS=false before opening,
mandatory stopped-state ACK, and two fixed 0.6-second pulses: requested v=0.020,
w=+0.060 then -0.060 in ROS body convention, mapped to serial w=-0.060 then
+0.060 for this installation. Units are requested m/s and rad/s, not measured
physical speeds. No reverse body command, raw PWM, MODE AUTO or firmware update
is sent. Each pulse is preceded/followed by STOP/ACK/IDLE/PWM=0 checks, with a
final STOP report and close. Missing ACK, read/write/short-write faults, firmware
ERR or reboot end the test; a following pulse is refused after a fault.

Observe wheel directions and relative rates for both requests. The firmware
label, PWM and uncalibrated ENC counts cannot establish physical direction,
speed or curve radius. Minimum PWM/deadband can make small requests behave
unexpectedly; nonzero reported PWM alone is not proof of the intended wheel
motion. Raised wheels cannot establish traction, body speed, stopping distance
or an actual floor curve. Stop with Ctrl+C; if STOP cannot be verified, use the
physical cutoff. This helper was tested only with fake serial by the agent.

## Measurements needed for autonomous floor curves

The user has said these measurements are not yet available. Do not copy test
fixture dimensions, assume a centred mount, or set calibration flags merely to
bypass configuration checks. The existing 0.36 m envelope remains unverified.

1. **Mark a body reference and axes.** Mark a rigid point inside the chassis:
   +x physical forward, +y left, yaw positive counterclockwise viewed from above.
   Locate the intended drive/turn reference from the actual wheel layout and
   observe it during stationary turns; do not assume the LiDAR or geometric
   chassis centre is the turn pivot. Skid/slip and pivot movement enter the
   motion error budget. Use this reference consistently for every measurement.
2. **Measure the physical footprint in metres.** With motor power off, use a
   tape/square to measure forward, rear, left and right extents from that mark,
   including wheels, brackets, cables and other protrusions. For a rectangular
   enclosing outline, CCW vertices are `[-rear,-right]`, `[front,-right]`,
   `[front,left]`, `[-rear,left]`. Extents need not be symmetric. Record the raw
   physical vertices as `footprint_xy`; retain the separate 0.16 m safety margin
   rather than adding it twice or shrinking it. Enclose concave outlines with
   a convex polygon. Record measurement uncertainty.
3. **Measure the LiDAR mount from the same mark.** `lidar_x` is forward offset
   and `lidar_y` is left offset, in metres, to the manufacturer's scan origin.
   Use its frame/mount drawing if the scan origin is not a visible casing mark.
   `lidar_yaw` maps raw LiDAR axes into body axes. The front-marker result gives
   an initial -165-degree estimate; verify left/right marker positions and
   refine it. For a physical marker at body bearing beta and raw bearing alpha,
   yaw is the wrapped difference beta-alpha, in radians. With translation,
   compare the complete transformed point, not just that angle difference.
   Verify the scanner is planar/upright; a yaw alone cannot fix a reflected or
   tilted mounting. Supply a measured static x/y/yaw or a valid scan-time TF,
   never both. Polygon mode uses `align_legacy_heading=false` to avoid applying
   the old heading-only correction twice.
4. **Measure actual straight/turn response on the floor.** Use a clear,
   controlled area, cutoff operator and the already-tested conservative
   straight/stationary-turn behavior with its LiDAR guards. Put visible rigid
   marks on the body and record from above with a known floor distance scale;
   keep people outside the collision envelope. Match video times to stable
   command intervals in the retained logs. Measure signed displacement of the
   chosen body mark over an interval, and signed change in body heading with
   two rigid marks. Compute actual speed as distance/time and angular speed as
   radians/time. Exclude acceleration/deceleration when estimating nominal
   gains, but record those transients separately. Do not integrate commanded
   velocities or use ENC/ODOM as calibrated measured progress. Repeat both
   directions, relevant request levels, load and surface conditions; record
   low-speed deadband, slip and asymmetry. A stationary-turn gain alone does
   not establish combined-curve response.
5. **Measure stopping and uncertainty.** Record time/distance from the zero
   request until the body stops, including scan/transport delay and load effects.
   Derive nominal gains as measured speed divided by requested speed. Model
   errors are absolute physical m/s and rad/s deviations, including measurement
   uncertainty, repeatability, transients and cross-coupling. Zero error is not
   justified by one sample. Initial curved floor commissioning needs the
   measured footprint/mount and defensible motion bounds from these tests;
   then compare actual low-speed arcs against the model and revise bounds.

Provide the reference description, footprint vertices, mount x/y/yaw, the
command/time/displacement/heading observations and stopping measurements.
There is no fabricated ready-to-run calibrated configuration in this change.
Once the measurements support the model, use polygon mode with measured values,
`mounting_calibrated=true`, `motion_model_calibrated=true`, `enable_curves=true`,
`curve_linear_speed_cap=0.02` and `curve_max_yaw_rate=0.06` initially. These caps
bound actual final simultaneous v/w requests, including smoothing transitions.
Straight travel brakes before adding curvature if it exceeds the curve cap;
a stationary turn brakes its angular output before adding translation. Every
final command is still swept-footprint/coverage checked. Stationary turns keep
the overall 0.25 rad/s ceiling. Begin with dry-run, then raised wheels, then a
supervised conservative floor course; do not select open_space yet.

## Executed software verification and rollback

All 79 functional checks pass: 45 pure core, 16 bridge fake-serial, two launcher,
five motor-probe fake-serial/clock and 11 actual ROS middleware checks. New cases
cover repeated distant-gap suppression, distinct/time release, immediate hazard
precedence and bounded recovery, candidate/final curve caps during smoothing,
actual ROS stable release and calibrated synthetic curves, explicit raised-wheel
motor selection, signed bounded pulses, default stopped checks, missing ACK,
serial faults/reboot, old wheels/watchdog behavior and final STOP cleanup.

Both ROS packages built in the available Jazzy container. Targeted changed-file
Flake8/PEP257, Python 3.10 grammar, compilation, shell syntax and diff checks pass.
Complete package suites: 67 tests, 63 pass, two existing Flake8 failures (28 and
54 findings), two copyright skips. New source has not been run on Jetson/Humble
or hardware. Synthetic polygon dimensions and motion gains are test fixtures.
The agent did not run motors or establish that the physical jerks are eliminated.

To roll back just this update, stop the stack, preserve local edits, select
`git switch --detach a45f5f58d94dc4f7d72af6d6f274be7b07535184` and rebuild the same
two packages. That retains the previous heading/sign/normal-braking fixes.
Switch back to `improve/lidar-swept-motion` and rebuild to restore this update.

The later [raised-wheel and Humble test follow-up](wheel_test_followup.md) records
the actual bounded motor-test result and fixes a too-short ROS transition-test
phase without changing runtime safety timeouts.
