# Room run: brief stops and normal turn braking

The user reports successful room traversal and obstacle avoidance without
collisions in run `continuous-20261113-160153-5796`. The uploaded terminal text
also shows both packages building and all 61 then-current functional checks
passing on the user's Humble system. The supplied `heading_marker.yaml` matches
the repository profile, including yaw -165 degrees and bridge angular sign -1.
These observations do not calibrate the physical footprint, speed or stopping.

## What the motor-run log establishes

Analysis begins after `Mode: --motors`, excluding the preceding synthetic ROS
tests. Duplicate lines from the shutdown tail are counted once. The excerpt
spans about 96.44 seconds from the first to last controller report: 251 throttled
controller records and 720 unique bridge TX traces. There are 30 logged
moving-to-zero wire transitions. These counts are reports, not a complete record
of control ticks, individual hazards, or human-observed jerks.

| Logged controller reason | Reports | Interpretation |
| --- | ---: | --- |
| `coverage_unknown` | 26 | Required finite-supported visibility was inadequate |
| `limited_command_coverage_unknown` | 1 | Final smoothed command lacked coverage |
| `scan_timeout_during_planning` | 8 | Source age crossed 0.35 s before publication |
| `observed_hazard_inside_footprint_envelope` | 8 | Close observed returns required an immediate stop |
| `control_compute_budget_exceeded` | 1 | Initial reported planning tick took 62.22 ms, exceeding 40 ms |
| `confirming_usable_perception` | 40 | Required fresh/time confirmation before release |

The remaining reports are 95 straight, 41 checked route-confirmation motions,
30 stationary turns and one startup wait. No controller crash, latched recovery
failure, firmware reboot or bridge command-timeout reason appears in the motor
excerpt. All 720 TX trace reasons are `command`: the controller requested the
brief zeros. The final STOP acknowledgement was observed.

Examples, relative to the first motor-run controller report:

- At 7.34 s, scan age reached 0.351 s and a 0.035 m/s request dropped to zero.
- At 11.06 s, coverage became unknown with central front range 1.179 m. The
  bridge changed from 0.05 to zero. Front range alone cannot verify side/rear
  footprint coverage; the old log does not identify the missing wedge.
- At 5.86 and 6.37 s, raw returns at bearings about +118/+117 degrees were
  0.331/0.270 m from the LiDAR, inside the configured envelope. Later hazard
  diagnostics include a 0.172 m return. These are observed hazards, not proven
  chassis returns or false positives; they must not be masked.

Reported source ages have median 0.234 s, p95 0.340 s and maximum 0.3524 s.
Accepted/arrival ages top out near 0.166 s. Most incoming source stamps already
lag arrival by roughly 0.175–0.20 s, consistent with acquisition/transport age
being separate from delivery cadence. The Unitree timestamp contract remains
unchanged. Frequent command publication does not refresh that source evidence.

Thus abrupt protective stops/restarts can explain jerks, but terminal text
cannot identify which event the user felt or establish its physical magnitude.
It is not a complete LaserScan recording or measured acceleration trace.

## Software changes

A normal straight-to-stationary-turn selection previously forced linear output
to zero as soon as angular output became nonzero, to prevent an uncalibrated
curve. It now decelerates translation with zero angular request, then turns.
`braking_before_stationary_turn` identifies this normal transition. The final
braking command is collision/coverage checked; unsafe braking, the front stop,
close hazards, invalid perception and stale data still override rate limiting.
The inverse turn-to-straight sequence still brakes angular motion first.

Visibility intersections now use bounded batches of NumPy operations with the
same vertices, rays, tolerances and angular/range support conditions. Only the
arithmetic is batched; current points, acquisition-age padding, full swept
footprint and final limited commands are still checked. Unknown space stays
unknown. No timeouts, coverage thresholds, margins, projection settings, speed
limits, sign mapping or curve-calibration requirements changed.

Logs now report finite-ray count, largest raw angular gap, source age on arrival,
and a failed required wedge with raw-frame start/width, required/supported
radii and whether its cause is an angular hole or insufficient range support.
New safety-stop reasons are logged immediately; repeated stops remain throttled.
This avoids losing a short initial stop to the normal 0.20 s transition throttle.

## Executed validation and limits

All 67 functional checks pass: 40 core, 16 fake-serial, two launcher and nine
actual ROS middleware checks with synthetic scans/fake transport. Five new core
checks cover batched/scalar intersections across angular wrap, exact edges and
batch boundaries; sparse/offset straight and curved motion; missing-wedge stop
and diagnostics; normal deceleration before turning; and immediate stopping
when a proposed braking sweep is unsafe. The new ROS check verifies immediate
coverage-stop logging despite a recent log, and throttling of its repetition.

Both packages build in the available Jazzy container. Six changed Python files
pass targeted Flake8/PEP257 and Python 3.10 grammar checks; compilation, shell
syntax and diff checks pass. Complete package suites report 62 tests: 58 pass,
two pre-existing Flake8 failures (vfh_pkg 28 findings, esp_bridge 54) and two
copyright skips. This update has not run on Jetson/Humble or robot hardware.

On the two previously supplied bags, scalar and batched visibility produce
identical 469 and 410 core decisions. Those older recordings usually take the
existing fast visibility path: local p95 times were about 2 ms in both versions,
with no demonstrated p95 improvement. They do not contain this room encounter.
A separate synthetic sparse-scan microbenchmark (100 warm checks/mode) measured
median visibility-check time 1.406 to 0.303 ms locally; this is not a Jetson
measurement, a complete controller timing result, or evidence of smoother rides.

The recorded-data comparison is reproducible, after sourcing/building packages:

```bash
cd ~/sensor-fusion-ugv-old-migration && source /opt/ros/humble/setup.bash && source ws/install/local_setup.bash && python3 tests/benchmark_scan_bag.py /absolute/path/to/recording_0.db3 --compare-visibility
```

The benchmark checks core decisions only; it does not enforce the adapter's
wall-clock computation deadline or reproduce physical motion.

## Update, verify, capture

Keep motor power off while updating/building. Retain the conservative profile
and the same heading file; do not enable curves or higher speed to address a jerk.

```bash
bash ~/sensor-fusion-ugv-old-migration/run_cloud36_test.sh --stop
cd ~/sensor-fusion-ugv-old-migration && git switch improve/lidar-swept-motion && git pull --ff-only origin improve/lidar-swept-motion
cd ~/sensor-fusion-ugv-old-migration/ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select esp_bridge vfh_pkg --executor sequential
cd ~/sensor-fusion-ugv-old-migration && bash tests/run_offline.sh
cd ~/sensor-fusion-ugv-old-migration && source /opt/ros/humble/setup.bash && source ws/install/local_setup.bash && ROS_DOMAIN_ID=87 python3 -m unittest discover -s tests -p test_ros_integration.py -v
cd ~/sensor-fusion-ugv-old-migration && bash run_cloud36_test.sh --dry-run --behavior enhanced --speed-profile conservative --params "$PWD/ws/src/vfh_pkg/config/heading_marker.yaml"
```

Check the new diagnostics, normal braking and unchanged immediate stops in dry-run.
Repeat the [wheels-raised direction and stop checks](heading_alignment.md), then
compare the same floor route at conservative speed with the same operator and
cutoff available. Use `--motors` in place of `--dry-run` only for those stages.
Safety-triggered jerks can remain: this change does not authorize coasting
through unknown space or extending scan freshness.

For a short recording of the next encounter, run this in a second Jetson terminal
while the tested launcher is running; it records only and publishes no motion:

```bash
cd ~/sensor-fusion-ugv-old-migration && source /opt/ros/humble/setup.bash && source ws/install/local_setup.bash && ros2 bag record -o "$HOME/ugv-jerk-$(date +%Y%m%d-%H%M%S)" /scan /diagnostic/cmd_vel_vfh /rosout
```

Stop recording with Ctrl+C, and retain the launcher's printed log directory too.
The new wedge diagnostic and raw scans are needed to distinguish missing rays
from insufficient finite range support. Do not infer that a side gap is free.

Stop the stack with Ctrl+C or its `--stop` command. To return to the immediately
preceding user-tested software, with the stack stopped and local edits saved,
select `git switch --detach 0acac9b9f229b9c47d005a8e8456ea034a7e2ad3` and rebuild
the same two packages. That retains the heading/sign/crash fixes. Switch back to
`improve/lidar-swept-motion` and rebuild to restore this update.
