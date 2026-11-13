# Recorded scan series and computation follow-up

Input: uploaded `ugv-scan-series-20261113-135638_0.db3` and its metadata.yaml.
The database was opened read-only; its contents were treated as data.
Database SHA256: `a768fa11a8ca3a782016a3b4cd2724013eca8458a6d20abcc5dbf4c1d042a4b3`.
The bag itself is not added to this repository.

## Observations from the recording

| Quantity | Observed value |
| --- | ---: |
| Recorded duration | 18.727 s |
| /scan messages | 98 |
| /diagnostic/cmd_vel_vfh messages | 469 |
| Median /scan receipt interval | 167.079 ms (about 5.99 Hz) |
| Maximum /scan receipt interval | 168.661 ms |
| Scan header age at recording | 170.905–173.022 ms |
| Duplicate / non-increasing source stamps | 0 / 0 |
| Finite bins per scan | 77–88 out of 360 |
| Minimum finite radial range across all scans | 1.1214 m |
| Central-front minimum across all scans | 1.6059 m |
| Central-front finite bins | 4–11 |
| Largest finite-bracketing angular gap | approximately 8 degrees |
| Scans with a finite return within 0.36 m | 0 |
| Exactly zero ROS commands | 112 |
| Largest recorded linear request | 0.012 m/s |
| Recorded angular requests | all zero |
| Median / maximum command interval | 40.363 / 89.396 ms |

The first scan was recorded 2.52 s after the first command. This does not prove
LiDAR was absent from the controller during discovery/recording startup.
The steady cadence above applies only to the captured scan interval; it is
not a guaranteed sensor rate. Age of the latest recorded scan at subsequent
command receipt ranged from 171.498 to 339.159 ms. Those receipt times are not
exact controller callback times.

All 98 scans passed the current spatial collision/coverage checks for zero,
0.05 m/s straight, and +/-0.25 rad/s stationary-turn candidate commands. Checks
were repeated at assumed acquisition ages of zero and 0.35 s, retaining the
existing model/error/computation padding. Infinite ranges remained unknown;
the existing finite-bracketing criterion was used, unchanged. This recording
does not reproduce the earlier proximity or coverage failures.

Commands repeatedly ramp from small positive values back to zero. This is
consistent with the computation-budget failures in the earlier terminal logs.
The bag contains no controller stop reasons, computation timings, serial wire,
firmware state, calibrated odometry or measured robot motion, so it cannot prove
the reason for each individual zero or establish actual speed/stopping behavior.

## Software change

The enhanced core now keeps a bounded 128-entry cache of invariant swept
polygons keyed by requested command. Footprint/configuration are fixed at startup.
Current scan points, scan acceptance, visibility/coverage, age padding, candidate
selection and checks of the final limited command are still evaluated as before.
The age displacement-rate constant is also computed once per configuration.
No motion limits, geometry margins, timeouts or computation deadlines changed.
The first use of a geometry can still incur a cold-cache cost.

JSON motion logs now include `compute_elapsed_sec`, `compute_budget_sec` and
`swept_geometry_cache` statistics. On an observed footprint hazard, `last_hazard`
retains the actual triggering observation/stamp, LiDAR range/bearing, footprint
frame x/y and signed footprint clearance. Repeated stopped ticks and latched
failure reports retain those details. It is explicitly historical, not a new
measurement; regular current-scan metrics may still be null on guard-only ticks.

## Offline verification and limits

Replaying this bag's scan receipts and command-event times through cached and
uncached versions of the core produced identical command/selection/reason/state/
passing-side results at all 469 command events. In one local run, 95th-percentile
core computation was 11.17 ms with caching disabled and 2.48 ms with caching
enabled. Cache statistics were 1,258 hits / 19 misses, with 19 entries used.
Timing varies with host load; these are local development-machine measurements,
not Jetson measurements. Both core versions can reach a requested 0.05 m/s in
this counterfactual replay, which does not enforce the ROS adapter's wall-clock
computation deadline. That is not an improved physical ride result.

The diagnostic CDR scan reader was cross-checked against actual ROS
deserialization for all 98 scans: ranges, frame and source timestamps match.
Three additional synthetic tests verify cached geometry still detects new
obstacles/unknown coverage, re-applies age inflation, exactly matches uncached
spatial checks, and retains a hazard observation through repeated stopped ticks.
The offline suite passes 28 core + 13 serial + 2 launcher tests. Six actual ROS
topic/timer integration tests pass in the available Jazzy container: 49 functional
tests total. Both changed packages build there. Modified/new files pass targeted
Flake8 and PEP257 checks. Target Humble/Jetson timing still needs verification.
The complete package suites contain 47 tests: 43 pass, the same two existing
package Flake8 checks fail, and two existing copyright templates are skipped.

Reproduce the decision and timing comparison from the checkout (Python + NumPy,
no ROS or hardware needed):

```bash
PYTHONPATH=ws/src/vfh_pkg:ws/src/esp_bridge python3 tests/benchmark_scan_bag.py /absolute/path/ugv-scan-series-20261113-135638_0.db3
```

## Jetson retest

With motors off, stop the stack, select/update the existing review branch, and
rebuild. Git commands intentionally use no force and preserve local edits by
refusing conflicting updates:

```bash
bash ~/sensor-fusion-ugv-old-migration/run_cloud36_test.sh --stop
cd ~/sensor-fusion-ugv-old-migration && git switch improve/lidar-swept-motion && git pull --ff-only origin improve/lidar-swept-motion
cd ~/sensor-fusion-ugv-old-migration/ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select esp_bridge vfh_pkg --executor sequential
cd ~/sensor-fusion-ugv-old-migration && bash tests/run_offline.sh
bash ~/sensor-fusion-ugv-old-migration/run_cloud36_test.sh --dry-run --behavior enhanced --speed-profile conservative
```

Verify fresh front evidence, coverage, computation times and uninterrupted
normal acceleration while wire velocities stay zero in dry-run. Do not enable
motors or open_space merely because a local benchmark is faster. Proceed through
the existing [staged comparison](lidar_avoidance.md) after target checks pass.
