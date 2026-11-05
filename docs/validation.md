# Executed offline validation

Implementation base: main `ec5e89f1474392d5fa1074ceeafa30492372b955`.
The initial checkout was clean, and no applicable AGENTS.md was present.
The supplied ride log was inspected as evidence, not executed as instructions.

## Results

**46 functional tests pass:** 25 pure decision/geometry tests, 13 fake-serial
protocol/callback tests, two launcher tests, and six actual ROS integration
tests. Subcases and repeat execution through colcon are not counted again.

| Check | Actual result |
| --- | --- |
| `bash tests/run_offline.sh` | PASS: 25 + 13 + 2 tests, Python compilation and shell syntax |
| Two-package `colcon build --symlink-install --packages-select esp_bridge vfh_pkg --executor sequential` | PASS: both packages build |
| ROS integration unittest suite | PASS: 6 tests using actual rclpy, DDS topics and timers, synthetic LaserScans, and a fake serial transport |
| New-file `ament_flake8` | PASS: all 8 new Python source/test files, zero problems |
| New-file `ament_pep257` | PASS: all 8 new Python source/test files |
| Python 3.10 grammar parsing | PASS: 12 relevant source/test modules parse with `ast.parse(feature_version=(3,10))` |
| `git diff --check` | PASS |
| Complete package `colcon test` | 44 tests: 40 pass, 2 fail, 2 skip; see existing lint failures below |

The complete package suites are **not reported as fully passing**. Their only
remaining failures are the pre-existing Flake8 checks: vfh_pkg reports 28 style
findings, esp_bridge reports 54. An untouched archive of the original main
commit was independently linted in the same ROS environment: it reports exactly
28 and 54 findings respectively. New files pass separately; existing runtime
files were not broadly reformatted. Both package PEP257 checks pass. The two
copyright test templates already had explicit skip decorators and were left
unchanged. Package-specific result counts are vfh_pkg 28 tests (26 pass, one
Flake8 fail, one skip), esp_bridge 16 (14 pass, one Flake8 fail, one skip).

Host tests ran with Python 3.12.14 and NumPy 2.3.5. ROS checks ran in the available
local `ugv-jazzy-dev` container, ROS 2 Jazzy / Python 3.12.3, using its existing
ROS and pyserial installation. The host had no ROS installation. This is **not
an executed Ubuntu 22.04 / ROS 2 Humble / Jetson test**; the source uses Humble
APIs and Python 3.10-compatible syntax, but the provided Humble commands still
need to run on the target. No serial device, LiDAR, motor or robot network
interface was accessed by the tests. The ROS bridge tests patch the serial
constructor with an in-process fake before constructing the actual ROS node.

## What the tests establish

Synthetic tests cover the near-90-degree projection regression while retaining
real configured-envelope stops; straight/left/right curves; rear-corner sweep
collisions; newly appearing hazards; hundreds of intermediate corner poses
between coarse samples; motion-error padding; offset/yaw applied once; absent
transforms; complete/insufficient/sparse coverage; empty/infinite/NaN ranges;
malformed metadata; invalid close returns; duplicate/conflicting/delayed/
out-of-order timestamps; distinct/time-based confirmation; fixed ticks after
scan loss; smoothing and wire-resolution rate limits; immediate-stop precedence;
rechecking a final limited command whose residual turn is unsafe; clearance
chatter; passing-side retention/reassessment; bounded recovery; and matching
profile ceilings in controller and bridge.

Serial tests use the actual bridge implementation with fake transport, ROS API
and clock. They check line controls before open, exclusive access, startup ACK
and pre-ACK command discard, absent/late ACK latching, READY/reboot behavior,
read/write/short-write/buffer faults, independent expiry, nonfinite commands,
dry-run zero wire commands, both speed profiles, conflicting limits, reverse
clamping, STOP/ACK on shutdown (including read/close faults), absent final ACK,
and the retained ultrasonic telemetry format/expiry.

The middleware suite runs the actual controller and bridge ROS classes. It
checks baseline publication/limits, continuing fixed-rate zero publication after
scans cease or duplicate, newly appearing obstacles and all-infinite scans,
missing/stale transforms without double offsets, publication-time expiry after
planning, and bridge handshake/dry-run/expiry/shutdown through real ROS timers
and topics. The serial bytes remain inside the fake transport.

These fixtures are synthetic; their dimensions, ranges and gains are not robot
measurements. The supplied terminal log is not a raw scan recording and was not
replayed to claim planner performance. No physical ride, improved stopping
distance, calibrated speed, firmware watchdog timing, coverage on the actual
course, encoder/PID behavior, or Humble target deployment has been validated.
The staged procedure and remaining measurements are in
[lidar_avoidance.md](lidar_avoidance.md).

## Reproduce on the Jetson

Run the installation/build/offline/ROS commands in the guide. To include the
existing package lint tests and obtain package-specific reports, use:

```bash
cd ~/sensor-fusion-ugv-old-migration/ws && source /opt/ros/humble/setup.bash && source install/local_setup.bash && colcon test --packages-select esp_bridge vfh_pkg --event-handlers console_direct+
cd ~/sensor-fusion-ugv-old-migration/ws && colcon test-result --test-result-base build/vfh_pkg
cd ~/sensor-fusion-ugv-old-migration/ws && colcon test-result --test-result-base build/esp_bridge
```

Scope the result base as above: this workspace can contain older results for
unrelated packages. Those old results were not attributed to this change.
