# Commissioning curved drive on the Jetson

## Live LiDAR preview with motor power off

Keep the ESP and LiDAR powered for the STOP handshake. Stop any previous robot
stack, update the feature branch, and build on Humble. Run each line on the Jetson.

```bash
bash ~/sensor-fusion-ugv-old-migration/run_cloud36_test.sh --stop
cd ~/sensor-fusion-ugv-old-migration && git switch improve/lidar-swept-motion && git pull --ff-only origin improve/lidar-swept-motion
cd ~/sensor-fusion-ugv-old-migration/ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select esp_bridge vfh_pkg --executor sequential
cd ~/sensor-fusion-ugv-old-migration && bash run_cloud36_test.sh --dry-run --behavior enhanced --speed-profile conservative --preview-curves --params "$PWD/ws/src/vfh_pkg/config/heading_marker.yaml"
```

Begin with usable finite returns around the stationary robot. The preview checks
the two signed arc requests against the live scan with the existing swept-footprint
and coverage code. It uses the configured 0.36 m LiDAR-origin envelope,
heading estimate and uncalibrated model. Its results describe that configuration.
It never adds a previewed arc to the set the controller can select. The launcher
accepts preview only in enhanced dry-run. Both deployed YAML files keep their
calibration and curve-enable flags false.

Read these records from the **same** printed `Logs:` directory.

| File | Evidence |
| --- | --- |
| `vfh.log` | Startup includes `curves=False preview_curves=True`. In a motion record look for `"preview_curves": true` and a nonempty `curve_checks` on the numbered observation. Each entry has a positive requested `v`, signed `w` (+0.06 left or −0.06 right) and the result of its sweep, such as `clear` or `swept_footprint_collision`. Two entries show both directions were evaluated. |
| `vfh.log` | `selected` and `command` describe the normal fallback selection and final limited ROS output. An empty `curve_checks` gives no evidence of evaluation. Read `reason`, `scan_age`, `coverage_gap` and front evidence. A scan fault, unconfirmed perception or translation latch can prevent evaluation. |
| `bridge.log` | Require `dry_run=True`, `RX ACK STOP; accepting fresh ROS commands`, successful TX with `wire="CMD v 0.000 w 0.000"`, `reason=dry_run/...` and `sent=1`. Check every logged TX for zero wire velocity. `in_v` and `planned_v` can be positive. Firmware debug should show `PWM:0/0`. Finish with `Final STOP acknowledgement observed=1` after Ctrl+C. |

TX logs are throttled and also report changes in wire value, reason or write
success. The middleware regression checks every fake-serial velocity write.
The preview caps are **requests** of 0.020 m/s and 0.060 rad/s. The selector
reduces translation for curvature, so the evaluated `v` can be below 0.020.
The original 10° finite-ray coverage gate, 0.35 s scan expiry, 40 ms compute
budget and independent 0.25 s bridge command expiry apply. Keep the recorded
13–15° gaps visible as stops.

## Raised wheels

End the dry-run with Ctrl+C. Secure **every driven wheel** above the floor,
mark the treads and keep the motor-power cutoff within reach. Run the bounded
probe after stopping the stack and closing other serial tools.

```bash
bash ~/sensor-fusion-ugv-old-migration/run_cloud36_test.sh --stop
cd ~/sensor-fusion-ugv-old-migration && python3 ugv_motor_check.py --test wheels-curves --motors --wheels-raised --angular-command-sign -1
```

Save both 0.6 s pulse records, the STOP ACK and IDLE/zero PWM after each, and
the final STOP ACK. Record which **physical** side runs faster, ideally with
video. Both sides should roll forward throughout each pulse. With this wiring,
the requested left arc (wire −0.060) should run the right side faster, and the
right arc (wire +0.060) should run the left side faster. Resolve an ambiguous
differential or wrong direction here. This serial probe has no LiDAR guard and
is for the stand only.

## Shortest supervised floor sequence

1. **Prepare a straight/spin measurement lane.** On the assembled, loaded
   robot mark forward, left and a repeatable body reference near the driven
   turn center. Measure its outer extents including wheels, cable sweep and
   protrusions. Measure the available lane and lateral runout. Check the mount
   is rigid and level, and compare surveyed front/left/right targets with live
   returns using the heading file. Verify fresh coverage on this course and the
   physical cutoff and expiry response on the stand. If the fallback envelope
   misses the body or target directions are wrong, resolve the geometry before
   floor operation. Allow independent clear runout for a missed stop and use a
   cutoff operator. The handoff has no measured stopping distance.
2. **Measure loaded straight travel, stationary turns and stopping with curves
   disabled.** Use the existing conservative LiDAR-guarded launcher shown
   below. Mark a scale on the floor and film two body marks from above. Match
   video time to the ROS and wire commands. Repeat the relevant low requests in
   each direction used, record deadband, speed, yaw, drift, transients and turn
   center movement. Measure physical time and distance to rest from a protective
   stop, command loss and cutoff under controlled conditions. Record the start
   of the hazard/command loss as well as the first wire zero, so latency is
   included. Keep the stop trigger clear of the moving robot. End a run on
   unexpected motion, insufficient runout, unreliable coverage or a fault.

```bash
cd ~/sensor-fusion-ugv-old-migration && bash run_cloud36_test.sh --motors --behavior enhanced --speed-profile conservative --params "$PWD/ws/src/vfh_pkg/config/heading_marker.yaml"
```

3. **Before a guarded floor arc**, review the measured convex CCW raw footprint
   in metres about the driven reference and the installed LiDAR x/y/yaw in that
   frame. Check mounting height and tilt against the projection. Verify surveyed
   targets after exactly one transform. Keep the 0.16 m margin separate. Derive
   nominal linear/angular gains and conservative absolute errors from measured
   response, including timing uncertainty, slip, startup and bounds on combined
   motion. Check that the whole predicted sweep and measured stopping runout
   fit the clear course. The 0.050 m/s request produced approximately 0.183 m/s
   in **one** observed run. It supplies neither a gain bound nor a prediction at
   the lower curve cap. If mixed-motion bounds cannot yet be justified, stay at
   the straight/spin stage. The production selector has no uncalibrated floor
   arc override.
4. **After that review**, use a separate measured trial parameter file with
   polygon geometry, one verified configured mount or scan-time TF, the
   calibrated model and `align_legacy_heading=false`. Verify the file first
   in ordinary enhanced dry-run with curve selection enabled for that trial.
   Then run one supervised approach per side with the same LiDAR guards, a
   cutoff operator and ample clear runout. Limit simultaneous requests to
   0.020 m/s and 0.060 rad/s. Straight/spin requests can still reach 0.050 m/s
   and 0.250 rad/s. Retain the separate trial file and the deployed fallback.
   Never inject a Twist into the bridge or use the raised-wheel helper on the
   floor to obtain arc measurements.

### Evidence to enable autonomous curves later

- A dated as-built outline and driven reference, measured mount pose, target
  residuals within documented measurement uncertainty, and one verified
  scan-to-body transform. The CAD shell omits the running gear.
- Synchronized video or independent pose and scale, scan bag, VFH and bridge
  logs, exact parameters, surface, load and charge for repeated left and right
  arcs, straight approaches, transitions and stops. Include the actual limited
  ROS command and wire sign. Record `/scan`, `/diagnostic/cmd_vel_vfh` and
  `/rosout`; the handoff bench bag lacks scan and measured odometry.
- Show that signed body displacement/yaw, wheel differential, drift and
  corner excursions for **both** curves, including transitions, fit the
  model's gain and absolute error bounds across the intended operating range.
  Bound physical stopping time/distance including perception/transport delay
  and measurement error inside the available clearance. Sampled IDLE/PWM
  timings alone cannot supply these measurements.
- On the target Jetson, retain passing offline and Humble integration results,
  dry-run zero-wire and raised-wheel STOP evidence, then supervised logs
  showing the intended arc and a checked straight resumption. Check zero on
  an envelope hazard, coverage loss, stale scan or command timeout and final
  STOP ACK. Check scan age and compute budget on the course; resolve repeated
  stops before increasing the operating scope.

Stop the live stack from a second terminal if needed.

```bash
bash ~/sensor-fusion-ugv-old-migration/run_cloud36_test.sh --stop
```
