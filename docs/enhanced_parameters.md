# Enhanced controller parameters

Defaults are declared by `vfh_pkg.motion_core.Config` and the ROS adapter.
All are startup parameters; restart the controller after changing them.
`footprint_xy` is a ROS string containing JSON, not a nested ROS array.
The calibration assertions must describe measurements, not merely enable flags.

| Parameter | Default | Units | Meaning |
| --- | --- | --- | --- |
| `speed_profile` | `conservative` | name | Authoritative conservative/open_space requested ceilings from esp_bridge.speed_profiles. |
| `control_hz` | `25.0` | Hz | Steady control timer; supported range 20–30. |
| `control_compute_budget_sec` | `0.04` | s | Maximum planning time per publication; overrun requests zero. Included in reachability padding. |
| `scan_timeout_sec` | `0.35` | s | Maximum source and monotonic accepted age; does not depend on publication cadence. |
| `future_tolerance_sec` | `0.02` | s | Small tolerated producer/consumer clock skew; larger future stamps stop. |
| `scan_frame` | `unilidar_lidar` | frame | Required incoming LaserScan frame ID. |
| `base_frame` | `base_link` | frame | Measured polygon and rotation-reference frame; unused as a physical assumption in legacy mode. |
| `geometry_mode` | `legacy_envelope` | name | legacy_envelope (scan-reference fallback) or measured convex polygon. |
| `align_legacy_heading` | `false` | bool | Explicitly rotate legacy scan points by lidar_yaw into heading-aligned axes at the LiDAR origin. No x/y translation, TF or polygon combination is allowed. Does not assert footprint/pivot calibration or enable curves. |
| `footprint_xy` | `[]` | JSON string, m | Strictly convex CCW vertices [[x,y],…] around base origin; empty only in legacy mode. |
| `robot_radius` | `0.2` | m | Existing configured fallback radius; not a measured chassis dimension. Polygon mode uses vertices instead. |
| `safety_margin` | `0.16` | m | Outward footprint inflation, retained in both geometry modes; do not shrink to suppress stops. |
| `transform_mode` | `configured` | name | configured measured mount or scan-time tf; configured offsets are ignored in tf mode. |
| `mounting_calibrated` | `false` | bool | Operator assertion that configured polygon mount was measured, including zero offsets. |
| `lidar_x` | `0.0` | m | Scan origin x in base frame; polygon/configured only. |
| `lidar_y` | `0.0` | m | Scan origin y in base frame; polygon/configured only. |
| `lidar_yaw` | `0.0` | rad | Scan-to-geometry yaw, positive CCW. Used in polygon/configured mode, or explicitly with align_legacy_heading in the fallback. The latter leaves the origin at the LiDAR; it is not a measured base_link transform. |
| `transform_timeout_sec` | `0.1` | s | Maximum nonstatic TF stamp difference from scan stamp. Missing TF stops. |
| `enable_curves` | `false` | bool | Requires polygon, valid mount and motion_model_calibrated; false preserves straight/spin fallback. |
| `preview_curves` | `false` | bool | Check signed curve candidates for the log while keeping them out of selection. Mutually exclusive with enable_curves. The launcher exposes this only in enhanced dry-run. See the [Jetson commissioning procedure](curve_commissioning.md). |
| `curve_linear_speed_cap` | `0.05` | requested m/s | Upper bound for simultaneous v/w output; >0 to 0.05. Straight-to-curve transitions brake before introducing angular output above this cap. Use 0.02 for initial measured-geometry commissioning. |
| `curve_max_yaw_rate` | `0.20` | requested rad/s | Largest curved candidate yaw; 0.06 to 0.25. Initial commissioning can select 0.06. Stationary turns retain the profile's 0.25 limit; their angular output brakes before adding translation. |
| `motion_model_calibrated` | `false` | bool | Operator assertion that nominal gains and absolute errors bound measured motion over the horizon. |
| `linear_model_scale` | `1.0` | physical m/s per requested m/s | Nominal linear gain. Default is unverified until calibrated. |
| `angular_model_scale` | `1.0` | physical rad/s per requested rad/s | Nominal angular gain. Default is unverified until calibrated. |
| `linear_model_error` | `0.0` | physical m/s | Absolute linear model error bound, used for horizon and age uncertainty padding. |
| `angular_model_error` | `0.0` | physical rad/s | Absolute angular error bound; includes centre-trajectory and footprint displacement. |
| `horizon_sec` | `4.0` | s | Constant-command model horizon. Physical stopping adequacy still requires measurement. |
| `sweep_step_sec` | `0.25` | s | Maximum trajectory interval; Lipschitz padding covers all intervening poses. |
| `coverage_max_gap_deg` | `10.0` | degrees | Largest finite-bracket interpolation gap; supported range >0 to 15. Infinity is not a ray. |
| `min_front_returns` | `3` | distinct finite bins | Minimum central-front evidence per scan, separately from whole-sweep coverage. |
| `front_stop_dist` | `0.65` | m | Central/front-equivalent threshold: immediately disable translation; checked stationary turn may remain usable. |
| `hard_turn_dist` | `0.85` | m | Anticipatory curve preference reaches full weight here; not a collision-envelope override. |
| `front_slowdown_dist` | `1.2` | m | Full conservative straight speed / route-clear front threshold. |
| `high_speed_clearance` | `1.8` | m | Both raw-front and path-equivalent evidence must exceed this for open_space speed. |
| `high_speed_footprint_clearance` | `0.3` | m | Minimum signed footprint clearance for above-conservative requests. |
| `release_margin` | `0.03` | m | Extra footprint clearance needed after a stop; a new hazard never waits for hysteresis. |
| `front_release_margin` | `0.1` | m | Extra front clearance above stop threshold before translation release. |
| `confirm_sec` | `0.2` | s | Minimum elapsed time between usable distinct observations for release. |
| `confirm_scans` | `2` | observations | Minimum distinct fresh observations as well as confirm_sec. |
| `perception_release_sec` | `0.75` | s | Continuous usable-observation duration required after missing/invalid/stale perception or compute expiry. Runs concurrently with normal confirmation and never extends the recovery deadline. |
| `perception_release_scans` | `5` | distinct fresh observations | Minimum count in addition to perception_release_sec. Failed evidence resets the gate; repeated control ticks do not count. Ordinary obstacle release uses confirm_sec/scans. |
| `linear_accel` | `0.08` | requested m/s² | Normal increase rate; safety stops/caps bypass normal limiting. |
| `linear_decel` | `0.15` | requested m/s² | Normal decrease rate; not a physical braking guarantee. |
| `angular_accel` | `0.5` | requested rad/s² | Normal angular magnitude increase rate. |
| `angular_decel` | `0.75` | requested rad/s² | Normal angular decrease/reversal rate; crosses zero before reverse. |
| `recovery_budget_sec` | `20.0` | s | Absolute episode deadline, never extended by noise or small improvements; exhaustion latches until restart. |
| `recovery_clear_sec` | `1.0` | s | Sustained usable straight route needed to finish an episode; not measured travel. |
| `recovery_clear_scans` | `4` | observations | Distinct observations required in addition to recovery_clear_sec. |
| `side_blocked_sec` | `1.0` | s | Committed candidate set must remain blocked this long before an opposite side can be reconsidered. |
| `progress_timeout_sec` | `3.0` | s | Time without material observed clearance gain before reconsidering a materially better verified opposite candidate. |
| `progress_delta` | `0.15` | m | Minimum high-water increase in front/path evidence; does not reset overall recovery budget. |
| `side_switch_cooldown_sec` | `2.0` | s | Minimum time between permitted side switches. |
| `max_side_switches` | `2` | count | Maximum switches per bounded recovery episode. |
| `forward_weight` | `3.0` | score | Forward requested-speed score weight. |
| `clearance_weight` | `1.5` | score/m | Swept-clearance score weight; clearance contribution saturates at 1 m. |
| `continuity_weight` | `0.25` | score | Penalty for angular change normalized to angular ceiling. |
| `commitment_weight` | `0.4` | score | Bonus for a motion on the selected passing side. |

The ROS wrapper additionally declares `behavior` (`baseline` default, or `enhanced`),
`scan_topic` (`/scan`) and `cmd_topic` (`/cmd_vel_vfh`). The shell remaps the
output to `/diagnostic/cmd_vel_vfh`. Only scan input is used in enhanced mode.

The bridge retains `dry_run=true`, `cmd_timeout_sec=0.25` s, `rate_hz=30` Hz,
`rx_hz=50` Hz, `allow_reverse=false`, mandatory startup STOP ACK within 5 s,
STOP retry interval 0.2 s and final STOP ACK window 0.5 s. Launcher overrides
port to the existing by-id path and keeps baud=115200. `speed_profile` defaults
to `conservative`; `max_linear_speed`/`max_angular_speed` default to that
profile and explicit overrides must match it. `trace_commands=true` and
`firmware_debug=true` preserve wire/firmware trace output. Telemetry retains
`ultra_timeout_sec=0.6` s; those readings do not influence enhanced avoidance.

`angular_command_sign` is a bridge-only integer, default `+1`, allowed values
`-1` or `+1`. It maps the limited ROS angular request to serial exactly once:
`w_wire = angular_command_sign * planned_w`; the planner retains positive-left,
counterclockwise axes. Dry-run/expiry/nonfinite stops still send exact zeros,
and sign selection changes neither speed ceilings nor reverse protection.
Invalid signs fail before opening serial. Startup and TX traces report the
sign, ROS input/limited requests and exact wire values. Firmware debug labels
and existing ODOM telemetry are passed through; they are not calibrated motion
feedback or used by the enhanced controller.

The optional `--params` file is passed to both nodes. Put controller settings
under `vfh_obstacle_avoidance.ros__parameters` and bridge settings under
`cmd_vel_to_serial.ros__parameters`. Launcher motor/dry-run, speed profile and
reverse selections are applied after the file. `heading_marker.yaml` explicitly
selects `angular_command_sign=-1` for this user's confirmed clockwise response
to positive serial w. Other installations retain `+1` unless measured otherwise.
This serial sign is independent of `lidar_yaw`; changing yaw cannot fix motor
polarity.

| Profile | Controller requested linear ceiling | Bridge linear ceiling | Angular ceiling |
| --- | ---: | ---: | ---: |
| conservative | 0.05 m/s | 0.05 m/s | 0.25 rad/s |
| open_space (explicit opt-in) | 0.08 m/s | 0.08 m/s | 0.25 rad/s |

Baseline launcher thresholds and the original continuous controller timing
remain 0.65/0.85/1.2 m, five clear scans and a 20 s avoidance timeout.
Enhanced-only parameters do not change baseline geometry or timing.
The launcher prohibits open_space with baseline behavior.
