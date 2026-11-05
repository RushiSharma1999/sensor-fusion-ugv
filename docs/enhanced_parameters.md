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
| `footprint_xy` | `[]` | JSON string, m | Strictly convex CCW vertices [[x,y],…] around base origin; empty only in legacy mode. |
| `robot_radius` | `0.2` | m | Existing configured fallback radius; not a measured chassis dimension. Polygon mode uses vertices instead. |
| `safety_margin` | `0.16` | m | Outward footprint inflation, retained in both geometry modes; do not shrink to suppress stops. |
| `transform_mode` | `configured` | name | configured measured mount or scan-time tf; configured offsets are ignored in tf mode. |
| `mounting_calibrated` | `false` | bool | Operator assertion that configured polygon mount was measured, including zero offsets. |
| `lidar_x` | `0.0` | m | Scan origin x in base frame; polygon/configured only. |
| `lidar_y` | `0.0` | m | Scan origin y in base frame; polygon/configured only. |
| `lidar_yaw` | `0.0` | rad | Scan axes yaw in base frame; positive CCW; polygon/configured only. |
| `transform_timeout_sec` | `0.1` | s | Maximum nonstatic TF stamp difference from scan stamp. Missing TF stops. |
| `enable_curves` | `false` | bool | Requires polygon, valid mount and motion_model_calibrated; false preserves straight/spin fallback. |
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

| Profile | Controller requested linear ceiling | Bridge linear ceiling | Angular ceiling |
| --- | ---: | ---: | ---: |
| conservative | 0.05 m/s | 0.05 m/s | 0.25 rad/s |
| open_space (explicit opt-in) | 0.08 m/s | 0.08 m/s | 0.25 rad/s |

Baseline launcher thresholds and the original continuous controller timing
remain 0.65/0.85/1.2 m, five clear scans and a 20 s avoidance timeout.
Enhanced-only parameters do not change baseline geometry or timing.
The launcher prohibits open_space with baseline behavior.
