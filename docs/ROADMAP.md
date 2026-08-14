# Robot-Ready Integration Roadmap

## Roadmap contract

The integration base is `integration/robot-ready`. Each phase branch starts from the latest accepted version of that branch, has one independently testable purpose, updates relevant tests and documentation, and returns only after its acceptance criteria are met. Do not create later branches early, mix unrelated cleanup into a phase, merge merely because code compiles, or claim hardware validation without recorded hardware evidence.

Every future Codex session must verify the current branch and worktree before changing files, inspect current source, and recheck the relevant [`CODEBASE_DEEP_DIVE.md`](../CODEBASE_DEEP_DIVE.md) finding. Hardware-dependent work must identify what is **not yet hardware verified** and record the robot/test configuration when testing occurs.

Status key: **In progress** means the phase is being prepared/reviewed; **Planned** means no completion is claimed.

```text
integration/robot-ready
chore/00-project-guardrails
fix/01-reproducible-pi-build
fix/02-unitree-driver-and-tf
feat/03-pointcloud-to-scan
refactor/04-command-ownership
fix/05-failsafe-supervisor
feat/06-esp-firmware-contract
fix/07-emergency-stop-and-serial
test/08-lidar-autonomy-baseline
fix/09-sensor-fusion-correctness
feat/10-rich-vfh-autonomy
feat/11-vision-integration
feat/12-pi-deployment
validation/13-release-candidate
```

## Phase 0 — Project guardrails

- **Branch name:** `chore/00-project-guardrails`
- **Status:** In progress until this branch is reviewed and merged.
- **Goal:** Establish source-of-truth, architecture, safety, hardware-unknown, branch, verification, and publication-evidence rules without changing implementation.
- **Primary issues addressed:** Ambiguous canonical stack; mixed evidence levels; missing durable acceptance/test records; undocumented phase boundaries.
- **Codex/software work:** Create root instructions, canonical architecture, ADR, robot-ready definition, roadmap, verification matrix, evidence ledger, and PR checklist. Do not modify `ws/src` or implementation behavior.
- **Human/hardware work:** Review the target architecture and roadmap. No hardware test is authorized or implied.
- **Acceptance gate:** Documentation is internally consistent, current versus target behavior is explicit, future results remain pending, and protected implementation/historical analysis files are unchanged.
- **Expected evidence:** Markdown review, clean diff checks, file-scope verification, and branch/status output. No runtime or hardware result.

## Phase 1 — Reproducible Pi build

- **Branch name:** `fix/01-reproducible-pi-build`
- **Status:** Planned.
- **Goal:** Make dependency installation and a clean ROS 2 build reproducible on the supported Raspberry Pi environment.
- **Primary issues addressed:** Unitree sibling SDK paths; incomplete package manifests; undeclared serial/ROS dependencies; ambiguous target prerequisites.
- **Codex/software work:** Correct build metadata and dependency declarations, document the supported OS/ROS/architecture, and add clean-build checks without changing autonomy algorithms.
- **Human/hardware work:** Provide or confirm the target Pi image, architecture, network access policy, and any vendor-library compatibility constraints.
- **Acceptance gate:** A fresh target-Pi clone can install declared dependencies, build all intended packages, and run applicable automated tests using documented commands.
- **Expected evidence:** Pi environment inventory, dependency/install logs, colcon build/test summaries, and release-relevant artifacts tied to the commit.

## Phase 2 — Unitree driver and TF

- **Branch name:** `fix/02-unitree-driver-and-tf`
- **Status:** Planned.
- **Goal:** Make Unitree startup, data validity, timestamping, and a measured robot-connected TF chain reliable.
- **Primary issues addressed:** Unchecked initialization; quaternion-order inconsistency; disconnected TF trees; unverified model, transport, and sensor mounting.
- **Codex/software work:** Add driver error handling and tests, correct source-verified transform issues, and expose measured extrinsics/configuration without guessing hardware values.
- **Human/hardware work:** Confirm LiDAR model/firmware/transport and measure the LiDAR/IMU pose relative to `base_link`; collect stationary and motion data.
- **Acceptance gate:** Driver start/failure behavior is deterministic, `/unilidar/cloud` is fresh and valid, and TF connects the sensor to the robot using traceable measured values.
- **Expected evidence:** Driver logs, topic-rate/field captures, TF inspection, measurement record, rosbag or equivalent data, and hardware configuration.

## Phase 3 — PointCloud2 to scan

- **Branch name:** `feat/03-pointcloud-to-scan`
- **Status:** Planned.
- **Goal:** Provide the verified robot-centric `/scan` required by navigation.
- **Primary issues addressed:** Missing `/scan` producer; absent projection/filter configuration; unverified scan orientation and invalid-data behavior.
- **Codex/software work:** Integrate and configure a projection, add launch/configuration and automated geometry/invalid-input tests, and document QoS and frames.
- **Human/hardware work:** Confirm projection height/filter bounds and validate obstacle bearings/ranges against measured robot geometry.
- **Acceptance gate:** `/scan` is fresh, finite/valid under the defined contract, oriented correctly in the robot frame, and stops/flags safely on invalid or stale source data.
- **Expected evidence:** Synthetic projection tests, topic/TF inspection, measured obstacle placements, configuration, and recorded scan artifacts.

## Phase 4 — Command ownership

- **Branch name:** `refactor/04-command-ownership`
- **Status:** Planned.
- **Goal:** Ensure production and test modes have deterministic command ownership.
- **Primary issues addressed:** Multiple direct `/cmd_vel` publishers; alternatives bypassing arbitration; no mutually exclusive full bring-up.
- **Codex/software work:** Route approved controllers to named candidate topics, define mutually exclusive modes, and add graph/launch tests proving one final command owner.
- **Human/hardware work:** Review intended teleoperation/test/autonomy modes and confirm which mode is permitted for each validation stage.
- **Acceptance gate:** The production graph has exactly one `/cmd_vel` publisher and one actuator-serial owner; baseline/experimental modes cannot collide with it.
- **Expected evidence:** Launch tests, ROS graph captures, mode matrix, publisher counts, and documented command-topic contract.

## Phase 5 — Fail-safe supervisor

- **Branch name:** `fix/05-failsafe-supervisor`
- **Status:** Planned.
- **Goal:** Make final arbitration fail safely under missing, stale, invalid, stopped, or disabled inputs.
- **Primary issues addressed:** Permissive cached safety state; missing fusion/scan freshness; smoothed or internally conflicting stop paths; no explicit autonomous enable/fault model.
- **Codex/software work:** Extend the existing supervisory role with explicit health/freshness, critical-stop priority, stopped startup, enable/disable, final limits, diagnostics, and automated failure-injection tests.
- **Human/hardware work:** Approve the required-input policy, operator enable/disable procedure, fault-reset rules, and provisional conservative test limits without treating them as final safe values.
- **Acceptance gate:** Every required fault has a documented deterministic output; critical stop and disable produce immediate final zero; no stale/invalid required input is silently clear.
- **Expected evidence:** Supervisor unit/launch tests, fault-injection logs, state-transition records, command traces, and reviewed safety requirements.

## Phase 6 — ESP firmware contract

- **Branch name:** `feat/06-esp-firmware-contract`
- **Status:** Planned.
- **Goal:** Check in compatible MCU firmware and establish a versioned host/firmware contract.
- **Primary issues addressed:** Firmware absent; unknown command units/mixing; unverified telemetry validity, watchdog, GPIO, encoder, and motor-driver behavior.
- **Codex/software work:** Add firmware source/build instructions and a versioned protocol specification with parsing/compatibility tests; keep hardware constants configurable and unguessed.
- **Human/hardware work:** Supply the actual controller/driver/wiring facts, measure wheel/encoder/kinematic values, and verify firmware flashing and bench I/O.
- **Acceptance gate:** Firmware builds reproducibly, identifies a compatible protocol, rejects invalid/stale commands safely, and matches host tests; required hardware configuration has provenance.
- **Expected evidence:** Firmware build log/binary hash, protocol fixtures, host/MCU compatibility results, wiring/config record, and bench logs.

## Phase 7 — Emergency stop and serial safety

- **Branch name:** `fix/07-emergency-stop-and-serial`
- **Status:** Planned.
- **Goal:** Validate the complete emergency-stop, serial-disconnect, command-timeout, braking, and serial-ownership path.
- **Primary issues addressed:** Canonical path lacks verified physical E-stop; runtime serial recovery/health is weak; no proof that MCU applies commanded zero or times out safely.
- **Codex/software work:** Integrate explicit MCU/serial health and emergency state with the supervisor, add bounded reconnect/fault behavior, and automate host-side loss scenarios.
- **Human/hardware work:** Inspect physical E-stop wiring; test on bench and with wheels raised; verify command-loss timeout, disconnect behavior, braking, and recovery procedure.
- **Acceptance gate:** Physical emergency action and each specified communication failure stop motion within the approved measured bound and remain safe through reconnect/reset.
- **Expected evidence:** Schematics or verified wiring record, timestamped command/motor traces, disconnect/watchdog tests, stopping measurements, and test configuration.

## Phase 8 — LiDAR autonomy baseline

- **Branch name:** `test/08-lidar-autonomy-baseline`
- **Status:** Planned.
- **Goal:** Use `gap_follow` as a mutually exclusive LiDAR-only baseline and early end-to-end autonomy test.
- **Primary issues addressed:** No verified scan-to-motor reference trial; baseline invalid/stale scan behavior and obstacle response not demonstrated.
- **Codex/software work:** Add reproducible scan fixtures/simulation tests, a baseline-only launch mode, logging, and documented course procedure without making it the production controller.
- **Human/hardware work:** Perform wheels-raised sign checks, E-stop precheck, then conservative low-speed controlled course trials with measured obstacles.
- **Acceptance gate:** The baseline mode has exclusive command ownership, handles invalid/stale scans safely, and completes its defined course within recorded constraints.
- **Expected evidence:** Automated scan tests, ROS graph, course layout, rosbag/CSV/video references, collisions/interventions, and repeated trial results.

## Phase 9 — Sensor-fusion correctness

- **Branch name:** `fix/09-sensor-fusion-correctness`
- **Status:** Planned.
- **Goal:** Make ToF/ultrasonic processing and deterministic fusion correct, fresh, calibrated, and testable.
- **Primary issues addressed:** ToF timer replays counted as samples; unspecified ToF units/geometry; stale/fail-open inputs; ignored ultrasonic validity/rear data; unverified thresholds.
- **Codex/software work:** Count unique observations, define validity/freshness, test parsing/calibration/hazard/critical-stop logic, and document deterministic fusion accurately—not as a Kalman filter.
- **Human/hardware work:** Confirm ToF model/units/orientation/aggregation, ultrasonic geometry and validity semantics, collect calibration datasets, and measure drop-off/obstacle response.
- **Acceptance gate:** Unique-sample calibration and loss behavior pass automated tests; critical sensor cases produce defined supervisor outcomes; configured units/geometry have provenance.
- **Expected evidence:** Captured raw/feature datasets, calibration results, parser/fusion test reports, fault-injection traces, and controlled sensor trials.

## Phase 10 — Rich VFH autonomy

- **Branch name:** `feat/10-rich-vfh-autonomy`
- **Status:** Planned.
- **Goal:** Qualify rich VFH plus sensor fusion as the production reactive controller.
- **Primary issues addressed:** Resolution-dependent histogram behavior; stale fusion; open-loop escape conflicts; unverified tuning and fused-course performance.
- **Codex/software work:** Resolve verified controller/supervisor interactions, add deterministic fixtures and comparison metrics, configure conservative limits, and create the production reactive launch path.
- **Human/hardware work:** Tune only from recorded trials, verify obstacle/drop-off behavior and recovery on controlled courses, and measure stop/clearance margins.
- **Acceptance gate:** The single-owner rich stack passes defined clear-path, obstacle, corner, sensor-fault, critical-stop, and repeatability trials at approved low speed.
- **Expected evidence:** Unit/integration reports, parameter/config revision, baseline comparison, course logs/rosbags/CSV, intervention/collision counts, and repeated trial data.

## Phase 11 — Vision integration

- **Branch name:** `feat/11-vision-integration`
- **Status:** Planned and optional for the first core release.
- **Goal:** Add camera target following as a safe, supervised candidate without weakening reactive avoidance.
- **Primary issues addressed:** Unverified UART angle unit/sign and class policy; blocking/stale behavior; camera stop smoothing; missing source/protocol evidence.
- **Codex/software work:** Specify/version the UART contract, validate fields/classes/freshness, route safety-critical conditions to the supervisor, and add candidate/fallback tests.
- **Human/hardware work:** Confirm camera transform, target class, angle convention, sender behavior, occlusion/loss cases, and controlled following limits.
- **Acceptance gate:** Vision cannot bypass safety; loss/invalid/stale input falls back or stops as specified; sign, class, range proxy, and target-follow trials are hardware verified.
- **Expected evidence:** Protocol fixtures, parser/controller/supervisor tests, measured sign/calibration record, UART captures, and controlled target-follow trials.

## Phase 12 — Pi deployment

- **Branch name:** `feat/12-pi-deployment`
- **Status:** Planned.
- **Goal:** Provide repeatable robot provisioning, preflight, and one approved production bring-up on the Pi.
- **Primary issues addressed:** Manual multi-node startup; unstable device/network assumptions; no stopped-start/enable deployment workflow; incomplete diagnostics.
- **Codex/software work:** Version provisioning and configuration, stable device mapping, preflight checks, production launch, logging, service/startup policy, and recovery instructions.
- **Human/hardware work:** Confirm deployed serial identities/network, perform a fresh-Pi rehearsal, validate operator workflow, and review field recovery/rollback.
- **Acceptance gate:** A fresh target Pi reaches a passing preflight and stopped production graph using only versioned instructions, then enables and disables safely through the documented procedure.
- **Expected evidence:** Fresh-clone transcript, system inventory, dependency/build/test/preflight logs, startup graph, firmware/config hashes, and operator checklist.

## Phase 13 — Release candidate validation

- **Branch name:** `validation/13-release-candidate`
- **Status:** Planned.
- **Goal:** Freeze and validate a traceable first robot-ready release candidate.
- **Primary issues addressed:** Cross-phase regression risk; missing end-to-end repeatability and release record; unresolved safety limitations.
- **Codex/software work:** Freeze approved configurations, run the full software/deployment test matrix, produce release notes, and assemble evidence without adding unrelated features.
- **Human/hardware work:** Repeat preflight, E-stop, communication-loss, wheels-raised, and controlled low-speed floor trials across the approved scenarios and surfaces.
- **Acceptance gate:** All required [`requirements/robot_ready_definition.md`](requirements/robot_ready_definition.md) gates are passed; no open safety-critical blocker remains; residual limitations and deferred work are explicit.
- **Expected evidence:** Release commit/tag, complete matrix, signed/reviewed test procedure, environment/configuration snapshot, logs/rosbags/CSV/video references, repeated-trial summary, and limitations.

## Recommended future IEEE-style manuscript outline

Engineering work should accumulate traceable material for a later paper, but Phase 0 does not create the manuscript.

1. Introduction
2. System Requirements and Problem Definition
3. Hardware and Software Architecture
4. LiDAR Processing and 3D-to-2D Projection
5. ToF and Ultrasonic Processing
6. Sensor-Fusion Method
7. Reactive Navigation and Command Arbitration
8. Safety and Fault Handling
9. Experimental Methodology
10. Results
11. Limitations and Future Work
12. Conclusion

The manuscript must distinguish implemented behavior, planned improvements, experimental alternatives, measured results, and limitations. No claim about performance, safety, hardware behavior, or successful autonomy is a verified result until it is traceable to a specific commit, configuration, test procedure, environment, and evidence artifact. Failed trials remain part of the record. Do not describe deterministic fusion as a Kalman filter or claim SLAM, Nav2, mapping, or other deferred systems are implemented without source and verification evidence.
