# ADR-001: Canonical Robot Autonomy Stack

## Status

Accepted for the robot-ready integration target; implementation and verification are pending.

This is a project architecture decision, not evidence that the current repository implements or verifies the complete stack.

## Context

The repository contains a rich modular path (`fusion_node`, VFH, camera candidate, `safety_mux_node`, and ESP bridge), a direct LiDAR-only `gap_follow` controller, and an integrated `esp_lidar_fusion_driver` experiment. Current source also lacks the PointCloud2-to-`/scan` connection, complete production bring-up, and MCU firmware.

Allowing multiple default `/cmd_vel` publishers or presenting all controllers as a single simultaneous stack would make command ownership and safety behavior ambiguous. The first release also needs a bounded goal that does not depend on unimplemented SLAM/Nav2 capabilities.

## Decision

### Selected production target

Adopt rich VFH navigation augmented by processed ToF and ultrasonic data through deterministic `fusion_node` sector penalties, critical stop, and speed limit. Route VFH and any other approved controller through final safety arbitration before the ESP motor bridge.

The first production milestone is low-speed reactive autonomous obstacle avoidance. It requires a planned Unitree PointCloud2-to-robot-centric-`/scan` projection, explicit health/freshness and stop behavior, one approved bring-up, checked-in compatible ESP firmware, and physical validation.

### Final command ownership

The production graph shall have exactly one final `/cmd_vel` publisher. Navigation and vision controllers publish named candidates. One final safety/supervisory layer owns arbitration, autonomous enable, critical stop, health/freshness enforcement, and final limits. The current `safety_mux_node` is the closest implementation and the starting point for later redesign; this ADR does not claim it already meets the target.

### `gap_follow`

Retain `gap_follow` as a simpler LiDAR-only baseline, experimental comparison controller, and early `/scan`/actuation integration test. It shall run in a mutually exclusive mode or be changed later to a candidate topic; it is not a simultaneous production final-command owner.

### `esp_lidar_fusion_driver`

Retain `esp_lidar_fusion_driver` / `vfh_with_esp_fusion` as an experimental alternative. Exclude it from normal production launch unless a later ADR explicitly redesigns its serial ownership, command interface, freshness behavior, and supervisory integration.

### Vision

Treat camera following as an optional supervised candidate-input subsystem. Integrate it after the core reactive avoidance stack is reliable. Vision does not bypass final safety arbitration.

### Deferred autonomy

Defer SLAM, Nav2, mapping, occupancy-grid/global/waypoint planning, full localization, Extended Kalman Filters, and advanced probabilistic fusion. Reconsider them only after perception, TF, odometry, safety, actuation, deployment, and low-speed testing are reliable.

## Rationale

- The rich path is the most internally coherent current architecture and already separates feature processing, fusion, navigation candidate, arbitration, and actuation boundary.
- A single final command owner makes source priority, freshness, critical stops, and failure behavior testable.
- Keeping alternatives preserves useful baselines without presenting mutually incompatible modes as production behavior.
- A reactive first release concentrates verification on the existing sensors and safety/actuation chain before adding global autonomy complexity.

## Alternatives considered

### Use `gap_follow` as production controller

Rejected as the canonical target because its current direct output bypasses ToF/ultrasonic fusion and final arbitration. Retained as a baseline because it is smaller and useful for early integration.

### Use `esp_lidar_fusion_driver` as production controller

Rejected for normal launch because it uses a separate ESP protocol, directly owns `/cmd_vel`, has unresolved default serial ownership with the actuator bridge, and does not provide a verified complete actuator path. Retained for experiments.

### Run all controllers simultaneously

Rejected. Multiple `/cmd_vel` publishers do not provide deterministic arbitration or an ownership contract.

### Adopt SLAM/Nav2 immediately

Deferred. It would add dependencies on TF, odometry, localization, maps, lifecycle, and planning before the existing reactive perception and safety chain is complete.

## Consequences

- Later production launches must make alternatives mutually exclusive and expose one `/cmd_vel` owner.
- The missing `/scan` projection, final safety behavior, ESP firmware contract, serial ownership, and hardware validation become explicit roadmap work.
- Candidate topics and supervisory interfaces may require later refactoring, but Phase 0 changes documentation only.
- Camera integration is not a gate for the first core release.
- Experimental results must identify which controller, commit, configuration, and safety envelope were used.

## Deferred decisions

- Exact PointCloud2 projection implementation and measured filter geometry.
- Final supervisor interface, enable-state mechanism, and fault-state model.
- Typed sensor/health messages and diagnostic schema.
- ESP protocol revision, firmware organization, and motor-driver contract.
- Hardware-specific values listed in [`../architecture/canonical_architecture.md`](../architecture/canonical_architecture.md).
- Whether any experimental controller should later be redesigned as a production candidate.

## Verification required

- Clean target-Pi build and dependency installation.
- Fresh Unitree cloud, connected measured TF, and valid robot-centric `/scan`.
- Runtime proof of exactly one final `/cmd_vel` publisher.
- Automated supervisor priority, invalid/stale input, critical-stop, and communication-loss tests.
- Checked-in MCU firmware build and host/firmware protocol compatibility.
- Wheels-raised motor direction, zero/braking, serial watchdog, and emergency-stop tests.
- Controlled low-speed baseline and rich-VFH course tests with logs tied to commits and configurations.

Required evidence is enumerated in [`../verification/test_matrix.md`](../verification/test_matrix.md); no item is passed by this ADR alone.
