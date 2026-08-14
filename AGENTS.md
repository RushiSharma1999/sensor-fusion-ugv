# Repository Instructions for Codex

These instructions apply to the entire repository. More-specific `AGENTS.md` files may add constraints for their directories but must not weaken these rules.

## Project mission

Progress this repository toward a UGV that can be freshly cloned onto the target Raspberry Pi, have documented dependencies installed, build and pass preflight checks, start through one approved bring-up command in a stopped state, be explicitly enabled, and drive autonomously at verified low speed with fail-safe behavior.

## Source of truth and evidence

Use this hierarchy and record discrepancies instead of silently reconciling them:

1. Current source code is authoritative for what is implemented.
2. Builds, automated tests, runtime tests, logs, rosbags, and measured hardware results are evidence that behavior works.
3. `CODEBASE_DEEP_DIVE.md` is the principal static-analysis and issue-identification reference; recheck its findings against current source before changing code.
4. READMEs, reports, presentations, comments, names, and historical documents describe intent or history and do not override source.

Preserve these evidence labels exactly:

- **Verified from code** — directly supported by inspected source.
- **Likely intent** — strongly suggested but not proved.
- **Documentation says** — reported by documentation and not independently verified.
- **Potential issue** — an unverified discrepancy, risk, or failure mode.

Do not upgrade an evidence label without new evidence. Preserve explicit uncertainty such as `unknown`, `cannot be verified`, `missing`, `not present`, `firmware absent`, `external component required`, `proposal`, `alternative`, and `experimental`. Describe future designs as **Target architecture**, **Planned architecture**, or **Proposed change**, never as current behavior.

## Canonical production target

The target production stack is Unitree `PointCloud2` -> planned robot-centric `/scan` projection -> rich VFH candidate `/cmd_vel_vfh`, augmented by ToF/ultrasonic processing and `fusion_node` sector penalties, critical stop, and speed limit. A final safety supervisor arbitrates candidate commands and is the sole publisher of final `/cmd_vel`; `cmd_vel_to_serial` forwards that command to checked-in, compatible ESP32 motor firmware.

Architecture constraints:

- A production configuration must have exactly one final `/cmd_vel` owner. Controllers publish named candidate topics; one supervisory layer performs final arbitration.
- Existing `safety_mux_node` is the closest current implementation of that role. Do not assume it already satisfies all target safety requirements.
- `gap_follow` is a LiDAR-only baseline, comparison controller, and early integration test—not a simultaneous production command producer.
- `esp_lidar_fusion_driver` / `vfh_with_esp_fusion` is an experimental alternative and is excluded from normal production bring-up unless explicitly redesigned.
- The missing `/unilidar/cloud` -> robot-centric `/scan` integration is planned, not implemented.
- ESP32 motor/encoder/safety behavior is not represented completely in this repository and must not be invented.
- The current deterministic feature fusion is not a Kalman filter or probabilistic estimator.
- Camera following is an optional later candidate-input subsystem. SLAM, Nav2, mapping, localization, waypoint navigation, and global planning are deferred beyond the first robot-ready release.

See `docs/architecture/canonical_architecture.md`, `docs/decisions/ADR-001-canonical-stack.md`, and `docs/ROADMAP.md`.

## Hardware facts that must not be guessed

Never fabricate the LiDAR model or mounting transform; wheel radius, track width, gear ratio, encoder counts, motor polarity, motor-driver interface or pins; safe motor/linear/angular speeds; ESP32 GPIO; braking or physical emergency-stop wiring; ToF units, orientation, or matrix layout; ultrasonic mounting geometry; camera transform or angle convention; vision UART angle units; serial device names; network configuration; stopping distance; or safe autonomous thresholds.

When work needs an unknown value:

1. expose it as a clearly named configuration value where appropriate;
2. mark it as requiring hardware measurement or human confirmation;
3. do not provide a guessed value; and
4. record the source of the confirmed value.

## Change discipline

Before modifying files, future Codex work must:

1. verify the current branch and worktree state;
2. inspect the relevant current source;
3. verify an alleged defect against that source;
4. stay within the branch's documented purpose and acceptance criteria;
5. preserve user changes and avoid unrelated algorithm or cleanup changes;
6. add or update tests and documentation relevant to the change;
7. run all feasible tests relevant to the changed files; and
8. state exactly what was tested, what was not, and why.

Each phase starts from the latest accepted `integration/robot-ready` and has one independently testable purpose. Do not mix unrelated cleanup into a phase or treat compilation alone as sufficient for merge. Hardware-dependent changes require explicit hardware validation notes.

## Safety rules

- Missing or stale required safety/perception input must have deliberate, documented health, freshness, and safe-state behavior. Never treat missing safety-critical data as clear by accident.
- Critical-stop conditions must ultimately reach the final supervisory layer rather than depend only on a smoothed candidate controller.
- Do not weaken command watchdogs, stop paths, input validation, or single-owner command arbitration without explicit justification and tests.
- Motion-affecting changes must distinguish: **verified by automated software test**, **verified on robot with wheels raised**, **verified during controlled low-speed floor testing**, and **not yet hardware verified**.
- Before autonomous floor testing require one final command owner, verified serial ownership, wheels-raised motor testing, forward/reverse and left/right sign checks, emergency-stop verification, communication-loss watchdog verification, and conservative confirmed speed limits.
- Documentation that requires a test is not evidence that the test passed.

## Testing and verification

Run every feasible test relevant to changed files and report exact commands and results. Do not invent ROS, deployment, or hardware results when the environment cannot perform them. Update `docs/verification/test_matrix.md` for planned/status changes and `docs/verification/evidence_log.md` only when evidence is actually produced. Failed tests belong in the evidence record; do not omit them.

## Git rules

Unless the user explicitly instructs otherwise, do not switch branches, merge, rebase shared branches, force-push, rewrite history, delete unrelated files, or silently discard user changes. Do not create roadmap branches early. Inspect `git status` before and after changes.

## Documentation and publication rules

Update architecture references, ADRs, requirements, verification matrix, and evidence log when a change affects their facts or decisions. Keep current behavior separate from target behavior and alternatives. Do not silently rewrite historical documents to match the target architecture.

Engineering evidence should be reusable for an IEEE-style paper. No performance, safety, hardware-behavior, or successful-autonomy claim is a verified result until it is traceable to a commit, configuration, test procedure, environment, and artifact. Clearly separate implemented behavior, planned improvements, experimental alternatives, measured results, and limitations. Never claim deferred SLAM/Nav2 systems or a Kalman filter exist without verified implementation.
