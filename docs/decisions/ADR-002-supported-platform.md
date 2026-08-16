# ADR-002: Phase 1 Supported Platform

## Status

Accepted as the Phase 1 compatibility baseline; fresh-clone verification is pending.

This decision selects the environment to reproduce. It is not evidence that the corrected repository has already passed a clean target build.

## Context

Recovered machine inspection established a Raspberry Pi 4 running Ubuntu 22.04.5 LTS Jammy on Linux `aarch64`, kernel `5.15.0-1102-raspi`, ROS 2 Humble, and Python 3.10.12, with `ROS_DOMAIN_ID=10` and the legacy workspace at `~/unilidar_sdk2/ws`. Inspection also established the recovered colcon package inventory, shell environment configuration, and externally installed ROS packages and tooling described in [`../legacy/known_working_pi_baseline.md`](../legacy/known_working_pi_baseline.md).

**Documentation says** — the previous-team handoff identifies this machine as a successfully functioning autonomous robot deployment. We did not independently run the robot, verify autonomous behavior, or reproduce that operational claim during recovery or Phase 1.

The initial Phase 1 implementation instead selected Ubuntu 24.04 and ROS 2 Jazzy without equivalent target-hardware evidence. Reproducing the inherited robot and migrating it to a different Ubuntu/ROS release are separate engineering decisions.

**Verified from code** the checked-in Unitree SDK provides a Linux `aarch64` archive, the wrapper selects that archive using `CMAKE_SYSTEM_PROCESSOR`, and the source/manifests use APIs intended to be resolved by the selected ROS installation and rosdep.

## Decision

Phase 1 supports this compatibility baseline:

- Raspberry Pi 4 target hardware;
- Ubuntu 22.04 LTS Jammy;
- Linux `aarch64`;
- ROS 2 Humble; and
- Python 3.10.

Build and verification scripts shall use `/opt/ros/humble/setup.bash`, reject a pre-sourced different ROS distribution, and provide an actionable failure when Humble is absent. ROS dependencies shall be resolved through package manifests and rosdep. System/bootstrap tools remain explicit, reviewable installation steps.

The recovered `ROS_DOMAIN_ID=10` is recorded as a legacy runtime default. It is not required for compilation, is not exported by Phase 1 scripts, and is not written to a user's `.bashrc`.

Raspberry Pi 5, Ubuntu 24.04, and ROS 2 Jazzy are not supported by Phase 1 evidence. A Humble-to-Jazzy migration is deferred to a separate proposal with its own dependency, build, runtime, and hardware verification.

## Rationale

- The recovered Pi 4 is the strongest available evidence of the inherited deployment environment.
- Matching Jammy/Humble first isolates repository reproducibility work from a ROS distribution migration.
- The checked-in vendor archive already targets the recovered `aarch64` processor name.
- An explicit baseline makes rosdep results, build logs, and future hardware evidence comparable.

## Consequences

- Active Phase 1 instructions and scripts default to Humble on Jammy ARM64.
- The checked-in `x86_64` Unitree archive remains useful source evidence but does not establish target compatibility.
- Pi 5 and Jazzy claims remain unverified.
- Historical Jazzy-targeted static checks remain in the evidence log as superseded context; they are not relabeled as Humble tests.
- The fresh-clone procedure in [`../verification/test_matrix.md`](../verification/test_matrix.md) must pass before Phase 1 reproducibility is accepted.

## Alternatives considered

### Continue with Ubuntu 24.04 and ROS 2 Jazzy

Deferred. There is no target-Pi build or runtime evidence for that migration, and it would combine two independent engineering changes.

### Claim Raspberry Pi 5 compatibility from ARM64 library inspection

Rejected. A matching processor name and archive format do not prove board, operating-system, driver, network, or runtime compatibility.

## Verification required

On a freshly imaged Raspberry Pi 4, check out the exact Phase 1 commit, install only documented prerequisites, resolve rosdep dependencies, perform a clean build, run tests, verify all source packages and console executables, confirm the Unitree ARM64 link input, repeat discovery in a new shell, reboot, and repeat the critical verification.

Until that procedure passes with a durable artifact, recovered machine inspection proves that the historical Humble environment existed, but it does not independently prove successful autonomous operation. Fresh-clone reproducibility remains **not yet hardware verified**.
