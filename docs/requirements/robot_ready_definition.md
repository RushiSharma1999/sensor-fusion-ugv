# Robot-Ready Definition

## Purpose and status

This document defines the acceptance contract for the robot-ready integration effort. It is a **target requirement**, not evidence that the current repository meets it. Current evidence is tracked in [`../verification/test_matrix.md`](../verification/test_matrix.md) and [`../verification/evidence_log.md`](../verification/evidence_log.md).

The first robot-ready autonomous release is a low-speed reactive obstacle-avoidance system using the Unitree LiDAR, ToF, ultrasonic sensors, rich VFH navigation, deterministic sensor fusion, final safety arbitration, an ESP motor bridge, and checked-in/verified MCU firmware. Camera target following is optional and may follow after this core is reliable.

The first release does not depend on SLAM, Nav2, mapping, global planning, waypoints, occupancy-grid navigation, full localization, advanced probabilistic fusion, or an Extended Kalman Filter.

## Term definitions

### Pi-cloneable

A target Raspberry Pi can start from a fresh clone at a recorded commit, follow versioned instructions to install all declared host dependencies, and build and test the ROS 2 workspace without undocumented files or developer-machine state.

### Robot-ready

The Pi-cloneable software, compatible checked-in MCU firmware, confirmed hardware configuration, final command ownership, safety/fault behavior, and physical stop paths have passed their software, deployment, wheels-raised, and controlled low-speed acceptance gates. “Robot-ready” does not mean every deferred research capability exists.

### Autonomously runnable

One supported bring-up command starts the approved production graph in a stopped state. After preflight success and explicit autonomous enable, the UGV can complete the defined low-speed reactive obstacle-avoidance trials, stop on required faults or operator emergency action, and leave traceable evidence.

## Software acceptance criteria

The release must satisfy all of the following with evidence tied to a commit and configuration:

- Dependency declarations and installation instructions cover every production package.
- A clean ROS 2 workspace build succeeds on the supported target architecture.
- Unit, parser, algorithm, and integration tests for production paths pass.
- The Unitree driver publishes fresh, valid data and the required TF tree is connected.
- A verified projection publishes fresh, finite, robot-centric `/scan` ranges with documented geometry and orientation.
- The production graph has exactly one final `/cmd_vel` publisher. Alternative controllers publish candidates or run only in mutually exclusive test modes.
- Required command, scan, fusion, ESP, and safety inputs have explicit validity and freshness behavior. Missing, stale, invalid, or disconnected required inputs reach a defined safe state.
- Critical stops reach the final supervisor and produce a zero final command without relying on candidate-command smoothing.
- Communication loss and serial disconnect have specified, tested stopping behavior at both host and MCU boundaries.
- The ESP firmware source and protocol version used by the release are checked in, buildable, compatible with the host bridge, and traceable to the release commit.
- Startup, autonomous enable/disable, shutdown, and fault recovery behaviors are documented and tested.
- No required hardware fact is supplied by guesswork; values include measurement or confirmation provenance.

## Hardware acceptance criteria

The configured physical robot must pass and record:

- Unitree data validity and LiDAR-to-robot transform measurement.
- `/scan` orientation and obstacle-location checks in the robot frame.
- Exclusive serial-device ownership and the verified host-to-MCU protocol.
- Wheels-raised forward/reverse motor-sign tests.
- Wheels-raised positive/negative angular-sign tests.
- Measured command-to-motor response, including zero, saturation, and braking behavior.
- Physical emergency-stop operation independent of assumptions in host documentation.
- MCU command-timeout and host/serial communication-loss stopping.
- Confirmed ToF units, orientation, layout/aggregation contract, and drop-off response.
- Confirmed ultrasonic mounting geometry, validity/no-echo semantics, and critical-distance stop response.
- Conservative maximum linear/angular speed configuration and measured stopping distance on each approved surface.
- Controlled low-speed autonomous obstacle-avoidance trials, including representative obstacles, corners, sensor loss, and stop recovery.
- Repeated release-candidate trials at the approved configuration without unresolved safety-critical failures.

Passing software tests alone does not pass a hardware criterion. Each record must say whether it was performed with wheels raised or during controlled floor testing.

## Deployment acceptance criteria

On the target Pi and physical release configuration:

1. Start from a fresh repository clone at the release commit.
2. Install dependencies using only the versioned instructions and manifests.
3. Build the workspace successfully and run the automated test suite.
4. Run a preflight command that validates devices, configuration, topic health, TF, command ownership, firmware compatibility, and stop paths.
5. Use one documented production bring-up command.
6. Confirm the robot starts stopped and cannot move until an explicit autonomous-enable action.
7. Confirm valid Unitree data and a valid robot-centric `/scan`.
8. Confirm exactly one final `/cmd_vel` owner and one intended serial actuator owner.
9. Confirm the release's checked-in ESP firmware is installed and compatible.
10. Exercise emergency-stop and communication-loss stops before autonomous motion.
11. Execute the controlled low-speed acceptance course and repeatability trials.
12. Archive the release commit/tag, host and MCU configurations, test procedure, environment, logs/rosbags/CSV, and result in the evidence ledger.

## Final definition of done

Robot-ready acceptance requires a fresh target-Pi clone, documented dependency installation, successful workspace build, passing automated tests, one supported bring-up command, valid Unitree data, valid robot-centric `/scan`, exactly one final `/cmd_vel` owner, checked-in compatible ESP firmware, verified command-to-motor behavior, explicit freshness/fault behavior, verified emergency-stop and communication-loss stopping, measured motor signs, controlled autonomous obstacle-avoidance testing, and recorded release evidence.

Until every applicable gate is passed, describe the repository as under integration or partially verified—not deployment-ready, robot-ready, or safely autonomous.
