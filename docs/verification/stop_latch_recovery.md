# STOP-latch recovery — 2026-11-01

The screenshot and live serial logs showed `stop_latched 1`, `ok 1`, zero PWM, and rejected nonzero commands. The existing firmware intentionally requires ARM to release STOP. The launcher originally never sent ARM or checked STATE, so launching --drive did not recover a previously latched STOP.

The user confirmed securely raised drive wheels, powered motors and a hardware stop available before a brief powered test. Motor bridge was stopped first; other perception/controller panes remained running.

## Changes

`start_lidar_only.sh --drive --arm --replace` now explicitly requests one-time recovery. The serial helper first sends only zero and status/debug requests, synchronizes through a known ACK marker, validates stopped PWM and hardware status, optionally sends ARM, requires ACK ARM stopped and stopped/unlatched STATE, sends zero again and releases the port. Only then may the existing bounded LiDAR bridge start. --drive alone does not silently clear a reported STOP latch; --arm requires --drive. No automatic ARM is sent after a running STOP. No changes to LiDAR stop distances, missing-data guards, PWM calibration, firmware or speed caps.

## Produced evidence and limitations

Initial helper attempt using inactive DTR/RTS timed out. A subsequent zero/status read with the bridge's standard modem-line settings restored telemetry. Firmware uptime/encoder counters reset across that diagnostic sequence, indicating a board reboot; STOP does not persist across resets in this firmware. The helper was corrected to use the same DTR/RTS settings as the working ROS bridge. This failed attempt is not counted as a passing ARM test.

A corrected real helper run received ACK ARM stopped and confirmed host_active=0, stop_latched=0, ok=1, pwmL=0, pwmR=0. Its before state was already unlatched after the reboot; a real 1-to-0 transition was not measured in that run. Fake-serial tests cover that transition and failed-ARM relatching.

The six-second LiDAR-driven bridge trial after recovery still received only zero commands (host_zero), with no encoder change in the current trial. Live VFH/probe then reported a frontal obstacle approximately 0.17–0.22 m away, inside the configured 0.50 m stop threshold. This test establishes recovery/status and intentional stopping, not motor movement or sufficient PWM torque. Older scrollback from the previous bridge instance is excluded from JSON metrics.

Artifacts: `lidar_motor_recovery_trial_2026-11-01.json` and `.log`. The motor bridge was stopped after the trial and a zero-output state was confirmed. Other five sensing/controller panes remained running. A follow-up clear-scene wheels-raised trial depends on the user clearing the frontal scene or resolving the measurement.

This is not a successful motor-movement, floor autonomy, physical E-stop, speed calibration or braking result. Sparse/missing front measurements and uncalibrated low PWM remain separate limitations.

## Clear-scene follow-up

The user then confirmed the front was clear and requested another wheels-raised trial. An 8-second LiDAR-driven bridge run produced 61 STATE records, 13 with host_active=1, no latched STOP records, and peak requested motor duties 25 left / 20 right. Encoder spans were 0 left / 0 right. This confirms valid nonzero requests now reach firmware, but does not establish motor movement. Insufficient PWM torque is a potential issue; motor power/driver/encoder wiring are not ruled out by unchanged counts.

The bridge was stopped again and stopped PWM was confirmed afterward. No firmware/PWM increases or scan-guard changes were made. Minimum starting PWM/motor response data were requested before choosing a motor-control correction. Files: `artifacts/lidar_motor_clear_scene_trial_2026-11-01.json`, `.log`, and corresponding VFH/probe logs.

Software regression command: `python3 -m unittest discover -s tests/lidar_only -v` — 59 tests passed. Shell syntax and git diff whitespace checks passed.

## Calibration next step

The user reported no measured starting PWM and requested a wheels-raised calibration procedure. A separate `bot_motor_calibration` sketch and interactive host script were prepared under `/home/sdp2026/Documents/ChatGPT/proj/`; real S3 compile/link and host simulation tests passed. They were not flashed or run on hardware. The full ROS session was stopped before the planned upload. See [procedure](motor_calibration_procedure.md). Measurements are required before selecting a motor-control correction.
