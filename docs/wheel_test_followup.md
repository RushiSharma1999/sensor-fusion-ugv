# Raised-wheel result and ROS timing regression

The user ran the bounded `wheels-curves` helper on the Jetson. Its output shows
startup ACK STOP and ACK DEBUG 1, both requested 0.6-second combined pulses,
STOP acknowledgement/IDLE/PWM=0 after each, and final STOP acknowledgement.
No firmware ERR, reboot or serial fault is reported. Requested v=0.020 and
serial w=-0.060/+0.060 match the selected angular sign -1. Reported PWM pairs
include 66/64 and 64/66, after an initial 8/8 ramp sample.

The user reports a very small forward-left difference and no reversed wheels.
That is consistent with a small differential, but it is not a measured body
yaw rate, curve radius, speed or stopping distance. PWM columns, firmware
STEER LEFT/RIGHT labels and uncalibrated ENC values do not establish physical
wheel mapping or floor traction. The -1 mapping deliberately uses firmware
labels opposite to the intended ROS body-turn labels. No gain or firmware/PID
change is inferred from this test. The helper has no LiDAR guard; do not run
its raised-wheel mode on the floor.

The raised-wheel serial/PWM/STOP test completed. Autonomous floor-curve
validation remains pending the measurements in the
[commissioning guide](perception_stability_and_curves.md#measurements-needed-for-autonomous-floor-curves).
The agent did not run motors or infer any missing physical dimensions.

## Separate failure in the uploaded validation output

The attached preflight output does **not** show all checks passing: 68 offline
checks and ten of eleven ROS tests passed, for 78/79. The failure is
`test_heading_aligned_turn_brakes_then_resumes_forward_without_type_failure`,
which expected full serial w=-0.250 within a fixed 1.2-second phase.

In that test's actual Humble log, a cold planning tick took about 59.99 ms and
correctly exceeded the unchanged 40 ms budget. The controller requested zero
and required fresh perception stability. A later report had six observations
but only 0.618 s since the first good observation, below the 0.75 s release
requirement. At the test cutoff the controller had resumed turning but only
reached w=+0.039. The fake serial bridge had not yet transmitted a full-speed
turn. The assertion's fixed phase duration was too short for the protection
and angular acceleration to finish. This is separate from the user's real
raised-wheel helper run; the ROS suite uses a fake serial transport.

The test now waits, with a six-second maximum per motion phase, for the actual
full signed wire turn and then forward resumption with zero angular output.
It still checks angular braking samples, absence of mixed commands in fallback
mode, the sign mapping and STOP cleanup. Failure remains bounded and reports
the controller state. The two-second test-handshake wait also requires an
actual fake-transport ACK; it does not change the bridge startup deadline.
These are test deadlines only. Runtime scan expiry, bridge expiry, compute
budget, confirmation, speed limits and coverage guards are unchanged.

A new ROS regression explicitly simulates a 60 ms compute interval with a fresh
scan and verifies immediate zero, the 0.75-second stability hold, and recovery
with fresh distinct observations. It neither sleeps the hardware nor modifies
the runtime guard to make the transition test pass.

Executed locally: all 12 ROS middleware tests pass in the available Jazzy
container. Changed-test Flake8/PEP257, Python 3.10 grammar and diff checks pass.
The 68 offline checks and runtime/package sources are unchanged from the prior
update; the functional suite now has 80 checks. The revised test still needs
its Humble rerun. No runtime build is required for this test/documentation-only
update; existing packages must already be built and sourced.

## Exact Jetson rerun (no hardware used by these tests)

Stop the live stack and update the same review branch without forcing away
local changes:

```bash
bash ~/sensor-fusion-ugv-old-migration/run_cloud36_test.sh --stop
cd ~/sensor-fusion-ugv-old-migration && git switch improve/lidar-swept-motion && git pull --ff-only origin improve/lidar-swept-motion
cd ~/sensor-fusion-ugv-old-migration && source /opt/ros/humble/setup.bash && source ws/install/local_setup.bash && ROS_DOMAIN_ID=87 python3 -m unittest discover -s tests -p test_ros_integration.py -v
```

Expected result is `Ran 12 tests` and `OK`. A cold compute-budget stop may still
appear in the test log; the assertion requires its recovery within the bounded
phase rather than suppressing the stop. Persistent inability to recover still
fails and needs investigation. This correction does not certify physical
perception timing, stopping adequacy or autonomous curved driving.
