# Evidence from the supplied ride log

Source: the uploaded `Pasted markdown(20261105-034817).md` (the request refers to
an `(1).md` name; only the supplied upload was available). This is terminal
output from two runs, not LaserScan samples, a rosbag, or a trajectory recording.
Commands printed in that document were treated as historical evidence, not
instructions to execute. The full 4,203 logical lines were inspected, including
all controller reports and aggregate bridge/firmware records.

There are 259 `[AVOID]` reports, 256 bridge TX records, and 529 firmware debug
records. All debug records containing the capability flags show
`TX_QUEUE:1|LIDAR_ONLY:1|SONAR_VETO:0`. Both shutdowns report final STOP ACK=1.
There is one logged bridge `cmd_timeout` TX. Logs are throttled and can be
interleaved; these counts are reports, not counts of scans or motor transitions.

Five maneuvers reached `clear_path_confirmed_brake_before_forward`: two in run
`194420-7001`, three in run `194521-7095`. One later run-1 maneuver was still
turning when that run ended. The final run-2 maneuver timed out.

| Run | First confirmation line | Resume line | Elapsed between logged reports |
| --- | ---: | ---: | ---: |
| 1 | 617 | 661 | 0.836343 s |
| 1 | 1295 | 1337 | 0.836059 s |
| 2 | 1817 | 1859 | 0.836196 s |
| 2 | 2279 | 2321 | 0.836687 s |
| 2 | 2941 | 2983 | 0.836480 s |

The source explains the pauses: it stops during five clearance observations,
publishes another brake on transition to cruise, and resumes on the next scan.
The reports are consistent with roughly 0.167 s scan intervals during these
sections. This is not a guarantee of driver cadence.

The last run-2 maneuver starts at timestamp 1793850362.107 (line 3115), then
alternates `confirming_clear_path` and
`observed_hazard_inside_configured_turn_clearance`; line 4133 reports
`avoidance_timeout_restart_required` at 1793850382.180. Several side minima
cross the configured 0.36 m envelope: for example lines 3559–3577 show left
values 0.349, 0.380, 0.348 m. These are configured-envelope hazards, not evidence
that the margin should be reduced. Source ordering also allowed the old maneuver
completion branch to precede the nearest-hazard branch; enhanced mode checks
hazards first.

Logged `front_min` reaches 0.000 at lines 3559, 3633, and others. The continuous
controller computes a minimum over both central radial range and `r*cos(angle)`
for returns with positive x and |y| <= 0.36. A return beside the robot can have
an arbitrarily small x even when its radial range is substantial. Thus this
log field is not consistently central-front range. Without raw scans it is
impossible to reconstruct the individual rays, distinguish obstacles from
chassis returns, or conclude that every near-zero value is an artefact.

The new synthetic regression places a return near 90 degrees just outside the
configured envelope, and independently verifies that returns inside it stop
both axes. Nothing is filtered as a presumed chassis return. The log has not
been replayed as if it were LiDAR data and establishes no improved physical
ride result.
