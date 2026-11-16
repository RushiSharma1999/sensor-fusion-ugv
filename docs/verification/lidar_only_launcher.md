# Original v4 firmware with the six-pane LiDAR launcher — 2026-11-01

**Verified from code** — At the user's request, the current launcher targets the unchanged original `bot_firmware_v4.ino` (SHA256 `211044ee2ac54f16b4314ac9a764ee2b44180d247819f4febd1e6d37d9387988`). The copy in `/home/sdp2026/Documents/ChatGPT/proj/bot_firmware_v4/bot_firmware_v4.ino` is byte-identical to `/home/sdp2026/Downloads/bot_firmware_v4.ino`. No firmware edit or flash was performed.

## Run it

1. Switch motor power off. Stop the previous managed ROS session:
   ```bash
   cd /home/sdp2026/sensor-fusion-ugv-migration
   ./stop_lidar_only.sh
   ```
2. If the ESP32 still has the newer LiDAR-only or calibration sketch, upload the original v4 first. Its identity is checked at startup; another firmware is refused. Open the identical copy in its correctly named Arduino sketch folder:
   ```bash
   /opt/arduino-1.8.19/arduino /home/sdp2026/Documents/ChatGPT/proj/bot_firmware_v4/bot_firmware_v4.ino
   ```
   In Arduino IDE select **Tools → Board → ESP32 Arduino → ESP32S3 Dev Module**, **USB Mode → Hardware CDC and JTAG**, **USB CDC On Boot → Disabled**, **Upload Mode → UART0 / Hardware CDC**, and the ESP32 serial port. Keep the existing confirmed flash/PSRAM settings. Click **Upload** (right-arrow button); wait for successful completion. Close Serial Monitor and Serial Plotter before continuing. The launcher uses the existing stable WCH by-id port. No encoder-count or geometry edits are needed; the user confirmed the original 65 mm wheel diameter, 150 mm axle track and 15 A-rising counts/revolution.
3. First verify the restored motor response with wheels raised and hardware stop available; the prior raised-wheel PWM25/20 trial used different firmware and recorded no encoder advancement. With motor power on, start from a desktop terminal:
   ```bash
   cd /home/sdp2026/sensor-fusion-ugv-migration
   ./start_lidar_only.sh
   ```
   This is the motion-enable command. No flags, manual ROS commands, Docker/YOLO, clean rebuild or calibration sketch are required. The changed bridge has already been built on this Jetson.
4. Stop the stack from another terminal:
   ```bash
   /home/sdp2026/sensor-fusion-ugv-migration/stop_lidar_only.sh
   ```
   Detaching tmux with Ctrl+B then D leaves it running. Reopen with `tmux attach -t lidar_only`.

## Current behavior and limits

Six panes: LiDAR, scan converter, VFH, safety mux, scan probe, motor bridge. Startup checks graph ownership, usable scan input and bounded commands, then sends only stopped identity/debug requests to v4. It verifies IDLE, AUTO disabled, zero PWM and no reported initialization error. A bounded retry tolerates early bytes lost during a USB-open MCU reset. It never sends ARM, MODE AUTO or raw PWM. The final mux remains the only `/cmd_vel` publisher; current maximum requests stay 0.08 m/s and 0.4 rad/s. Invalid/stale/missing frontal returns still stop motion. Network and projection settings are unchanged.

v4's default `US|ID:...|T:...|VALID:...|CM:...` records are converted to `/esp/ultrasonic_raw`. Invalid/missing/stale samples retain invalid flags; duplicate sample timestamps cannot indefinitely refresh them. `DBG|...` is available at `/esp/motor_status` and in pane 6; logging is re-enabled once after serial open. Inspect MODE, DOING, WHY, PWM and ENC when diagnosing no motion. **LiDAR selects navigation on the Jetson, while unchanged v4 retains its local sonar veto.** A nearby sonar obstacle can therefore prevent movement even when LiDAR commands forward. Camera/fusion nodes are excluded.

v4 has no ARM/latching STOP: a following valid nonzero host command resumes MANUAL. Use the stop launcher or actual hardware stop to halt driving. Never enable onboard MODE AUTO; that mode can drive independently of LiDAR. The original MANUAL watchdog remains 500 ms plus control scheduling; Jetson command/scan checks remain 250 ms.

Default v4 disables ODOM output; this integration does not fabricate odometry. Encoder counts remain visible in DBG. Motor speed is open-loop: original breakaway PWM60 is retained, with straight requested 0.08 mapping to PWM79/79. It may help the earlier weak-output symptom, but actual starting torque, loaded speed and stopping are unverified. The requested speed bound is not a measured physical speed limit. Firmware unchanged means these original behaviors remain.

## Verification

**Verified by automated software test:** 75 callback/serial/launcher tests; shell syntax and six-pane dry-run; actual ROS bridge build; original v4 ESP32-S3 target compile; native original-sketch command/watchdog/veto assertions. An isolated real ROS/DDS/pyserial integration used a virtual serial port and original v4 with GPIO/sensor stubs: clear scans produced virtual PWM79/79; all-inf scans and scan loss returned zero. It checked one final publisher and v4 telemetry handling. See [integration result](artifacts/v4_ros_pty_2026-11-01.json), [tests](artifacts/v4_compat_unit_tests_2026-11-01.log), [builds and failed-attempt notes](artifacts/v4_build_results_2026-11-01.txt), and [source hashes](artifacts/v4_compat_source_hashes_2026-11-01.json).

**Not yet hardware verified:** restored v4 flashing/running, physical movement, motor/encoder signs, hardware stop, communication-loss stopping, loaded speed or autonomous floor behavior. No hardware motor serial connection was opened in this change. Earlier raised-wheel trials apply only to the newer firmware and remain historical evidence.

---

# Historical launcher instructions (superseded)

The material below records earlier firmware and startup choices. Use the current v4 instructions above.

# Current no-flags startup — 2026-11-01

At the user's request, normal `./start_lidar_only.sh` now means operator enable: replace managed sessions, open the six panes, pass ROS checks, release startup software STOP once while outputs remain zero, then connect the LiDAR motor bridge. `./stop_lidar_only.sh` stops the stack without flags. A STOP received during operation is not automatically released. `--stationary` remains available for diagnostics.

This changes startup convenience only. No firmware duty, speed limit, scan validity policy or obstacle stop distance was changed. The last clear-scene wheels-raised trial reached PWM25/20 with no encoder advancement; motor response remains unresolved. Floor driving, real speed, stopping and emergency-stop behavior are not verified. Use the supplied LiDAR-only firmware, not the calibration-only sketch. The new default has been checked with dry runs and software tests; Codex did not start a floor run.

The following earlier flag-based instructions are historical and are superseded by the commands above.

# One-command LiDAR-only launcher

From a desktop terminal on the Jetson, after uploading `bot_firmware_lidar_only.ino`:

```bash
cd /home/sdp2026/sensor-fusion-ugv-migration
./start_lidar_only.sh --drive --replace
```

This replaces only the old `bot_stack`/`lidar_only` tmux sessions and opens six titled panes: LiDAR, scan converter, VFH, safety mux, scan probe, motor bridge. No camera, fusion controller, gap-follow or Foxglove process is started. Add `--arm` only when intentionally releasing a prior firmware STOP: `./start_lidar_only.sh --drive --arm --replace`. This helper confirms zero output and acknowledgment before the bridge starts; ordinary --drive never automatically releases a reported STOP. The bridge starts last after startup checks for one final command publisher, one scan/candidate publisher, usable fresh scans and bounded final requests. It never sends MODE AUTO, raw PWM, or made-up nonzero motion commands. ARM is sent only with the explicit --arm option.

**`--drive` opens the motor serial connection. Powered motors may move immediately. First motor testing must use securely raised wheels and the actual hardware stop.** The source-level software checks do not verify physical stopping, emergency-stop wiring, wheel signs or floor safety.

The default command without `--drive` opens the diagnostic stack and leaves the bridge disabled:

```bash
./start_lidar_only.sh --replace
```

To stop from another terminal:

```bash
/home/sdp2026/sensor-fusion-ugv-migration/start_lidar_only.sh --stop
```

To detach without stopping, press Ctrl+B, release both keys, then D. Reopen with `tmux attach -t lidar_only`. Detaching leaves the nodes and motor bridge running. Use --stop to stop the stack; physical motor disable remains the immediate physical stop.

Logs: `ws/log/lidar_only/<timestamp>/`. Each pane retains its exit output. The bridge logs received STATE/ODOM/ULTRA lines so firmware state and requested PWM are visible. Duplicate local robot nodes, a busy/missing serial port, invalid LiDAR-only configuration, and readiness failure prevent bridge startup. Closing a pane stops that node; existing scan/candidate/serial watchdogs remain in place. There is no automatic restart of failed nodes and no ongoing single-publisher graph enforcement beyond the existing node guards.

The profile is `config/lidar_only.yaml` (same provisional 0.08 m/s and 0.4 rad/s requested maxima, 0.50 m frontal stop, 0.25 s timeouts). The converter retains the previously deployed target frame/height/range settings; their physical geometry remains unverified.

This launcher cannot repair the observed intermittent front returns. Guarded stops remain intentional. It also cannot guarantee motor movement: the new firmware removed minimum PWM and uses an uncalibrated open-loop speed scale. Straight 0.01–0.08 m/s maps to approximately PWM 3–27, which can stall loaded motors. Calibrate motor response rather than bypassing scan guards or forcing a speed floor.

Advanced options: `--port /dev/serial/by-id/...`, `--detach`, `--dry-run --drive`, `--help`. Network setup only adds/replaces the existing `192.168.1.2/24` on `enP8p1s0` and brings that interface up when necessary; it never flushes other addresses.

Verified by automated software test: shell syntax, seven launcher CLI/quoting/mode checks, real stationary six-pane startup/replacement, live ROS readiness checks without serial actuation, and managed shutdown. Not yet hardware verified: motor-enabled launch, actual ESP32 firmware revision, wheel signs, real speeds, physical stopping or floor testing. The --drive bridge path was inspected/printed but was not started by Codex.

The [STOP recovery record](stop_latch_recovery.md) documents the live latch/status diagnosis, an initial serial-handshake timeout, and the later zero-output wheels-raised trial. That trial was stopped by a close frontal reading, so motor movement remains unverified.
