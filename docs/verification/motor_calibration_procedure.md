# Raised-wheel motor calibration

The actual LiDAR-driven test cleared STOP and reached PWM 25 left / 20 right but produced no encoder advancement. Low output torque is a potential cause; motor power, driver and encoder conditions still require verification. Do not force a LiDAR speed floor or weaken the scan guards to solve this.

This package measures each motor's first encoder movement under a small PWM sweep. It is **calibration-only firmware**, not navigation. The LiDAR launcher must stay stopped while it is installed. It accepts no ROS velocity commands or standalone AUTO mode.

1. Disable motor power. Keep the ESP32 USB connected and the robot securely supported with both drive wheels off the floor.
2. In a normal Jetson terminal, stop the stack:

   ```bash
   /home/sdp2026/sensor-fusion-ugv-migration/start_lidar_only.sh --stop
   ```

3. Close Serial Monitor. Open the calibration sketch:

   ```bash
   /opt/arduino-1.8.19/arduino /home/sdp2026/Documents/ChatGPT/proj/bot_motor_calibration/bot_motor_calibration.ino &
   ```

4. Arduino settings for the existing verified USB-to-UART adapter: Board **ESP32S3 Dev Module**, **USB CDC On Boot: Disabled**, **Upload Mode: UART0 / Hardware CDC**, **Upload Speed: 115200**, Port corresponding to `usb-1a86_USB_Single_Serial_5A84021142-if00` (currently `/dev/ttyACM0`). Keep the other working flash/PSRAM settings. Click Verify, then Upload. Wait for success. Keep motor power OFF throughout upload. Close Arduino IDE afterward.
5. Run the guided host script:

   ```bash
   python3 /home/sdp2026/Documents/ChatGPT/proj/bot_motor_calibration/calibrate_motors.py --max-pwm 60
   ```

6. Follow its prompts in order:
   - Type `POWER OFF` only after confirming motor power is off.
   - Turn the LEFT wheel one full revolution by hand in the direction corresponding to forward driving, stop it, then press Enter.
   - Do the same for the RIGHT wheel. It expects approximately 15 counts per revolution, as confirmed by the user. A failed hand check aborts before powered tests.
   - Keep both wheels raised, stop turning them and keep hands clear.
   - Only when prompted, switch motor power ON, have the physical stop available, and type `RAISED`.
   - The script tests LEFT, then RIGHT separately, stepping PWM by 2 up to 60. It stops a motor on the first encoder count or at that limit. Press Ctrl+C or use the physical stop at any time. Do not touch a powered wheel.
7. When the script finishes, switch motor power OFF. The result is saved at:

   ```text
   /home/sdp2026/Documents/ChatGPT/proj/bot_motor_calibration/motor_calibration_result.json
   ```

8. Send that JSON before applying a motor-control change. If a wheel never moves by PWM60, the script reports null rather than increasing beyond the selected ceiling. Do not interpret missing encoder counts as proof that more PWM is safe; retain the hand-check result and observe the wheel/driver.

The first ceiling 60 comes from the original v4 `PWM_BREAKAWAY` setting. It is a bounded first bench test, not a measured safe PWM or a new operational speed floor. The MCU also retains the original absolute upper bound 200, but no larger value is used unless an operator explicitly chooses a different `--max-pwm` value. Nothing here certifies a safe larger ceiling.

STOP, invalid input and command age >=250 ms zero both PWM outputs and disarm calibration. Positive duty only and one motor at a time are enforced on the MCU. It needs an explicit CAL BEGIN ceiling before any CAL PWM request. Motor/encoder pins and signs are copied from v4; physical polarity is inferred from the user-defined forward hand turn and powered count sign, then recorded for later review. No GPIO/wiring or safe speed is invented.

After measurements, the LiDAR-only firmware must be restored or updated with reviewed motor control. Do not run the robot on the floor with calibration firmware. A first-movement PWM on raised wheels is not a speed calibration, closed-loop controller tuning, or loaded-floor validation.

## Software verification

- Real ESP32-S3 build/link with installed Arduino-ESP32 3.3.12, CDC disabled: 331805 bytes program / 23616 bytes globals.
- Native C++ tests with address/undefined sanitizers (leak detection disabled for sandbox tracing): stopped boot, explicit ceiling, separate motors, zero/STOP, invalid/nonfinite/overflow/reverse/dual-channel rejection, watchdog threshold/wrap, status not refreshing watchdog, hardware fault rejection.
- Host-script tests cover parser behavior, one-motor sweeps, stopping at first simulated count, and never exceeding a selected ceiling. These use models; they do not measure real motors.
- This sketch has not been flashed or physically operated by Codex. No successful calibration or motor movement is claimed yet.
