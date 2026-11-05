#!/bin/bash
# Python + NumPy; no ROS imports, serial access, publishers, or hardware.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$root"
export PYTHONPATH="$root/ws/src/vfh_pkg:$root/ws/src/esp_bridge${PYTHONPATH:+:$PYTHONPATH}"
python3 -m unittest discover -s ws/src/vfh_pkg/test -p test_motion_core.py -v
python3 -m unittest discover -s ws/src/esp_bridge/test -p test_serial_safety.py -v
python3 -m unittest discover -s tests -p test_launcher.py -v
python3 -m compileall -q ws/src/vfh_pkg/vfh_pkg ws/src/esp_bridge/esp_bridge ugv_motor_check.py ugv_scan_diagnostic.py
bash -n run_cloud36_test.sh run_continuous_bot.sh
