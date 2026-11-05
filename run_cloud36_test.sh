#!/bin/bash
# One LiDAR-only implementation; both entry points retain cloud_scan_num=36.
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec bash "$root/run_continuous_bot.sh" "$@"
