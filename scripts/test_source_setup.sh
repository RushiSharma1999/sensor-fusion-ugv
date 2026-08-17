#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
UGV_SETUP_HELPER="${SCRIPT_DIR}/lib/source_setup.bash"

if [[ ! -f "${UGV_SETUP_HELPER}" ]]; then
    echo "ERROR: Setup-source helper not found: ${UGV_SETUP_HELPER}" >&2
    exit 1
fi
source "${UGV_SETUP_HELPER}"

UGV_SETUP_TEST_DIR="$(mktemp -d "${TMPDIR:-/tmp}/ugv-source-setup.XXXXXX")"
cleanup_setup_test() {
    if [[ -n "${UGV_SETUP_TEST_DIR}" && -d "${UGV_SETUP_TEST_DIR}" ]]; then
        rm -rf -- "${UGV_SETUP_TEST_DIR}"
    fi
}
trap cleanup_setup_test EXIT

UGV_MOCK_SETUP="${UGV_SETUP_TEST_DIR}/mock_setup.bash"
printf '%s\n' \
    'if [[ -n "${AMENT_TRACE_SETUP_FILES}" ]]; then' \
    '    : "mock trace enabled"' \
    'fi' \
    'export UGV_MOCK_SETUP_SOURCED=1' > "${UGV_MOCK_SETUP}"

unset AMENT_TRACE_SETUP_FILES UGV_MOCK_SETUP_SOURCED
source_setup_file "${UGV_MOCK_SETUP}"
if [[ "${UGV_MOCK_SETUP_SOURCED:-}" != 1 ]]; then
    echo "ERROR: Mock setup did not modify the current shell." >&2
    exit 1
fi
case "$-" in
    *u*) ;;
    *)
        echo "ERROR: nounset was not restored after successful setup sourcing." >&2
        exit 1
        ;;
esac
if ( : "${UGV_PROBE_MUST_REMAIN_UNSET}" ) 2>/dev/null; then
    echo "ERROR: nounset was reported enabled but did not reject an unset probe." >&2
    exit 1
fi

set +u
unset AMENT_TRACE_SETUP_FILES UGV_MOCK_SETUP_SOURCED
source_setup_file "${UGV_MOCK_SETUP}"
case "$-" in
    *u*)
        echo "ERROR: helper enabled nounset although the caller had it disabled." >&2
        exit 1
        ;;
esac
set -u

UGV_MISSING_SETUP="${UGV_SETUP_TEST_DIR}/missing_setup.bash"
UGV_MISSING_LOG="${UGV_SETUP_TEST_DIR}/missing_setup.log"
if source_setup_file "${UGV_MISSING_SETUP}" 2> "${UGV_MISSING_LOG}"; then
    echo "ERROR: Missing setup file unexpectedly succeeded." >&2
    exit 1
fi
if ! grep -Fqx \
    "ERROR: Setup file not found or unreadable: ${UGV_MISSING_SETUP}" \
    "${UGV_MISSING_LOG}"; then
    echo "ERROR: Missing setup file did not produce the expected diagnostic." >&2
    exit 1
fi
case "$-" in
    *u*) ;;
    *)
        echo "ERROR: nounset changed after the missing-file failure." >&2
        exit 1
        ;;
esac

for UGV_SCRIPT in \
    "${REPOSITORY_ROOT}/build_ws.sh" \
    "${REPOSITORY_ROOT}/scripts/verify_build.sh" \
    "${REPOSITORY_ROOT}/scripts/verify_unitree_phase2.sh" \
    "${REPOSITORY_ROOT}/run_lidar.sh"; do
    if ! grep -Fq 'source_setup_file "${UGV_ROS_SETUP}"' "${UGV_SCRIPT}"; then
        echo "ERROR: ROS setup does not use source_setup_file: ${UGV_SCRIPT}" >&2
        exit 1
    fi
done
for UGV_SCRIPT in \
    "${REPOSITORY_ROOT}/scripts/verify_build.sh" \
    "${REPOSITORY_ROOT}/run_lidar.sh"; do
    if ! grep -Fq 'source_setup_file "${WORKSPACE_DIR}/install/setup.bash"' "${UGV_SCRIPT}"; then
        echo "ERROR: Workspace setup does not use source_setup_file: ${UGV_SCRIPT}" >&2
        exit 1
    fi
done
UGV_PHASE2_SCRIPT="${REPOSITORY_ROOT}/scripts/verify_unitree_phase2.sh"
if ! grep -Fq 'source_setup_file "${UGV_PHASE2_INSTALL_DIR}/setup.bash"' \
    "${UGV_PHASE2_SCRIPT}"; then
    echo "ERROR: Focused workspace setup does not use source_setup_file: ${UGV_PHASE2_SCRIPT}" >&2
    exit 1
fi

echo "PASS: setup sourcing tolerates unset AMENT_TRACE_SETUP_FILES, restores nounset, and rejects a missing setup file."
