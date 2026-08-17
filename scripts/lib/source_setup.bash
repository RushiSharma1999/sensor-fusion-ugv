#!/usr/bin/env bash

# Source a ROS/ament/colcon setup file in the current shell without exposing
# generated setup code to the caller's nounset setting. Environment changes
# made by the setup file are retained, and the caller's prior nounset state is
# restored whether sourcing succeeds or fails.
source_setup_file() {
    if [[ "$#" -ne 1 ]]; then
        echo "ERROR: source_setup_file requires exactly one setup-file path." >&2
        return 2
    fi

    local ugv_setup_file="$1"
    if [[ ! -f "${ugv_setup_file}" || ! -r "${ugv_setup_file}" ]]; then
        echo "ERROR: Setup file not found or unreadable: ${ugv_setup_file}" >&2
        return 1
    fi

    local ugv_restore_nounset=false
    case "$-" in
        *u*) ugv_restore_nounset=true ;;
    esac

    set +u
    local ugv_source_status=0
    if source "${ugv_setup_file}"; then
        ugv_source_status=0
    else
        ugv_source_status=$?
    fi

    if [[ "${ugv_restore_nounset}" == true ]]; then
        set -u
    else
        set +u
    fi

    if [[ "${ugv_source_status}" -ne 0 ]]; then
        echo "ERROR: Failed to source setup file (${ugv_source_status}): ${ugv_setup_file}" >&2
        return "${ugv_source_status}"
    fi
}
