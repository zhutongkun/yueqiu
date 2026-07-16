#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
WORKSPACE="${MOON_WORKSPACE:-$SCRIPT_DIR}"

log() {
    printf '[restore-app] %s\n' "$*"
}

# Resolve the workspace consistently with the other deployment scripts even
# though service recovery itself does not read workspace files.
: "$WORKSPACE"

die() {
    printf '[restore-app] ERROR: %s\n' "$*" >&2
    exit 1
}

if [ "${1:-}" != "--confirm" ]; then
    printf 'This script is intentionally manual.\n' >&2
    printf 'After confirming the robot is parked and the competition launch has exited, run:\n' >&2
    printf '  %s --confirm\n' "$0" >&2
    exit 2
fi

command -v systemctl >/dev/null 2>&1 || die "systemctl is unavailable"
if [ "$(id -u)" -eq 0 ]; then
    systemctl restart start_app_node.service
else
    command -v sudo >/dev/null 2>&1 || die "sudo is required"
    sudo systemctl restart start_app_node.service
fi

systemctl is-active --quiet start_app_node.service || die "start_app_node.service did not become active"
log "start_app_node.service is active"
