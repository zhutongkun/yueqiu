#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
WORKSPACE="${MOON_WORKSPACE:-$SCRIPT_DIR}"
PYTHON_BIN="${MOON_PYTHON:-python3}"
ROSLAUNCH_PID=""

log() {
    printf '[run-moon] %s\n' "$*"
}

warn() {
    printf '[run-moon] WARNING: %s\n' "$*" >&2
}

die() {
    printf '[run-moon] ERROR: %s\n' "$*" >&2
    exit 1
}

find_ros_setup() {
    if [ -n "${MOON_ROS_SETUP:-}" ]; then
        [ -r "$MOON_ROS_SETUP" ] || return 1
        printf '%s\n' "$MOON_ROS_SETUP"
        return 0
    fi

    if [ -n "${ROS_DISTRO:-}" ] && [ -r "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
        printf '%s\n' "/opt/ros/$ROS_DISTRO/setup.bash"
        return 0
    fi
    for candidate in \
        /opt/ros/noetic/setup.bash \
        /opt/ros/melodic/setup.bash \
        /opt/ros_ws/noetic/setup.bash \
        /opt/ros_ws/melodic/setup.bash \
        /opt/ros/*/setup.bash; do
        if [ -r "$candidate" ]; then
            printf '%s\n' "$candidate"
            return 0
        fi
    done
    return 1
}

systemctl_as_root() {
    if [ "$(id -u)" -eq 0 ]; then
        systemctl "$@"
    else
        command -v sudo >/dev/null 2>&1 || die "sudo is required to manage start_app_node.service"
        sudo systemctl "$@"
    fi
}

cleanup() {
    exit_code=$?
    trap - EXIT INT TERM
    log "Stopping mission interfaces and publishing zero velocity"
    "$SCRIPT_DIR/stop_moon_competition.sh" || warn "Safety stop returned an error; inspect ROS and controller state immediately"
    if [ -n "$ROSLAUNCH_PID" ] && kill -0 "$ROSLAUNCH_PID" 2>/dev/null; then
        kill -INT "$ROSLAUNCH_PID" 2>/dev/null || true
        wait "$ROSLAUNCH_PID" 2>/dev/null || true
    fi
    log "APP service remains stopped. Restore it manually with restore_app_service.sh --confirm after the robot is safe."
    exit "$exit_code"
}

[ -d "$WORKSPACE/src/competition" ] || die "competition package not found under $WORKSPACE"
command -v "$PYTHON_BIN" >/dev/null 2>&1 || die "$PYTHON_BIN is not installed"
"$PYTHON_BIN" -c 'import sys; raise SystemExit(0 if sys.version_info >= (3, 8) else 1)' ||
    die "Moon YOLOv5 requires Python >= 3.8; found $("$PYTHON_BIN" --version 2>&1)"
ROS_SETUP="$(find_ros_setup)" || die "No ROS1 installation found under /opt/ros"
# shellcheck disable=SC1090
source "$ROS_SETUP"
[ "${ROS_VERSION:-}" = "1" ] || die "ROS1 is required"
[ -r "$WORKSPACE/devel/setup.bash" ] || die "Missing $WORKSPACE/devel/setup.bash; run setup_moon_runtime.sh first"
# shellcheck disable=SC1090
source "$WORKSPACE/devel/setup.bash"
command -v roslaunch >/dev/null 2>&1 || die "roslaunch is not available"
rospack find competition >/dev/null 2>&1 || die "competition package is not visible in the sourced workspace"

if command -v systemctl >/dev/null 2>&1; then
    APP_STATUS="$(systemctl is-active start_app_node.service 2>/dev/null || true)"
    [ -n "$APP_STATUS" ] || die "Unable to query start_app_node.service; verify systemd before launching robot hardware"
    log "start_app_node.service status before launch: ${APP_STATUS:-unknown}"
    if [ "$APP_STATUS" = "active" ] || [ "$APP_STATUS" = "activating" ]; then
        systemctl_as_root stop start_app_node.service
    fi
    APP_STATUS="$(systemctl is-active start_app_node.service 2>/dev/null || true)"
    [ -n "$APP_STATUS" ] || die "Unable to verify start_app_node.service after stop request"
    case "$APP_STATUS" in
        active|activating) die "start_app_node.service is still active and may own robot hardware" ;;
        unknown|not-found) warn "start_app_node.service is not installed on this image" ;;
        *) log "start_app_node.service is not active" ;;
    esac
else
    die "systemctl is unavailable; cannot verify the conflicting APP service"
fi

mkdir -p "$WORKSPACE/runtime/ros_logs"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$WORKSPACE/runtime/ros_logs}"
export YOLOv5_AUTOINSTALL=false
export PIP_CONFIG_FILE=/dev/null
export PIP_DISABLE_PIP_VERSION_CHECK=1
export PIP_FIND_LINKS=""
export PIP_NO_CACHE_DIR=1
export PIP_NO_INDEX=1
export ULTRALYTICS_OFFLINE=true
trap cleanup EXIT INT TERM

log "Launching competition/position_correction_pick.launch"
roslaunch competition position_correction_pick.launch "$@" &
ROSLAUNCH_PID=$!
wait "$ROSLAUNCH_PID"
