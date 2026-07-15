#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
WORKSPACE="${MOON_WORKSPACE:-$SCRIPT_DIR}"

log() {
    printf '[stop-moon] %s\n' "$*"
}

warn() {
    printf '[stop-moon] WARNING: %s\n' "$*" >&2
}

source_setup() {
    set +u
    # shellcheck disable=SC1090
    source "$1"
    set -u
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

service_available() {
    printf '%s\n' "$SERVICE_LIST" | grep -Fqx "$1"
}

call_stop_service() {
    service_name="$1"
    if service_available "$service_name"; then
        if timeout 4s rosservice call "$service_name" "{}"; then
            log "Called $service_name"
        else
            warn "Call failed: $service_name"
        fi
    else
        warn "Service is not currently available: $service_name"
    fi
}

ROS_SETUP="$(find_ros_setup 2>/dev/null || true)"
if [ -n "$ROS_SETUP" ]; then
    source_setup "$ROS_SETUP"
fi
if [ -r "$WORKSPACE/devel/setup.bash" ]; then
    source_setup "$WORKSPACE/devel/setup.bash"
fi

if ! command -v rosservice >/dev/null 2>&1 || ! command -v rostopic >/dev/null 2>&1; then
    warn "ROS command-line tools are unavailable; cannot send software stop requests"
    exit 0
fi

if ! command -v timeout >/dev/null 2>&1; then
    warn "GNU timeout is unavailable; refusing commands that could block during a safety stop"
    exit 0
fi

SERVICE_LIST="$(timeout 4s rosservice list 2>/dev/null || true)"

call_stop_service /competition/stop_mission
call_stop_service /moon_detector/stop
call_stop_service /ramp/stop
call_stop_service /position_correction/stop
call_stop_service /shape_recognition/stop
call_stop_service /yolov5/stop
call_stop_service /moon_detector/unload
call_stop_service /yolov5/unload

timeout 4s rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID "{}" >/dev/null 2>&1 ||
    warn "Could not publish move_base cancellation"
timeout 4s rostopic pub -1 /controller/cmd_vel geometry_msgs/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 ||
    warn "Could not publish zero velocity"

log "Safety stop requests completed. Verify the chassis is stationary before approaching the robot."
