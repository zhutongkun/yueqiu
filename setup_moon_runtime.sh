#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
WORKSPACE="${MOON_WORKSPACE:-$SCRIPT_DIR}"
MODEL_PATH="${MOON_MODEL_PATH:-$WORKSPACE/src/competition/models/moon/best.pt}"
EXPECTED_MODEL_SHA256="${MOON_MODEL_SHA256:-ab953a754cc6ea68742d49ccf26d8b122bb771fe65aa6bd582761bdeaaa7da34}"
PYTHON_BIN="${MOON_PYTHON:-python3}"
BUILD_TOOL_REQUEST="${MOON_CATKIN_BUILD_TOOL:-auto}"

log() {
    printf '[setup-moon] %s\n' "$*"
}

warn() {
    printf '[setup-moon] WARNING: %s\n' "$*" >&2
}

die() {
    printf '[setup-moon] ERROR: %s\n' "$*" >&2
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

select_build_tool() {
    case "$BUILD_TOOL_REQUEST" in
        auto)
            if [ -d "$WORKSPACE/.catkin_tools" ]; then
                printf '%s\n' catkin
            else
                printf '%s\n' catkin_make
            fi
            ;;
        catkin|catkin_build)
            printf '%s\n' catkin
            ;;
        catkin_make)
            printf '%s\n' catkin_make
            ;;
        *)
            die "MOON_CATKIN_BUILD_TOOL must be auto, catkin, catkin_build, or catkin_make"
            ;;
    esac
}

module_exists() {
    "$PYTHON_BIN" -c "import importlib.util, sys; sys.exit(0 if importlib.util.find_spec('$1') else 1)" >/dev/null 2>&1
}

install_pure_python_dependency() {
    module_name="$1"
    package_spec="$2"
    if module_exists "$module_name"; then
        return 0
    fi

    log "Installing missing pure-Python dependency without dependencies: $package_spec"
    "$PYTHON_BIN" -m pip --version >/dev/null 2>&1 ||
        die "$PYTHON_BIN has no usable pip module; install $package_spec from the target image's trusted package source"
    "$PYTHON_BIN" -m pip --disable-pip-version-check install --user --no-deps "$package_spec" ||
        die "Could not install $package_spec. Install a target-image-compatible package and rerun."
    "$PYTHON_BIN" -c "import $module_name" >/dev/null 2>&1 ||
        die "$package_spec was installed but cannot be imported in $PYTHON_BIN"
}

install_exact_pure_python_dependency() {
    module_name="$1"
    package_name="$2"
    package_spec="$3"
    expected_version="$4"
    installed_version=""
    if module_exists "$module_name"; then
        installed_version="$("$PYTHON_BIN" -c "from importlib.metadata import version; print(version('$package_name'))" 2>/dev/null || true)"
        if [ "$installed_version" = "$expected_version" ]; then
            return 0
        fi
        warn "$package_name version '$installed_version' does not match required $expected_version"
    fi

    log "Installing pinned pure-Python dependency without dependencies: $package_spec"
    "$PYTHON_BIN" -m pip --version >/dev/null 2>&1 ||
        die "$PYTHON_BIN has no usable pip module; install $package_spec from a trusted source"
    "$PYTHON_BIN" -m pip --disable-pip-version-check install --user --no-deps \
        "$package_spec" || die "Could not install $package_spec"
    installed_version="$("$PYTHON_BIN" -c "from importlib.metadata import version; print(version('$package_name'))" 2>/dev/null || true)"
    [ "$installed_version" = "$expected_version" ] ||
        die "$package_name version check failed: expected $expected_version, got '$installed_version'"
}

require_python_import() {
    import_name="$1"
    guidance="$2"
    "$PYTHON_BIN" -c "import $import_name" >/dev/null 2>&1 || die "$guidance"
}

[ -d "$WORKSPACE/src" ] || die "Not a catkin workspace: $WORKSPACE/src is missing"
[ -f "$WORKSPACE/src/competition/package.xml" ] || die "competition package is missing under $WORKSPACE/src"
command -v "$PYTHON_BIN" >/dev/null 2>&1 || die "$PYTHON_BIN is not installed"
"$PYTHON_BIN" -c 'import sys; raise SystemExit(0 if sys.version_info >= (3, 8) else 1)' ||
    die "Moon YOLOv5 requires Python >= 3.8; found $("$PYTHON_BIN" --version 2>&1)"
log "Moon Python: $("$PYTHON_BIN" --version 2>&1)"
[ -r /proc/device-tree/model ] || die "Jetson device-tree model is unavailable; run this script on the target robot"

JETSON_MODEL="$(tr -d '\000' </proc/device-tree/model)"
case "$JETSON_MODEL" in
    *Jetson*) ;;
    *) die "Unsupported target '$JETSON_MODEL'; Tensor and camera dependencies must be checked on the Jetson" ;;
esac
[ "$(uname -m)" = "aarch64" ] || die "Expected aarch64 Jetson, found $(uname -m)"
log "Target: $JETSON_MODEL"

[ -f "$MODEL_PATH" ] || die "Moon model is missing: $MODEL_PATH"
command -v sha256sum >/dev/null 2>&1 || die "sha256sum is required"
ACTUAL_MODEL_SHA256="$(sha256sum "$MODEL_PATH" | awk '{print $1}')"
[ "$ACTUAL_MODEL_SHA256" = "$EXPECTED_MODEL_SHA256" ] ||
    die "Moon model SHA256 mismatch: expected $EXPECTED_MODEL_SHA256, got $ACTUAL_MODEL_SHA256"
log "Model SHA256 verified: $ACTUAL_MODEL_SHA256"
stat "$MODEL_PATH"

ROS_SETUP="$(find_ros_setup)" || die "No ROS1 setup.bash found under /opt/ros"
# shellcheck disable=SC1090
source "$ROS_SETUP"
[ "${ROS_VERSION:-}" = "1" ] || die "ROS_VERSION must be 1, found '${ROS_VERSION:-unset}'"
[ -n "${ROS_DISTRO:-}" ] || die "ROS_DISTRO is unset after sourcing $ROS_SETUP"
BUILD_TOOL="$(select_build_tool)"
if [ "$BUILD_TOOL" = "catkin" ]; then
    command -v catkin >/dev/null 2>&1 || die "catkin_tools is required by the existing workspace or MOON_CATKIN_BUILD_TOOL override"
else
    command -v catkin_make >/dev/null 2>&1 || die "catkin_make is not installed or not on PATH"
    if [ ! -e "$WORKSPACE/src/CMakeLists.txt" ]; then
        command -v catkin_init_workspace >/dev/null 2>&1 || die "catkin_init_workspace is required to initialise this catkin_make workspace"
    fi
fi
command -v roscore >/dev/null 2>&1 || die "roscore is not installed or not on PATH"
command -v rostopic >/dev/null 2>&1 || die "rostopic is not installed or not on PATH"
command -v rosservice >/dev/null 2>&1 || die "rosservice is not installed or not on PATH"
log "ROS: version=$ROS_VERSION distro=$ROS_DISTRO setup=$ROS_SETUP build_tool=$BUILD_TOOL"

module_exists torch || die "PyTorch is missing. Install the NVIDIA JetPack-compatible PyTorch build; this script will not install or upgrade torch."
module_exists cv2 || die "OpenCV is missing. Install the ROS/cv_bridge-compatible system OpenCV; this script will not install or upgrade opencv."
module_exists numpy || die "NumPy is missing. Install the version compatible with the target OpenCV/PyTorch stack."
module_exists torchvision || die "torchvision is missing. Install the build matched to the installed NVIDIA PyTorch package."

# These packages contain Python code only in the runtime path used here.  --no-deps
# prevents pip from replacing JetPack, CUDA, torch, NumPy, or OpenCV packages.
install_pure_python_dependency packaging "packaging"
install_pure_python_dependency requests "requests"
install_pure_python_dependency tqdm "tqdm"
install_exact_pure_python_dependency ultralytics ultralytics "ultralytics==8.4.83" 8.4.83

# All permitted setup-time installations are complete. From this point the
# validation and competition runtime are strictly offline and non-mutating.
export YOLOv5_AUTOINSTALL=false
export PIP_CONFIG_FILE=/dev/null
export PIP_DISABLE_PIP_VERSION_CHECK=1
export PIP_FIND_LINKS=""
export PIP_NO_CACHE_DIR=1
export PIP_NO_INDEX=1
export ULTRALYTICS_OFFLINE=true

# Do not auto-install packages with native extensions or target-image bindings.
# They must come from a source compatible with the Jetson Python/NumPy ABI.
for module_name in yaml psutil PIL pandas matplotlib pycuda sklearn transforms3d scipy seaborn message_filters; do
    module_exists "$module_name" || die "$module_name is missing; install a Jetson/Python-compatible build without upgrading torch, NumPy, or OpenCV"
done

require_python_import pycuda.driver "pycuda.driver cannot be imported; install the JetPack/CUDA-matched PyCUDA build"
require_python_import tensorrt "TensorRT Python bindings cannot be imported; install the bindings supplied with JetPack"
require_python_import sklearn "scikit-learn cannot be imported; install a target-compatible build for ramp and shape recognition"
require_python_import transforms3d "transforms3d cannot be imported; install it for shape recognition"
require_python_import scipy "SciPy cannot be imported; install a target-compatible build"
require_python_import seaborn "seaborn cannot be imported; install it for the vendored YOLOv5 runtime"
require_python_import message_filters "ROS1 message_filters cannot be imported in $PYTHON_BIN; verify the selected ROS underlay supports Python 3"
require_python_import ultralytics "ultralytics==8.4.83 is installed but cannot be imported; resolve its pure-Python dependencies before deployment"

"$PYTHON_BIN" - <<'PY'
import cv2
import numpy
import torch
import torchvision
import rospy
import yaml
from cv_bridge import CvBridge

print("OpenCV:", cv2.__version__)
print("NumPy:", numpy.__version__)
print("PyTorch:", torch.__version__)
print("torchvision:", torchvision.__version__)
print("PyYAML:", yaml.__version__)
print("CUDA available:", torch.cuda.is_available())
if torch.cuda.is_available():
    print("CUDA device:", torch.cuda.get_device_name(0))
else:
    print("WARNING: CUDA is unavailable; moon_detector device:auto will use the slower CPU path")
CvBridge()
print("ROS Python and cv_bridge imports: OK")
PY

export MOON_MODEL_PATH_FOR_CHECK="$MODEL_PATH"
export MOON_YOLOV5_DIR_FOR_CHECK="$WORKSPACE/src/competition/third_party/yolov5"
"$PYTHON_BIN" - <<'PY'
import os
import sys

import torch

yolov5_dir = os.environ["MOON_YOLOV5_DIR_FOR_CHECK"]
model_path = os.environ["MOON_MODEL_PATH_FOR_CHECK"]
os.environ["YOLOV5_AUTOINSTALL"] = "false"
sys.path.insert(0, yolov5_dir)

from models.common import DetectMultiBackend

expected = [
    "satellite",
    "space_station",
    "lunar_crater",
    "lunar_rover",
    "meteorite",
    "earth",
    "lunar_soil",
    "moon",
    "rocket",
    "astronaut",
]
model = DetectMultiBackend(model_path, device=torch.device("cpu"), fp16=False)
names = model.names
actual = [names[index] for index in range(len(names))] if isinstance(names, dict) else list(names)
if actual != expected:
    raise SystemExit("Moon model class mapping mismatch: %r" % (actual,))
print("Vendored YOLOv5 model load and ten-class mapping: OK")
PY

if [ -r /etc/nv_tegra_release ]; then
    log "L4T: $(head -n 1 /etc/nv_tegra_release)"
else
    warn "/etc/nv_tegra_release is unavailable; record JetPack/L4T manually"
fi
if command -v nvcc >/dev/null 2>&1; then
    nvcc --version | tail -n 1
else
    warn "nvcc is not installed; CUDA runtime may still be present"
fi
if command -v dpkg-query >/dev/null 2>&1; then
    dpkg-query -W 'nvidia-jetpack' 'nvidia-l4t-core' 'tensorrt' 'libnvinfer*' 2>/dev/null || true
fi

chmod +x \
    "$WORKSPACE/setup_moon_runtime.sh" \
    "$WORKSPACE/run_moon_competition.sh" \
    "$WORKSPACE/stop_moon_competition.sh" \
    "$WORKSPACE/restore_app_service.sh" \
    "$WORKSPACE/build_moon_tensorrt_engine.sh" \
    "$WORKSPACE/src/competition/scripts/navigation_transport/moon_detector.py" \
    "$WORKSPACE/src/competition/scripts/navigation_transport/voice_control_navigation.py"

if [ "$BUILD_TOOL" = "catkin_make" ] && [ ! -e "$WORKSPACE/src/CMakeLists.txt" ]; then
    log "Initialising missing catkin workspace marker: $WORKSPACE/src/CMakeLists.txt"
    (cd "$WORKSPACE/src" && catkin_init_workspace)
fi

log "Building catkin workspace: $WORKSPACE"
if [ "$BUILD_TOOL" = "catkin" ]; then
    (cd "$WORKSPACE" && catkin build)
else
    (cd "$WORKSPACE" && catkin_make)
fi

[ -r "$WORKSPACE/devel/setup.bash" ] || die "$BUILD_TOOL completed without devel/setup.bash"
# shellcheck disable=SC1090
source "$WORKSPACE/devel/setup.bash"
rospack find competition >/dev/null
"$PYTHON_BIN" - <<'PY'
import message_filters
import kinematics.transform
import sdk.common
from interfaces.msg import Pose2D

print("Workspace Python imports: message_filters, interfaces, sdk, kinematics OK")
PY
log "Setup and catkin build completed. Run $WORKSPACE/run_moon_competition.sh on the robot."
