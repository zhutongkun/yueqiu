#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
WORKSPACE="${MOON_WORKSPACE:-$SCRIPT_DIR}"
PYTHON_BIN="${MOON_PYTHON:-python3}"
MODEL_PATH="${MOON_MODEL_PATH:-$WORKSPACE/src/competition/models/moon/best.pt}"
EXPECTED_MODEL_SHA256="${MOON_MODEL_SHA256:-ab953a754cc6ea68742d49ccf26d8b122bb771fe65aa6bd582761bdeaaa7da34}"
YOLOV5_DIR="$WORKSPACE/src/competition/third_party/yolov5"
EXPORT_SCRIPT="$YOLOV5_DIR/export.py"
ENGINE_PATH="${MODEL_PATH%.pt}.engine"
WORKSPACE_GB="${MOON_TENSORRT_WORKSPACE_GB:-2}"
ONNX_OPSET="${MOON_ONNX_OPSET:-12}"
OUTPUT_DIR="$WORKSPACE/runtime/tensorrt"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
METADATA_FILE="$OUTPUT_DIR/build_${STAMP}.txt"
BUILD_LOG="$OUTPUT_DIR/build_${STAMP}.log"

log() {
    printf '[build-moon-trt] %s\n' "$*"
}

die() {
    printf '[build-moon-trt] ERROR: %s\n' "$*" >&2
    exit 1
}

[ -r /proc/device-tree/model ] || die "Run this script on the target Jetson only"
JETSON_MODEL="$(tr -d '\000' </proc/device-tree/model)"
case "$JETSON_MODEL" in
    *Jetson*) ;;
    *) die "Device is not a Jetson: $JETSON_MODEL" ;;
esac
[ "$(uname -m)" = "aarch64" ] || die "TensorRT engine must be built on the target aarch64 Jetson"
[ -f "$EXPORT_SCRIPT" ] || die "Vendored YOLOv5 export.py is missing: $EXPORT_SCRIPT"
[ -f "$MODEL_PATH" ] || die "Source model is missing: $MODEL_PATH"
[ "$(sha256sum "$MODEL_PATH" | awk '{print $1}')" = "$EXPECTED_MODEL_SHA256" ] || die "best.pt SHA256 mismatch"
command -v "$PYTHON_BIN" >/dev/null 2>&1 || die "$PYTHON_BIN is unavailable"

if [ -e "$ENGINE_PATH" ] && [ "${MOON_OVERWRITE_ENGINE:-0}" != "1" ]; then
    die "$ENGINE_PATH already exists; set MOON_OVERWRITE_ENGINE=1 only after preserving its metadata"
fi

"$PYTHON_BIN" - <<'PY'
from packaging.version import parse

import onnx
import onnxscript
import torch
import tensorrt

print("PyTorch:", torch.__version__)
print("TensorRT:", tensorrt.__version__)
print("ONNX:", onnx.__version__)
print("ONNXScript:", getattr(onnxscript, "__version__", "unknown"))
if parse(onnx.__version__) < parse("1.12.0"):
    raise SystemExit("ONNX 1.12.0 or newer is required by the vendored exporter")
print("CUDA available:", torch.cuda.is_available())
if not torch.cuda.is_available():
    raise SystemExit("CUDA is unavailable; refusing to build a Jetson TensorRT engine")
print("CUDA device:", torch.cuda.get_device_name(0))
PY

mkdir -p "$OUTPUT_DIR"
{
    printf 'built_at_utc=%s\n' "$STAMP"
    printf 'workspace=%s\n' "$WORKSPACE"
    printf 'jetson_model=%s\n' "$JETSON_MODEL"
    printf 'architecture=%s\n' "$(uname -m)"
    printf 'kernel=%s\n' "$(uname -a)"
    printf 'model_path=%s\n' "$MODEL_PATH"
    printf 'model_sha256=%s\n' "$EXPECTED_MODEL_SHA256"
    printf 'export_script=%s\n' "$EXPORT_SCRIPT"
    printf 'image_size=640x640\n'
    printf 'precision=FP16\n'
    printf 'onnx_opset=%s\n' "$ONNX_OPSET"
    printf 'workspace_gb=%s\n' "$WORKSPACE_GB"
    printf 'classes=0:satellite,1:space_station,2:lunar_crater,3:lunar_rover,4:meteorite,5:earth,6:lunar_soil,7:moon,8:rocket,9:astronaut\n'
    if [ -r /etc/nv_tegra_release ]; then
        printf 'l4t=%s\n' "$(head -n 1 /etc/nv_tegra_release)"
    fi
    if command -v nvcc >/dev/null 2>&1; then
        nvcc --version
    fi
    if command -v dpkg-query >/dev/null 2>&1; then
        dpkg-query -W 'nvidia-jetpack' 'nvidia-l4t-core' 'tensorrt' 'libnvinfer*' 2>/dev/null || true
    fi
    "$PYTHON_BIN" - <<'PY'
import cv2
import onnx
import onnxscript
import tensorrt
import torch
import torchvision

print("opencv=" + cv2.__version__)
print("onnx=" + onnx.__version__)
print("onnxscript=" + getattr(onnxscript, "__version__", "unknown"))
print("tensorrt=" + tensorrt.__version__)
print("torch=" + torch.__version__)
print("torchvision=" + torchvision.__version__)
PY
    if command -v git >/dev/null 2>&1 && git -C "$WORKSPACE" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
        printf 'git_commit=%s\n' "$(git -C "$WORKSPACE" rev-parse HEAD)"
    fi
} >"$METADATA_FILE"

export YOLOV5_AUTOINSTALL=false
# The vendored exporter contains dependency auto-install fallbacks. Disable all
# package indexes and caches so an unmet version fails instead of mutating the
# target Jetson environment.
export PIP_CONFIG_FILE=/dev/null
export PIP_DISABLE_PIP_VERSION_CHECK=1
export PIP_FIND_LINKS=""
export PIP_NO_CACHE_DIR=1
export PIP_NO_INDEX=1
log "Building FP16 engine locally on $JETSON_MODEL"
(
    cd "$YOLOV5_DIR"
    "$PYTHON_BIN" "$EXPORT_SCRIPT" \
        --weights "$MODEL_PATH" \
        --include engine \
        --imgsz 640 640 \
        --batch-size 1 \
        --device 0 \
        --half \
        --opset "$ONNX_OPSET" \
        --workspace "$WORKSPACE_GB"
) 2>&1 | tee "$BUILD_LOG"

[ -s "$ENGINE_PATH" ] || die "YOLOv5 export completed without creating $ENGINE_PATH"
{
    printf 'engine_path=%s\n' "$ENGINE_PATH"
    printf 'engine_size_bytes=%s\n' "$(stat -c %s "$ENGINE_PATH")"
    printf 'engine_sha256=%s\n' "$(sha256sum "$ENGINE_PATH" | awk '{print $1}')"
    printf 'build_log=%s\n' "$BUILD_LOG"
} >>"$METADATA_FILE"

log "Engine created: $ENGINE_PATH"
log "Build metadata: $METADATA_FILE"
log "Do not copy this engine to a different Jetson or TensorRT/CUDA version. Validate all ten classes before enabling it."
