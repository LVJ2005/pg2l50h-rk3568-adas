#!/bin/sh
set -eu
PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PATH
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$SCRIPT_DIR"
if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: run as root (sudo sh run.sh)."
    exit 1
fi
INSMOD=$(command -v insmod 2>/dev/null || true)
RMMOD=$(command -v rmmod 2>/dev/null || true)
if [ -z "$INSMOD" ] || [ -z "$RMMOD" ]; then
    echo "ERROR: install kmod; insmod/rmmod are required."
    exit 1
fi
echo "[1/4] Build existing full-frame PCIe driver"
(cd driver && make clean && make)
echo "[2/4] Build ADAS application"
(cd app_pcie && make clean && make)
if lsmod | grep -q '^pango_pci_driver[[:space:]]'; then
    "$RMMOD" pango_pci_driver
fi
echo "[3/4] Load full-frame MSI ping-pong driver"
"$INSMOD" driver/pango_pci_driver.ko \
    dma_line_delay_us="${DMA_LINE_DELAY_US:-20}" \
    dma_frame_tail_delay_us="${DMA_FRAME_TAIL_DELAY_US:-300}" \
    frame_batch_mode=1
echo "[4/4] Start quad-frame ADAS (FP detector + FP classifier by default)"
cd app_pcie/build
export PCIE_USE_MMAP="${PCIE_USE_MMAP:-1}"
export FPGA_BGR888=1
export ADAS_LANE_DEBUG="${ADAS_LANE_DEBUG:-0}"
export ADAS_LANE_CONTRAST="${ADAS_LANE_CONTRAST:-12}"
export ADAS_LANE_SCAN_ROWS="${ADAS_LANE_SCAN_ROWS:-100}"
export ADAS_DISPLAY_BACKEND="${ADAS_DISPLAY_BACKEND:-opencv}"
export ADAS_CAMERA_ORDER="${ADAS_CAMERA_ORDER:-rbg}"
export ADAS_VIEW_ORDER="${ADAS_VIEW_ORDER:-$ADAS_CAMERA_ORDER}"
export ADAS_INFER_ORDER="${ADAS_INFER_ORDER:-$ADAS_CAMERA_ORDER}"
export ADAS_COLOR_COMPARE="${ADAS_COLOR_COMPARE:-0}"
export ADAS_MODEL_DIR="${ADAS_MODEL_DIR:-../models}"
export ADAS_MODE="${ADAS_MODE:-assist}"
export ADAS_DET_PROFILE="${ADAS_DET_PROFILE:-fp}"
export ADAS_CLS_PROFILE="${ADAS_CLS_PROFILE:-fp}"
export ADAS_INFER_INTERVAL="${ADAS_INFER_INTERVAL:-1}"
export ADAS_LANE_HZ="${ADAS_LANE_HZ:-8}"
export ADAS_FRONT_VIEW=0
export ADAS_SIGNAL_CONFIRMED="${ADAS_SIGNAL_CONFIRMED:-1}"
export RECOGNITION_CONTROL_BIND="${RECOGNITION_CONTROL_BIND:-0.0.0.0}"
export RECOGNITION_CONTROL_PORT="${RECOGNITION_CONTROL_PORT:-9527}"
exec ./app "$@"
