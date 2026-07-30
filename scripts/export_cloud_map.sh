#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DB_PATH="${1:-${DB_PATH:-$HOME/.ros/rtabmap.db}}"
OUTPUT_NAME="${2:-${OUTPUT_NAME:-rtabmap_$(date +%Y%m%d_%H%M%S)}}"
OUTPUT_DIR="${OUTPUT_DIR:-$ROOT_DIR/cloud_map}"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
EXPORT_VOXEL="${EXPORT_VOXEL:-0}"
EXPORT_ASCII="${EXPORT_ASCII:-0}"

set +u
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  source "$ROOT_DIR/install/setup.bash"
fi
set -u

find_rtabmap_export() {
  if command -v rtabmap-export >/dev/null 2>&1; then
    command -v rtabmap-export
    return 0
  fi

  local candidate
  for candidate in \
    "$ROOT_DIR/install/bin/rtabmap-export" \
    "/opt/ros/${ROS_DISTRO_NAME}/bin/rtabmap-export"; do
    if [[ -x "$candidate" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done

  return 1
}

if [[ ! -f "$DB_PATH" ]]; then
  echo "[ERROR] Database not found: $DB_PATH" >&2
  exit 1
fi

RTABMAP_EXPORT_BIN="$(find_rtabmap_export || true)"
if [[ -z "$RTABMAP_EXPORT_BIN" ]]; then
  echo "[ERROR] Cannot find rtabmap-export in current environment." >&2
  echo "[HINT] Source the workspace containing RTAB-Map tools, or build/install them first." >&2
  exit 2
fi

mkdir -p "$OUTPUT_DIR"

cmd=(
  "$RTABMAP_EXPORT_BIN"
  "--cloud"
  "--scan"
  "--output" "$OUTPUT_NAME"
  "--output_dir" "$OUTPUT_DIR"
)

if [[ "$EXPORT_ASCII" == "1" ]]; then
  cmd+=("--ascii")
fi

if [[ -n "$EXPORT_VOXEL" && "$EXPORT_VOXEL" != "0" ]]; then
  cmd+=("--voxel" "$EXPORT_VOXEL")
fi

cmd+=("$DB_PATH")

echo "[INFO] Database   : $DB_PATH"
echo "[INFO] Output dir : $OUTPUT_DIR"
echo "[INFO] Output base: $OUTPUT_NAME"
echo "[INFO] Command    : ${cmd[*]}"

"${cmd[@]}"

echo "[INFO] Export done."
echo "[INFO] Cloud file : $OUTPUT_DIR/${OUTPUT_NAME}_cloud.ply"
