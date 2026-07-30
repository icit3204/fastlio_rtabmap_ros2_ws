#!/usr/bin/env bash
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  set -euo pipefail
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RTABMAP_INSTALL_DIR="$ROOT_DIR/third_party/rtabmap-0.23.4/install"

if [[ ! -d "$RTABMAP_INSTALL_DIR" ]]; then
  echo "[ERROR] Install dir not found: $RTABMAP_INSTALL_DIR" >&2
  echo "[HINT] Build first: $ROOT_DIR/scripts/build_rtabmap_0234.sh" >&2
  return 1 2>/dev/null || exit 1
fi

RTABMAP_CONFIG=$(find "$RTABMAP_INSTALL_DIR" -name RTABMapConfig.cmake | head -n 1 || true)
if [[ -z "$RTABMAP_CONFIG" ]]; then
  echo "[ERROR] RTABMapConfig.cmake not found under $RTABMAP_INSTALL_DIR" >&2
  return 1 2>/dev/null || exit 1
fi

export RTABMap_DIR="$(dirname "$RTABMAP_CONFIG")"
export CMAKE_PREFIX_PATH="$RTABMAP_INSTALL_DIR:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$RTABMAP_INSTALL_DIR/lib:${LD_LIBRARY_PATH:-}"

detect_torch_lib_dir() {
  local candidates=(
    "/home/dog/.local/lib/python3.10/site-packages/torch/lib"
    "/home/dog/.conda/envs/py310/lib/python3.10/site-packages/torch/lib"
    "/home/dog/miniforge3/envs/py310/lib/python3.10/site-packages/torch/lib"
  )
  local d
  for d in "${candidates[@]}"; do
    if [[ -f "$d/libtorch.so" ]]; then
      echo "$d"
      return 0
    fi
  done
  return 1
}

TORCH_LIB_DIR="${RTABMAP_TORCH_LIB_DIR:-$(detect_torch_lib_dir || true)}"
if [[ -n "$TORCH_LIB_DIR" ]]; then
  export LD_LIBRARY_PATH="$TORCH_LIB_DIR:$LD_LIBRARY_PATH"
fi

echo "[OK] RTABMap_DIR=$RTABMap_DIR"
echo "[OK] CMAKE_PREFIX_PATH prepended with $RTABMAP_INSTALL_DIR"
echo "[OK] LD_LIBRARY_PATH prepended with $RTABMAP_INSTALL_DIR/lib"
