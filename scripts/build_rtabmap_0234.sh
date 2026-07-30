#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RTABMAP_SRC_DIR="$ROOT_DIR/third_party/rtabmap-0.23.4"
RTABMAP_BUILD_DIR="$RTABMAP_SRC_DIR/build_local"
RTABMAP_INSTALL_DIR="$RTABMAP_SRC_DIR/install"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
JOBS="${JOBS:-4}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
RTABMAP_OPENCV_DIR="${RTABMAP_OPENCV_DIR:-${OpenCV_DIR:-}}"
TORCH_ROOT="${RTABMAP_TORCH_ROOT:-${TORCH_ROOT:-}}"

detect_opencv_dir() {
  local candidates=()
  local pc_dir=""

  candidates+=(
    "/usr/lib/cmake/opencv4"
    "/usr/local/lib/cmake/opencv4"
    "/usr/local/lib64/cmake/opencv4"
    "/usr/lib/aarch64-linux-gnu/cmake/opencv4"
    "/usr/lib/x86_64-linux-gnu/cmake/opencv4"
  )

  if command -v pkg-config >/dev/null 2>&1; then
    pc_dir="$(pkg-config --variable=pcfiledir opencv4 2>/dev/null || true)"
    if [[ -n "$pc_dir" ]]; then
      candidates+=("${pc_dir%/pkgconfig}/cmake/opencv4")
    fi
  fi

  local d
  for d in "${candidates[@]}"; do
    if [[ -n "$d" && -f "$d/OpenCVConfig.cmake" ]]; then
      echo "$d"
      return 0
    fi
  done

  local found
  found="$(find /usr /usr/local -type f -path '*/cmake/opencv4/OpenCVConfig.cmake' 2>/dev/null | head -n 1 || true)"
  if [[ -n "$found" ]]; then
    dirname "$found"
    return 0
  fi
  return 1
}

# <修改 version1 增加PCL路径检测>
detect_pcl_dir() {
  if [[ -f "/usr/lib/aarch64-linux-gnu/cmake/pcl/PCLConfig.cmake" ]]; then
    echo "/usr/lib/aarch64-linux-gnu/cmake/pcl"
    return 0
  fi
  if [[ -f "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake" ]]; then
    echo "/usr/lib/x86_64-linux-gnu/cmake/pcl"
    return 0
  fi
  return 1
}
PCL_DIR="${PCL_DIR:-$(detect_pcl_dir || true)}"

# <修改 version1 增加VTK路径检测>
detect_vtk_dir() {
  local vtk_config
  vtk_config="$(find /usr/lib -name vtk-config.cmake -path '*/cmake/*' 2>/dev/null | head -n 1 || true)"
  if [[ -n "$vtk_config" ]]; then
    dirname "$vtk_config"
    return 0
  fi
  return 1
}
VTK_DIR="${VTK_DIR:-$(detect_vtk_dir || true)}"

detect_torch_root() {
  local candidates=()
  local py root

  candidates+=(
    "/home/dog/.local/lib/python3.10/site-packages/torch"
    "/home/dog/.conda/envs/py310/lib/python3.10/site-packages/torch"
    "/home/dog/miniforge3/envs/py310/lib/python3.10/site-packages/torch"
  )

  for py in /usr/bin/python3.10 /usr/bin/python3 /home/dog/.conda/envs/py310/bin/python /home/dog/miniforge3/envs/py310/bin/python; do
    if [[ -x "$py" ]]; then
      root="$("$py" - <<'PY' 2>/dev/null || true
import os
try:
    import torch
    print(os.path.dirname(torch.__file__))
except Exception:
    pass
PY
)"
      if [[ -n "$root" ]]; then
        candidates+=("$root")
      fi
    fi
  done

  local d
  for d in "${candidates[@]}"; do
    if [[ -n "$d" && -f "$d/share/cmake/Torch/TorchConfig.cmake" ]]; then
      echo "$d"
      return 0
    fi
  done
  return 1
}

detect_cuda_compiler() {
  local candidates=(
    "${CUDACXX:-}"
    "/usr/local/cuda/bin/nvcc"
    "/usr/local/cuda-12.6/bin/nvcc"
  )
  local found
  found="$(command -v nvcc 2>/dev/null || true)"
  if [[ -n "$found" ]]; then
    candidates+=("$found")
  fi

  local nvcc
  for nvcc in "${candidates[@]}"; do
    if [[ -n "$nvcc" && -x "$nvcc" ]]; then
      echo "$nvcc"
      return 0
    fi
  done
  return 1
}

if [[ -z "$RTABMAP_OPENCV_DIR" ]]; then
  RTABMAP_OPENCV_DIR="$(detect_opencv_dir || true)"
fi

if [[ -z "$TORCH_ROOT" ]]; then
  TORCH_ROOT="$(detect_torch_root || true)"
fi
CUDA_COMPILER="${CMAKE_CUDA_COMPILER:-$(detect_cuda_compiler || true)}"

if [[ ! -d "$RTABMAP_SRC_DIR" ]]; then
  echo "[ERROR] RTABMap source dir not found: $RTABMAP_SRC_DIR" >&2
  exit 1
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]]; then
  echo "[ERROR] ROS distro not found: /opt/ros/${ROS_DISTRO_NAME}/setup.bash" >&2
  exit 2
fi

if [[ -z "$RTABMAP_OPENCV_DIR" || ! -f "$RTABMAP_OPENCV_DIR/OpenCVConfig.cmake" ]]; then
  echo "[ERROR] Invalid OpenCV dir: ${RTABMAP_OPENCV_DIR:-<empty>}" >&2
  exit 3
fi

# <修改 version1 增加PCL路径验证>
if [[ -z "$PCL_DIR" || ! -f "$PCL_DIR/PCLConfig.cmake" ]]; then
  echo "[ERROR] PCLConfig.cmake not found. Install libpcl-dev or set PCL_DIR manually." >&2
  exit 4
fi

# <修改 version1 增加VTK路径验证>
if [[ -z "$VTK_DIR" || ! -f "$VTK_DIR/vtk-config.cmake" ]]; then
  echo "[ERROR] vtk-config.cmake not found. Install libvtk9-dev or set VTK_DIR manually." >&2
  exit 5
fi

WITH_TORCH_VALUE="OFF"
RTABMAP_RPATH="$RTABMAP_INSTALL_DIR/lib"
CMAKE_PREFIX_VALUE="${CMAKE_PREFIX_PATH:-}"
if [[ -n "$TORCH_ROOT" && -f "$TORCH_ROOT/share/cmake/Torch/TorchConfig.cmake" ]]; then
  WITH_TORCH_VALUE="ON"
  RTABMAP_RPATH="$RTABMAP_RPATH;$TORCH_ROOT/lib"
  CMAKE_PREFIX_VALUE="$TORCH_ROOT/share/cmake/Torch;$TORCH_ROOT${CMAKE_PREFIX_VALUE:+;$CMAKE_PREFIX_VALUE}"
  export LD_LIBRARY_PATH="$TORCH_ROOT/lib:${LD_LIBRARY_PATH:-}"
  if [[ -n "$CUDA_COMPILER" ]]; then
    export CUDACXX="$CUDA_COMPILER"
    export PATH="$(dirname "$CUDA_COMPILER"):$PATH"
  elif [[ -f "$TORCH_ROOT/lib/libtorch_cuda.so" ]]; then
    echo "[WARN] CUDA Torch found but nvcc is unavailable. Building RTABMap without Torch/SuperPoint support." >&2
    WITH_TORCH_VALUE="OFF"
  fi
else
  echo "[WARN] Torch CMake config not found. Building RTABMap without Torch/SuperPoint support." >&2
fi

mkdir -p "$RTABMAP_BUILD_DIR" "$RTABMAP_INSTALL_DIR"

if [[ -f "$RTABMAP_BUILD_DIR/CMakeCache.txt" ]]; then
  cmake -S "$RTABMAP_SRC_DIR" -B "$RTABMAP_BUILD_DIR" \
    -U '*Torch*' \
    -U '*TORCH*' \
    -U '*Caffe2*' \
    -U '*CAFFE2*' \
    -U '*C10*' \
    -U '*c10*' \
    -U '*CAFFE2*' >/dev/null
fi

echo "[INFO] RTABMap src    : $RTABMAP_SRC_DIR"
echo "[INFO] RTABMap build  : $RTABMAP_BUILD_DIR"
echo "[INFO] RTABMap install: $RTABMAP_INSTALL_DIR"
echo "[INFO] OpenCV_DIR     : $RTABMAP_OPENCV_DIR"
echo "[INFO] PCL_DIR        : $PCL_DIR"
echo "[INFO] VTK_DIR        : $VTK_DIR"
echo "[INFO] Torch root     : ${TORCH_ROOT:-<disabled>}"
echo "[INFO] CUDA compiler  : ${CUDA_COMPILER:-<not found>}"
echo "[INFO] Jobs           : $JOBS"
echo "[INFO] Build type     : $BUILD_TYPE"

set +u
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
set -u

cmake_args=(
  -S "$RTABMAP_SRC_DIR"
  -B "$RTABMAP_BUILD_DIR"
  -DCMAKE_C_COMPILER="${CC:-/usr/bin/gcc}"
  -DCMAKE_CXX_COMPILER="${CXX:-/usr/bin/g++}"
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
  -DCMAKE_INSTALL_PREFIX="$RTABMAP_INSTALL_DIR" \
  -DOpenCV_DIR="$RTABMAP_OPENCV_DIR" \
  -DPCL_DIR="$PCL_DIR" \
  -DVTK_DIR="$VTK_DIR" \
  -DBUILD_APP=OFF \
  -DBUILD_TOOLS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DWITH_PDAL=OFF \
  -DWITH_LIBLAS=OFF \
  -DWITH_CUDASIFT=OFF \
  -DWITH_FREENECT=OFF \
  -DWITH_FREENECT2=OFF \
  -DWITH_K4W2=OFF \
  -DWITH_K4A=OFF \
  -DWITH_OPENNI=OFF \
  -DWITH_OPENNI2=OFF \
  -DWITH_DC1394=OFF \
  -DWITH_CVSBA=OFF \
  -DWITH_CCCORELIB=OFF \
  -DWITH_OPEN3D=OFF \
  -DWITH_LOAM=OFF \
  -DWITH_FLOAM=OFF \
  -DWITH_GRIDMAP=OFF \
  -DWITH_CPUTSDF=OFF \
  -DWITH_OPENCHISEL=OFF \
  -DWITH_ALICE_VISION=OFF \
  -DWITH_FOVIS=OFF \
  -DWITH_VISO2=OFF \
  -DWITH_DVO=OFF \
  -DWITH_ORB_SLAM=OFF \
  -DWITH_OKVIS=OFF \
  -DWITH_MSCKF_VIO=OFF \
  -DWITH_VINS_FUSION=OFF \
  -DWITH_OPENVINS=OFF \
  -DWITH_CUVSLAM=OFF \
  -DWITH_REALSENSE=OFF \
  -DWITH_REALSENSE_SLAM=OFF \
  -DWITH_REALSENSE2=OFF \
  -DWITH_MYNTEYE=OFF \
  -DWITH_DEPTHAI=OFF \
  -DWITH_XVSDK=OFF \
  -DWITH_ORBBEC_SDK=OFF \
  -DWITH_TORCH="$WITH_TORCH_VALUE" \
  -DWITH_PYTHON=ON \
  -DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_VALUE" \
  -DCMAKE_INSTALL_RPATH="$RTABMAP_RPATH"
)

if [[ "$WITH_TORCH_VALUE" == "ON" ]]; then
  cmake_args+=(-DTorch_DIR="$TORCH_ROOT/share/cmake/Torch")
  if [[ -n "$CUDA_COMPILER" ]]; then
    cmake_args+=(-DCMAKE_CUDA_COMPILER="$CUDA_COMPILER")
  fi
fi

cmake "${cmake_args[@]}"

cmake --build "$RTABMAP_BUILD_DIR" --parallel "$JOBS"
cmake --install "$RTABMAP_BUILD_DIR"

echo "[DONE] RTABMap 0.23.4 installed to $RTABMAP_INSTALL_DIR"
