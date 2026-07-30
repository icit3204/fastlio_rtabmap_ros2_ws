#!/usr/bin/env bash
# Underground Map Editor — 一键启动脚本
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$HOME/venvs/underground_map_editor"

# 虚拟环境不存在则自动创建
if [ ! -f "$VENV_DIR/bin/python3" ]; then
    echo "[run.sh] 创建虚拟环境: $VENV_DIR"
    python3 -m venv "$VENV_DIR"
    source "$VENV_DIR/bin/activate"
    pip install --upgrade pip
    pip install -r "$SCRIPT_DIR/requirements.txt"
else
    source "$VENV_DIR/bin/activate"
fi

cd "$SCRIPT_DIR"
echo "[run.sh] 启动 Underground Map Editor..."
python3 main.py
