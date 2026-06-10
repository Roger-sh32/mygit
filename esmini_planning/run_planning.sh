#!/usr/bin/env bash
# esmini + Planning 一键跑（Python 3.10 + planning .so）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MYGIT_ROOT="$(dirname "$SCRIPT_DIR")"
PLAN_ROOT="$MYGIT_ROOT/Controllers/release_x86-planning"

export LD_LIBRARY_PATH="$PLAN_ROOT/lib:$PLAN_ROOT/thirdparty/lib:${LD_LIBRARY_PATH:-}"
export AD_CORE_BIN_PATH="$PLAN_ROOT/bin"
export XS_UGV_MODEL="${XS_UGV_MODEL:-GL-T6}"
export XS_UGV_SCENE="${XS_UGV_SCENE:-Normal}"
export XS_LOG_PATH="$PLAN_ROOT/log"
mkdir -p "$XS_LOG_PATH/local_planner" "$SCRIPT_DIR/logs"

echo "Planning C++ 日志: $PLAN_ROOT/log/local_planner/"
echo "Python 会话日志: $SCRIPT_DIR/logs/"
echo "---"

PYTHON="${PLANNING_PYTHON:-/usr/bin/python3.10}"
if [ ! -x "$PYTHON" ]; then
  echo "错误: 未找到 Python 3.10 ($PYTHON)，planning 的 libpdv_planning 需要 3.10" >&2
  exit 1
fi

exec "$PYTHON" "$SCRIPT_DIR/esmini_run.py" "$@"
