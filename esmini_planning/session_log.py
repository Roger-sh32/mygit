"""Python 侧会话日志（写入 esmini_planning/logs/）。"""

from __future__ import annotations

import os
from datetime import datetime
from typing import Optional, TextIO

from env_setup import PLANNING_LOG_DIR, PLANNING_RELEASE_ROOT


def native_cpp_log_dir() -> str:
    """Planning C++ 实际写盘目录（与 run_local_planner.sh 一致）。"""
    return os.path.join(PLANNING_RELEASE_ROOT, "log", "local_planner")


class SessionLog:
    def __init__(self, prefix: str = "planning_session"):
        os.makedirs(PLANNING_LOG_DIR, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(PLANNING_LOG_DIR, "{}_{}.log".format(prefix, stamp))
        self._fp: Optional[TextIO] = open(self.path, "w", encoding="utf-8")
        self.line("=== esmini_planning session {} ===".format(stamp))
        self.line("Python 会话日志: {}".format(self.path))
        self.line("Planning C++ 日志目录: {}".format(native_cpp_log_dir()))
        self.line(
            "说明: C++ glog 主要写入 release_x86-planning/log/local_planner/，"
            "不是 esmini_planning/logs/ 下的空目录。"
        )

    def line(self, msg: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        text = "[{}] {}\n".format(ts, msg)
        if self._fp:
            self._fp.write(text)
            self._fp.flush()
        print(msg, flush=True)

    def close(self) -> None:
        if self._fp:
            self._fp.close()
            self._fp = None
