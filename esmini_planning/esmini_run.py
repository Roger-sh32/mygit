"""
esmini + Planning 一键启动（独立于 esmini_operator）。

用法：
  ./run_planning.sh
  python3.10 esmini_planning/esmini_run.py
  python3.10 esmini_planning/esmini_run.py --initial-speed-ms 15
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from typing import List, Optional

_MODULE_DIR = os.path.dirname(os.path.abspath(__file__))
_MYGIT_ROOT = os.path.dirname(_MODULE_DIR)

if _MODULE_DIR not in sys.path:
    sys.path.insert(0, _MODULE_DIR)

from assets import DEFAULT_XOSC, ESMINI_BIN, PLANNING_SCENARIOS, XSSCENARIOS_DIR

ESMINI_DEMO_ROOT = os.path.join(_MYGIT_ROOT, "esmini-demo_Linux", "esmini-demo")
ESMINI_PATH = ESMINI_BIN
XOSC_PATH = DEFAULT_XOSC
CONTROLLER_SCRIPT = os.path.join(_MODULE_DIR, "planning_controller.py")

ESMINI_START_DELAY_SEC = 2.0

# libpdv_planning 绑定 Python 3.10
PREFERRED_PYTHON = os.environ.get(
    "PLANNING_PYTHON",
    "/usr/bin/python3.10" if os.path.isfile("/usr/bin/python3.10") else sys.executable,
)


def _esmini_workdir(esmini: str) -> str:
    demo_root = os.path.dirname(os.path.dirname(esmini))
    if os.path.isfile(os.path.join(demo_root, "config.yml")):
        return demo_root
    return os.path.dirname(esmini) or "."


def _build_esmini_cmd(xosc: str, esmini: str) -> List[str]:
    return [
        esmini,
        "--window",
        "60",
        "60",
        "1024",
        "576",
        "--osc",
        xosc,
        "--osi_receiver_ip",
        "127.0.0.1",
    ]


def _stop_process(proc: subprocess.Popen) -> None:
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            proc.kill()


def _planning_env() -> dict:
    """继承当前环境，并确保 planning .so 路径在子进程中生效。"""
    from env_setup import apply_runtime_env, PLANNING_RELEASE_ROOT

    apply_runtime_env()
    env = os.environ.copy()
    lib = os.path.join(PLANNING_RELEASE_ROOT, "lib")
    third = os.path.join(PLANNING_RELEASE_ROOT, "thirdparty", "lib")
    ld = env.get("LD_LIBRARY_PATH", "")
    for d in (lib, third):
        if d not in ld.split(":"):
            ld = d + (":" + ld if ld else "")
    env["LD_LIBRARY_PATH"] = ld
    return env


def run_planning_loop(
    xosc: str,
    esmini: str,
    controller: str,
    python_exe: str,
    esmini_delay: float,
    max_loops: int,
    initial_speed_ms: float,
    plan_every: int,
    bootstrap_frames: int,
) -> int:
    if not os.path.isfile(esmini):
        print("错误: 找不到 esmini:", esmini, file=sys.stderr)
        return 1
    if not os.path.isfile(controller):
        print("错误: 找不到 planning_controller:", controller, file=sys.stderr)
        return 1
    if not os.path.isfile(python_exe):
        print(
            "错误: 找不到 Python 解释器 {}（planning 需 Python 3.10）".format(python_exe),
            file=sys.stderr,
        )
        return 1

    esmini_cmd = _build_esmini_cmd(xosc, esmini)
    ctrl_cmd = [
        python_exe,
        controller,
        "--max-loops",
        str(max_loops),
        "--plan-every",
        str(plan_every),
        "--bootstrap-frames",
        str(bootstrap_frames),
    ]
    ctrl_cmd.extend(["--initial-speed-ms", str(initial_speed_ms)])

    print("Planning 一键模式（esmini_planning，与 esmini_operator 隔离）")
    print("  esmini:", " ".join(esmini_cmd))
    print("  控制器:", " ".join(ctrl_cmd))
    print("  python:", python_exe)
    print("  等待 esmini:", esmini_delay, "s")
    print("---")

    env = _planning_env()
    esmini_proc = subprocess.Popen(
        esmini_cmd, cwd=_esmini_workdir(esmini), env=env
    )
    rc = 1
    try:
        time.sleep(esmini_delay)
        if esmini_proc.poll() is not None:
            print("错误: esmini 已退出", esmini_proc.returncode, file=sys.stderr)
            return 1
        rc = subprocess.call(ctrl_cmd, env=env, cwd=_MODULE_DIR)
        esmini_rc = esmini_proc.poll()
        if esmini_rc is not None:
            print("esmini 已退出，returncode={}".format(esmini_rc), flush=True)
    finally:
        _stop_process(esmini_proc)
    return rc


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(description="esmini + Planning 一键闭环")
    p.add_argument(
        "--scenario",
        choices=sorted(PLANNING_SCENARIOS.keys()),
        help="场景短名（见 xsscenarios/README.md）",
    )
    p.add_argument("--xosc", default=None, help="OpenSCENARIO 路径（覆盖 --scenario）")
    p.add_argument("--esmini", default=ESMINI_PATH)
    p.add_argument("--controller", default=CONTROLLER_SCRIPT)
    p.add_argument("--python", default=PREFERRED_PYTHON, dest="python_exe")
    p.add_argument("--esmini-delay", type=float, default=ESMINI_START_DELAY_SEC)
    p.add_argument("--max-loops", type=int, default=4000)
    p.add_argument(
        "--initial-speed-ms",
        type=float,
        default=20.0,
        help="bootstrap 初速 m/s（默认 20，AEB 场景比 33.33 更安全）",
    )
    p.add_argument("--plan-every", type=int, default=1)
    p.add_argument("--bootstrap-frames", type=int, default=40)
    args = p.parse_args(argv)

    if args.xosc:
        xosc = args.xosc
    elif args.scenario:
        xosc = os.path.join(XSSCENARIOS_DIR, PLANNING_SCENARIOS[args.scenario][0])
    else:
        xosc = DEFAULT_XOSC

    return run_planning_loop(
        xosc,
        args.esmini,
        args.controller,
        args.python_exe,
        args.esmini_delay,
        args.max_loops,
        args.initial_speed_ms,
        args.plan_every,
        args.bootstrap_frames,
    )


if __name__ == "__main__":
    sys.exit(main())
