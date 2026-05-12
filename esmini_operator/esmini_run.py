"""
esmini 启动器：播放模式（仅仿真）与测试模式（仿真 + 被测算法 + 评测 + HTML 报告）。

用法示例：
  播放：python esmini_run.py
        python esmini_run.py --mode playback --xosc /path/to/scenario.xosc
  测试：python esmini_run.py --mode test
        python esmini_run.py --mode test --algo /path/to/AEB_controller.py
"""

from __future__ import annotations

import argparse
import glob
import html
import json
import os
import re
import subprocess
import sys
import time
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# 默认路径（可按本机仓库位置修改；也可用命令行参数覆盖）
# ---------------------------------------------------------------------------
_OPERATOR_DIR = os.path.dirname(os.path.abspath(__file__))
# esmini_operator 的上级的上级 -> 例如 .../桌面/git
_REPO_ROOT = os.path.dirname(os.path.dirname(_OPERATOR_DIR))

ESMINI_PATH = os.path.join(_REPO_ROOT, "esmini", "bin", "esmini")
FILE_PATH = os.path.join(_REPO_ROOT, "esmini", "resources", "xosc", "AEB-test.xosc")
EVAL_FRAMEWORK_ROOT = os.path.join(_REPO_ROOT, "esmini_eval_framework")
# 被测算法入口（可替换为其它控制器脚本，需兼容当前命令行参数）
DEFAULT_ALGO_SCRIPT = os.path.join(
    EVAL_FRAMEWORK_ROOT, "Algorithm_undertest", "AEB_controller.py"
)

# 播放模式默认是否带 OSI（与原 OSI_INFO_OUTPUT=True 一致；可用 --no-playback-osi 关闭）
PLAYBACK_USE_OSI = True
# 测试模式：等 esmini 就绪后再起算法（秒）
TEST_ESMINI_START_DELAY_SEC = 2.0
# 测试输出子目录（在 esmini_operator 下）
TEST_RUNS_SUBDIR = "test_runs"
REPORTS_SUBDIR = "reports"


def _build_esmini_cmd(
    xosc: str,
    use_osi: bool,
    window_args: Optional[List[str]] = None,
) -> List[str]:
    if window_args is None:
        window_args = ["--window", "60", "60", "1024", "576"]
    cmd = [ESMINI_PATH] + window_args + ["--osc", xosc]
    if use_osi:
        cmd.extend(["--osi_receiver_ip", "127.0.0.1"])
    return cmd


def run_playback(xosc: str, esmini: str, use_osi: bool) -> int:
    """仅启动 esmini 播放场景。"""
    cmd = _build_esmini_cmd(xosc, use_osi)
    cmd[0] = esmini
    print("播放模式:", " ".join(cmd))
    return subprocess.call(cmd)


def _parse_result_json_line(stdout: str) -> Optional[str]:
    """从算法 stdout 解析「已写: xxx_result.json」。"""
    for line in stdout.splitlines():
        line = line.strip()
        if line.startswith("已写:"):
            return line.split("已写:", 1)[1].strip()
    return None


def _fmt_json_cell(v: Any) -> str:
    if v is None:
        return ""
    if isinstance(v, float):
        if v != v:  # NaN
            return "NaN"
        if abs(v) > 1e308:
            return "inf"
        return "{:.6g}".format(v)
    if isinstance(v, (list, dict)):
        return html.escape(json.dumps(v, ensure_ascii=False, indent=2))
    return html.escape(str(v))


def write_result_html(result: Dict[str, Any], html_path: str, meta: Dict[str, str]) -> None:
    """将评测 dict 写成简单单页 HTML。"""
    rows_html = []
    skip_keys = {"fail_reason", "config"}
    for k in sorted(result.keys()):
        if k in skip_keys:
            continue
        rows_html.append(
            "<tr><th>{}</th><td>{}</td></tr>".format(
                html.escape(str(k)),
                _fmt_json_cell(result.get(k)),
            )
        )

    fail_list = result.get("fail_reason") or []
    if not isinstance(fail_list, list):
        fail_list = [str(fail_list)]
    fr_html = "".join(
        "<li>{}</li>".format(html.escape(str(x))) for x in fail_list
    )
    if not fr_html:
        fr_html = "<li>（无）</li>"

    cfg = result.get("config") or {}
    cfg_html = "<pre>{}</pre>".format(
        html.escape(json.dumps(cfg, ensure_ascii=False, indent=2))
    )

    meta_rows = "".join(
        "<tr><th>{}</th><td>{}</td></tr>".format(html.escape(a), html.escape(b))
        for a, b in meta.items()
    )

    final = str(result.get("final_result", ""))
    badge = "PASS" if final == "PASS" else "FAIL"
    badge_class = "pass" if badge == "PASS" else "fail"

    doc = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8"/>
  <title>测试结果 — {title}</title>
  <style>
    body {{ font-family: system-ui, sans-serif; margin: 24px; background: #f5f5f5; }}
    h1 {{ font-size: 1.25rem; }}
    .badge {{ display: inline-block; padding: 6px 14px; border-radius: 6px; font-weight: 600; }}
    .badge.pass {{ background: #c8e6c9; color: #1b5e20; }}
    .badge.fail {{ background: #ffcdd2; color: #b71c1c; }}
    table {{ border-collapse: collapse; background: #fff; margin: 16px 0; min-width: 420px; }}
    th, td {{ border: 1px solid #ddd; padding: 8px 12px; text-align: left; vertical-align: top; }}
    th {{ background: #eee; width: 220px; }}
    pre {{ background: #fff; border: 1px solid #ddd; padding: 12px; overflow: auto; }}
    ul {{ background: #fff; border: 1px solid #ddd; padding: 12px 12px 12px 28px; }}
  </style>
</head>
<body>
  <h1>测试结果 <span class="badge {badge_class}">{badge}</span></h1>
  <p>生成时间：{time}</p>
  <h2>运行信息</h2>
  <table>{meta_rows}</table>
  <h2>指标</h2>
  <table>{rows}</table>
  <h2>fail_reason</h2>
  <ul>{fr}</ul>
  <h2>评测配置 config</h2>
  {cfg}
</body>
</html>""".format(
        title=html.escape(meta.get("case", "run")),
        badge_class=badge_class,
        badge=badge,
        time=html.escape(meta.get("generated_at", "")),
        meta_rows=meta_rows,
        rows="".join(rows_html),
        fr=fr_html,
        cfg=cfg_html,
    )

    os.makedirs(os.path.dirname(html_path) or ".", exist_ok=True)
    with open(html_path, "w", encoding="utf-8") as fp:
        fp.write(doc)


def run_test(
    xosc: str,
    esmini: str,
    algo_script: str,
    esmini_delay: float,
    max_loops: int,
) -> int:
    """
    先启动 esmini（带 OSI），再运行被测算法；结束后结束仿真进程并生成 HTML。
    """
    if not os.path.isfile(algo_script):
        print("错误: 找不到被测算法脚本:", algo_script, file=sys.stderr)
        return 1
    if not os.path.isfile(esmini):
        print("错误: 找不到 esmini 可执行文件:", esmini, file=sys.stderr)
        return 1

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = os.path.join(_OPERATOR_DIR, TEST_RUNS_SUBDIR, stamp)
    os.makedirs(run_dir, exist_ok=True)
    case_name = "operator_run"

    esmini_cmd = _build_esmini_cmd(xosc, use_osi=True)
    esmini_cmd[0] = esmini

    print("测试模式")
    print("  esmini:", " ".join(esmini_cmd))
    print("  算法:", sys.executable, algo_script, "...")
    print("  日志目录:", run_dir)

    esmini_proc = subprocess.Popen(
        esmini_cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
    )
    rc = 1
    result_json_path: Optional[str] = None
    try:
        time.sleep(esmini_delay)
        if esmini_proc.poll() is not None:
            print("错误: esmini 已退出，退出码", esmini_proc.returncode, file=sys.stderr)
            return 1

        algo_cmd = [
            sys.executable,
            algo_script,
            "--log-dir",
            run_dir,
            "--case-name",
            case_name,
            "--max-loops",
            str(max_loops),
        ]
        proc = subprocess.run(
            algo_cmd,
            capture_output=True,
            text=True,
            timeout=None,
        )
        out = (proc.stdout or "") + "\n" + (proc.stderr or "")
        print(proc.stdout or "", end="")
        if proc.stderr:
            print(proc.stderr, end="", file=sys.stderr)

        if proc.returncode != 0:
            print("算法进程退出码:", proc.returncode, file=sys.stderr)
            rc = proc.returncode
        else:
            rc = 0

        result_json_path = _parse_result_json_line((proc.stdout or "") + "\n" + (proc.stderr or ""))
        if not result_json_path or not os.path.isfile(result_json_path):
            pattern = os.path.join(run_dir, "{}_*_frame_log_result.json".format(case_name))
            cand = sorted(glob.glob(pattern), key=os.path.getmtime, reverse=True)
            if cand:
                result_json_path = cand[0]

        if result_json_path and os.path.isfile(result_json_path):
            reports_dir = os.path.join(_OPERATOR_DIR, REPORTS_SUBDIR)
            os.makedirs(reports_dir, exist_ok=True)
            html_name = os.path.splitext(os.path.basename(result_json_path))[0] + ".html"
            html_path = os.path.join(reports_dir, html_name)
            with open(result_json_path, "r", encoding="utf-8") as fp:
                result = json.load(fp)
            meta = {
                "case": case_name,
                "xosc": xosc,
                "algo": algo_script,
                "result_json": result_json_path,
                "generated_at": datetime.now().isoformat(timespec="seconds"),
            }
            write_result_html(result, html_path, meta)
            print("HTML 报告:", html_path)
        else:
            print("警告: 未找到评测结果 JSON，跳过 HTML 生成", file=sys.stderr)
            if result_json_path:
                print("  解析路径:", result_json_path, file=sys.stderr)

    finally:
        if esmini_proc.poll() is None:
            esmini_proc.terminate()
            try:
                esmini_proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                esmini_proc.kill()

    return rc


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(description="esmini 播放 / 测试（仿真+算法+评测+HTML）")
    p.add_argument(
        "--mode",
        choices=("playback", "test"),
        default="playback",
        help="playback=仅播放 xosc；test=起仿真与被测算法并生成评测与 HTML",
    )
    p.add_argument("--xosc", default=FILE_PATH, help="OpenSCENARIO 文件路径")
    p.add_argument("--esmini", default=ESMINI_PATH, help="esmini 可执行文件")
    p.add_argument(
        "--algo",
        default=DEFAULT_ALGO_SCRIPT,
        help="测试模式下运行的被测算法脚本（默认 AEB_controller.py，可替换）",
    )
    p.add_argument(
        "--no-playback-osi",
        action="store_true",
        help="播放模式下关闭 OSI（默认开启，与原先脚本一致）",
    )
    p.add_argument(
        "--esmini-delay",
        type=float,
        default=TEST_ESMINI_START_DELAY_SEC,
        help="测试模式下等待 esmini 就绪的秒数",
    )
    p.add_argument(
        "--max-loops",
        type=int,
        default=4000,
        help="测试模式传给算法的最大主循环步数",
    )
    args = p.parse_args(argv)

    if args.mode == "playback":
        return run_playback(
            args.xosc,
            args.esmini,
            use_osi=PLAYBACK_USE_OSI and not args.no_playback_osi,
        )
    return run_test(
        args.xosc,
        args.esmini,
        args.algo,
        args.esmini_delay,
        args.max_loops,
    )


if __name__ == "__main__":
    sys.exit(main())
