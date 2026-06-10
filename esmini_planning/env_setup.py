"""Planning 运行环境：库路径、PDV 懒加载（避免 mcap 依赖）。"""

from __future__ import annotations

import importlib.util
import os
import sys
import types
from typing import Any, Tuple

_MODULE_DIR = os.path.dirname(os.path.abspath(__file__))
_MYGIT_ROOT = os.path.dirname(_MODULE_DIR)

PLANNING_RELEASE_ROOT = os.path.join(
    _MYGIT_ROOT, "Controllers", "release_x86-planning"
)
PDV_PYTHON_ROOT = os.path.join(PLANNING_RELEASE_ROOT, "python")
PLANNING_LIB = os.path.join(PLANNING_RELEASE_ROOT, "lib", "libplanning_base.so")
PLANNING_LOG_DIR = os.path.join(_MODULE_DIR, "logs")
PLANNING_NATIVE_LOG_DIR = os.path.join(PLANNING_RELEASE_ROOT, "log", "local_planner")

# libpdv_planning 为 Python 3.10 编译；与 esmini 闭环的 sim_eval(3.11) 分开
PREFERRED_PYTHON = os.environ.get(
    "PLANNING_PYTHON",
    "/usr/bin/python3.10" if os.path.isfile("/usr/bin/python3.10") else sys.executable,
)


def apply_runtime_env() -> None:
    """设置 planning .so 运行时环境（可在 import libpdv_planning 前调用）。"""
    lib_dir = os.path.join(PLANNING_RELEASE_ROOT, "lib")
    thirdparty = os.path.join(PLANNING_RELEASE_ROOT, "thirdparty", "lib")
    ld = os.environ.get("LD_LIBRARY_PATH", "")
    for d in (lib_dir, thirdparty):
        if d not in ld.split(":"):
            ld = d + (":" + ld if ld else "")
    os.environ["LD_LIBRARY_PATH"] = ld

    os.environ.setdefault(
        "AD_CORE_BIN_PATH", os.path.join(PLANNING_RELEASE_ROOT, "bin")
    )
    os.environ.setdefault("XS_UGV_MODEL", "GL-T6")
    os.environ.setdefault("XS_UGV_SCENE", "Normal")
    os.makedirs(PLANNING_LOG_DIR, exist_ok=True)
    os.makedirs(PLANNING_NATIVE_LOG_DIR, exist_ok=True)
    os.environ.setdefault("XS_LOG_PATH", os.path.join(PLANNING_RELEASE_ROOT, "log"))

    if PDV_PYTHON_ROOT not in sys.path:
        sys.path.insert(0, PDV_PYTHON_ROOT)
    if lib_dir not in sys.path:
        sys.path.insert(0, lib_dir)


def _load_module(mod_name: str, rel_path: str) -> Any:
    path = os.path.join(PDV_PYTHON_ROOT, rel_path)
    spec = importlib.util.spec_from_file_location(mod_name, path)
    if spec is None or spec.loader is None:
        raise ImportError("无法加载模块: {}".format(path))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[mod_name] = mod
    spec.loader.exec_module(mod)
    return mod


def load_pdv_planning() -> Tuple[Any, Any, Any, Any]:
    """
    懒加载 PDV 子模块，不触发 pdv/__init__.py 对 mcap 的依赖。

    Returns:
        (PlanningInput class, PlanningOutput class, PlanningWrapper class, create_test_input fn)
    """
    apply_runtime_env()
    for name in ("pdv", "pdv.core", "pdv.data", "pdv.planning"):
        sys.modules.setdefault(name, types.ModuleType(name))

    _load_module("pdv.core.exceptions", "pdv/core/exceptions.py")
    planning_input_mod = _load_module(
        "pdv.data.planning_input", "pdv/data/planning_input.py"
    )
    planning_output_mod = _load_module(
        "pdv.data.planning_output", "pdv/data/planning_output.py"
    )
    wrapper_mod = _load_module("pdv.planning.wrapper", "pdv/planning/wrapper.py")

    return (
        planning_input_mod.PlanningInput,
        planning_output_mod.PlanningOutput,
        wrapper_mod.PlanningWrapper,
        wrapper_mod.create_test_input,
    )
