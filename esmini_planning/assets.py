"""mygit 资产路径：xmap（路网）、xsscenarios（场景库）。"""

from __future__ import annotations

import os

_MODULE_DIR = os.path.dirname(os.path.abspath(__file__))
MYGIT_ROOT = os.path.dirname(_MODULE_DIR)

XMAP_DIR = os.path.join(MYGIT_ROOT, "xmap")
XSSCENARIOS_DIR = os.path.join(MYGIT_ROOT, "xsscenarios")

ESMINI_DEMO_ROOT = os.path.join(MYGIT_ROOT, "esmini-demo_Linux", "esmini-demo")
ESMINI_BIN = os.path.join(ESMINI_DEMO_ROOT, "bin", "esmini")
ESMINI_CATALOGS = os.path.join(ESMINI_DEMO_ROOT, "resources", "xosc", "Catalogs")

DEFAULT_XODR = os.path.join(XMAP_DIR, "straight_500m.xodr")
DEFAULT_XOSC = os.path.join(XSSCENARIOS_DIR, "AEB-test-planning.xosc")

# Planning 场景库（均为 Ego=UDP + car_gl_t6；路网见各 xosc RoadNetwork）
PLANNING_SCENARIOS = {
    "aeb": ("AEB-test-planning.xosc", "straight_500m.xodr", "AEB 变道+制动序列"),
    "slow-lead": ("slow-lead-vehicle-planning.xosc", "straight_500m_signs.xodr", "前方慢车跟驰"),
    "cut-in": ("cut-in_simple-planning.xosc", "straight_500m.xodr", "邻道切入+急刹"),
    "alks-cut-in": ("alks-cut-in-planning.xosc", "straight_500m.xodr", "对向车道切入+急刹"),
}

DEFAULT_EGO_VEHICLE = "car_gl_t6"
VEHICLE_CATALOG_DIR = os.path.join(XSSCENARIOS_DIR, "Catalogs", "Vehicles")

# 被测对象：正式 Planning 库
PLANNING_RELEASE = os.path.join(MYGIT_ROOT, "Controllers", "release_x86-planning")
