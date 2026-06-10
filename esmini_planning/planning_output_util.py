"""从 PlanningWrapper 结果提取轨迹（estop 时 success 可能为 False 但仍有轨迹）。"""

from __future__ import annotations

from typing import Any, List


def effective_trajectory(output: Any) -> List[Any]:
    if output is None:
        return []
    traj = getattr(output, "trajectory", None) or []
    if traj:
        return list(traj)
    raw = getattr(output, "raw_result", None) or {}
    data = raw.get("trajectory") or []
    if not data:
        return []
    try:
        from pdv.data.planning_output import TrajectoryPoint

        return [TrajectoryPoint.from_dict(p) for p in data]
    except Exception:
        return data


def trajectory_point_count(output: Any) -> int:
    traj = effective_trajectory(output)
    if traj:
        return len(traj)
    raw = getattr(output, "raw_result", None) or {}
    return int(raw.get("trajectory_size", 0) or 0)
