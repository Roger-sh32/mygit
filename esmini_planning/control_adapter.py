"""PlanningOutput → esmini UDPDriver 控制量。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, List, Optional, Tuple

from planning_output_util import effective_trajectory


@dataclass
class DriveCommand:
    throttle: float = 0.0
    brake: float = 0.0
    steering_angle: float = 0.0
    use_state_xyh: bool = False
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0
    speed_ms: float = 0.0
    source: str = "fallback"


def _pt_field(pt: Any, name: str, default: float = 0.0) -> float:
    if isinstance(pt, dict):
        return float(pt.get(name, default))
    return float(getattr(pt, name, default))


class PlanningControlAdapter:
    """
    planning 轨迹 → driverInput；bootstrap 阶段由 controller 发 stateXYH。

    制动：轨迹 v/a 跟踪 + TTC 兜底；max_decel 对齐 GL-T6 / esmini MaxDeceleration=8 m/s²。
    """

    def __init__(
        self,
        cruise_speed_ms: float = 20.0,
        cruise_throttle: float = 0.18,
        max_brake: float = 1.0,
        max_decel_ms2: float = 8.0,
        lookahead_s: float = 1.5,
        horizon_s: float = 5.0,
        ttc_brake_threshold_s: float = 2.0,
        brake_ramp_up: float = 0.4,
        brake_ramp_down: float = 0.1,
    ):
        self.cruise_speed_ms = cruise_speed_ms
        self.cruise_throttle = cruise_throttle
        self.max_brake = max_brake
        self.max_decel_ms2 = max(0.5, max_decel_ms2)
        self.lookahead_s = lookahead_s
        self.horizon_s = horizon_s
        self.ttc_brake_threshold_s = ttc_brake_threshold_s
        self.brake_ramp_up = brake_ramp_up
        self.brake_ramp_down = brake_ramp_down
        self._last_brake = 0.0

    def from_planning_output(
        self,
        output: Any,
        ego_x: float,
        ego_y: float,
        ego_heading: float,
        ego_speed: float,
        ttc: Optional[float] = None,
    ) -> DriveCommand:
        traj: List[Any] = effective_trajectory(output)
        if not traj:
            err = getattr(output, "error_msg", "") if output else "no output"
            return self._fallback(ego_speed, ttc=ttc, reason=err or "empty trajectory")

        target_v, target_a, target_heading, min_horizon_v = self._plan_speed_profile(traj)
        target_v = min(max(0.0, target_v), self.cruise_speed_ms)
        steer = self._heading_error_to_steer(ego_heading, target_heading)

        speed_gap = ego_speed - target_v
        need_brake = (
            speed_gap > 0.3
            or min_horizon_v < ego_speed - 0.5
            or target_a < -0.5
            or target_v < 1.0
            or (ttc is not None and ttc < self.ttc_brake_threshold_s)
        )

        if need_brake:
            brake, brake_mode = self._compute_brake(
                ego_speed, speed_gap, target_v, target_a, ttc
            )
            self._last_brake = brake
            ttc_s = (
                "{:.2f}".format(ttc)
                if ttc is not None and ttc < 900
                else "inf"
            )
            cmd = DriveCommand(
                throttle=0.0,
                brake=brake,
                steering_angle=steer,
                speed_ms=target_v,
                source=(
                    "brake:{} v={:.1f} tgt={:.1f} a={:.1f} min_v={:.1f} "
                    "b={:.2f} ttc={}"
                ).format(
                    brake_mode,
                    ego_speed,
                    target_v,
                    target_a,
                    min_horizon_v,
                    brake,
                    ttc_s,
                ),
            )
        else:
            self._last_brake = max(0.0, self._last_brake - self.brake_ramp_down)
            if ego_speed < self.cruise_speed_ms - 1.0:
                throttle = self.cruise_throttle
            else:
                throttle = 0.0
            cmd = DriveCommand(
                throttle=throttle,
                brake=0.0,
                steering_angle=steer,
                speed_ms=target_v,
                source="cruise v={:.1f} tgt={:.1f}".format(ego_speed, target_v),
            )

        return cmd

    def bootstrap_command(
        self,
        x: float,
        y: float,
        heading: float,
        speed_ms: float,
    ) -> DriveCommand:
        self._last_brake = 0.0
        return DriveCommand(
            use_state_xyh=True,
            x=x,
            y=y,
            heading=heading,
            speed_ms=speed_ms,
            source="bootstrap",
        )

    def _compute_brake(
        self,
        ego_speed: float,
        speed_gap: float,
        target_v: float,
        target_a: float,
        ttc: Optional[float],
    ) -> Tuple[float, str]:
        """返回 (brake 0~1, mode)。"""
        traj_brake = 0.0
        if speed_gap > 0.0:
            # 速度误差 → 所需减速度占 max_decel 的比例
            traj_brake = max(
                traj_brake,
                min(1.0, speed_gap / max(ego_speed, 1.0) * 1.2),
                min(1.0, speed_gap / self.max_decel_ms2 * 0.85),
            )
        if target_a < -0.3:
            traj_brake = max(traj_brake, min(1.0, (-target_a) / self.max_decel_ms2))
        if target_v < 1.0 and ego_speed > 1.5:
            traj_brake = max(traj_brake, min(1.0, 0.55 + speed_gap * 0.06))

        ttc_brake = 0.0
        if ttc is not None and ttc < self.ttc_brake_threshold_s:
            if ttc <= 0.0:
                ttc_brake = self.max_brake
            elif ttc <= 0.5:
                ttc_brake = self.max_brake
            elif ttc <= 1.0:
                ttc_brake = 0.85
            else:
                frac = (self.ttc_brake_threshold_s - ttc) / self.ttc_brake_threshold_s
                ttc_brake = 0.45 + frac * 0.55

        raw = max(traj_brake, ttc_brake)
        emergency = (
            (ttc is not None and ttc < 1.2)
            or speed_gap > 4.0
            or (target_v < 0.5 and ego_speed > 2.0)
        )

        if emergency:
            brake = min(self.max_brake, raw)
            mode = "emergency"
        else:
            brake = min(self.max_brake, raw)
            brake = min(brake, self._last_brake + self.brake_ramp_up)
            brake = max(brake, self._last_brake - self.brake_ramp_down)
            mode = "traj" if traj_brake >= ttc_brake else "ttc"

        return brake, mode

    def _plan_speed_profile(
        self,
        traj: List[Any],
    ) -> Tuple[float, float, float, float]:
        rows = self._normalize_rows(traj)
        if not rows:
            return self.cruise_speed_ms, 0.0, 0.0, self.cruise_speed_ms

        min_horizon_v = rows[0][1]
        min_horizon_a = rows[0][2]
        for t, v, a, _h in rows:
            if t <= self.horizon_s:
                min_horizon_v = min(min_horizon_v, v)
                if a < min_horizon_a:
                    min_horizon_a = a

        la_v, la_a, la_h = self._interp_at(rows, self.lookahead_s)
        near_v, near_a, near_h = self._interp_at(rows, 0.5)
        target_v = min(la_v, min_horizon_v, near_v, self.cruise_speed_ms)
        target_a = min(la_a, min_horizon_a, near_a)
        return target_v, target_a, la_h if la_h else near_h, min_horizon_v

    def _normalize_rows(
        self,
        traj: List[Any],
    ) -> List[Tuple[float, float, float, float]]:
        rows: List[Tuple[float, float, float, float]] = []
        for i, pt in enumerate(traj):
            rows.append(
                (
                    _pt_field(pt, "t", i * 0.1),
                    _pt_field(pt, "v", 0.0),
                    _pt_field(pt, "a", 0.0),
                    _pt_field(pt, "heading", 0.0),
                )
            )
        rows.sort(key=lambda r: r[0])
        return rows

    @staticmethod
    def _interp_at(
        rows: List[Tuple[float, float, float, float]],
        t_query: float,
    ) -> Tuple[float, float, float]:
        if t_query <= rows[0][0]:
            return rows[0][1], rows[0][2], rows[0][3]
        for i in range(len(rows) - 1):
            t0, v0, a0, h0 = rows[i]
            t1, v1, a1, h1 = rows[i + 1]
            if t0 <= t_query <= t1:
                if t1 <= t0:
                    return v1, a1, h1
                r = (t_query - t0) / (t1 - t0)
                return (
                    v0 + r * (v1 - v0),
                    a0 + r * (a1 - a0),
                    h0 + r * (h1 - h0),
                )
        return rows[-1][1], rows[-1][2], rows[-1][3]

    def _fallback(
        self,
        ego_speed: float,
        ttc: Optional[float] = None,
        reason: str = "",
    ) -> DriveCommand:
        if ttc is not None and ttc < self.ttc_brake_threshold_s:
            brake, mode = self._compute_brake(
                ego_speed, max(0.0, ego_speed), 0.0, 0.0, ttc
            )
            self._last_brake = brake
            return DriveCommand(
                throttle=0.0,
                brake=brake,
                steering_angle=0.0,
                source="fallback_{}:{}".format(mode, reason[:32]),
            )
        if ego_speed < self.cruise_speed_ms - 0.5:
            return DriveCommand(
                throttle=self.cruise_throttle,
                brake=0.0,
                steering_angle=0.0,
                source="fallback_accel:{}".format(reason[:40]),
            )
        return DriveCommand(
            throttle=0.0,
            brake=0.0,
            steering_angle=0.0,
            source="fallback:{}".format(reason[:40]),
        )

    @staticmethod
    def _heading_error_to_steer(ego_heading: float, target_heading: float) -> float:
        err = target_heading - ego_heading
        while err > math.pi:
            err -= 2 * math.pi
        while err < -math.pi:
            err += 2 * math.pi
        return max(-0.5, min(0.5, err))
