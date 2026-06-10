"""straight_500m → xsproto.globalpath.TaskList（P4 路由）。"""

from __future__ import annotations

import math
from typing import List, Tuple

from map_builder import StraightRoadSpec
from xsproto_builder import _ensure_xsproto_path


def _import_task_list_proto():
    _ensure_xsproto_path()
    from globalpath.task_list_pb2 import (  # type: ignore
        AvoidAllowBackType,
        AvoidObstacleType,
        Direction,
        FollowType,
        RoadType,
        TaskList,
        TaskPoint,
        TurnInfo,
    )

    return (
        TaskList,
        TaskPoint,
        FollowType,
        TurnInfo,
        RoadType,
        Direction,
        AvoidObstacleType,
        AvoidAllowBackType,
    )


def _nearest_lane_y(ego_y: float, spec: StraightRoadSpec) -> float:
    return min(spec.lane_center_y, key=lambda cy: abs(cy - ego_y))


def _prefer_lane_id(lane_y: float, spec: StraightRoadSpec) -> int:
    """与 map_builder 三车道 y 对齐的 prefer_lane_id（0=不指定）。"""
    if abs(lane_y - spec.lane_center_y[2]) < 0.05:
        return 1
    if abs(lane_y - spec.lane_center_y[1]) < 0.05:
        return 0
    if abs(lane_y - spec.lane_center_y[0]) < 0.05:
        return 2
    return 0


def _centerline_id(lane_y: float, spec: StraightRoadSpec) -> int:
    """与 FovHdMap roadlines id=100+idx 对齐。"""
    for idx, cy in enumerate(spec.lane_center_y):
        if abs(lane_y - cy) < 0.05:
            return 100 + idx
    return 101


def _sample_range(x0: float, x1: float, step_m: float) -> List[float]:
    if x1 <= x0:
        return [float(x0)]
    xs: List[float] = []
    x = x0
    while x <= x1 + 1e-6:
        xs.append(float(x))
        x += step_m
    if xs[-1] < x1 - 1e-3:
        xs.append(float(x1))
    return xs


class StraightRoadTaskListBuilder:
    """沿 straight_500m 直道生成 TaskList，供 VirtualMapBuilder 构建 lane_seq。"""

    def __init__(self, spec: StraightRoadSpec | None = None):
        (
            self.TaskList,
            self.TaskPoint,
            self.FollowType,
            self.TurnInfo,
            self.RoadType,
            self.Direction,
            self.AvoidObstacleType,
            self.AvoidAllowBackType,
        ) = _import_task_list_proto()
        self.spec = spec or StraightRoadSpec()
        self._point_id = 1

    def _make_point(
        self,
        x_m: float,
        y_m: float,
        heading_deg: float,
        max_speed_kmh: float,
        lane_y: float,
        point_type: int = 0,
    ):
        pt = self.TaskPoint()
        pt.id = self._point_id
        self._point_id += 1
        pt.x = x_m * 100.0
        pt.y = y_m * 100.0
        pt.angle = -1.0
        pt.max_speed = max_speed_kmh
        pt.centerline_id = _centerline_id(lane_y, self.spec)
        pt.turn_info = self.TurnInfo.FORWARD_TURN
        pt.follow_type = self.FollowType.LANE
        pt.point_type = point_type
        pt.point_source = 0
        pt.prefer_lane_id = _prefer_lane_id(lane_y, self.spec)
        pt.road_type = self.RoadType.CITY
        pt.stopline_type = 0
        pt.region_type = 0
        pt.direction = self.Direction.FORWARD
        pt.avoid_obstacle_type = self.AvoidObstacleType.AVOID_OBSTACLE
        pt.replan_point = 0
        if heading_deg >= 0:
            pt.angle = heading_deg % 360.0
        return pt

    def build_task_list_bytes(
        self,
        timestamp: float,
        ego_x: float,
        ego_y: float,
        ego_yaw: float,
        seq_id: int,
        max_speed_kmh: float = 72.0,
        forward_m: float = 350.0,
        back_m: float = 50.0,
        step_m: float = 5.0,
    ) -> bytes:
        spec = self.spec
        lane_y = _nearest_lane_y(ego_y, spec)
        heading_deg = math.degrees(float(ego_yaw)) % 360.0

        x_start = max(0.0, float(ego_x))
        x_end = min(spec.length, x_start + forward_m)
        x_pre0 = max(0.0, x_start - back_m)

        msg = self.TaskList()
        hdr = msg.header
        hdr.timestamp = float(timestamp)
        hdr.seq_id = int(seq_id)
        msg.localpose_time = float(timestamp)
        msg.globalpose_time = float(timestamp)
        msg.message_seq_num = int(seq_id)

        self._point_id = 1
        pre_xs = _sample_range(x_pre0, x_start, step_m)
        if len(pre_xs) > 1:
            pre_xs = pre_xs[:-1]
        for x in pre_xs:
            msg.pre_points.append(
                self._make_point(x, lane_y, heading_deg, max_speed_kmh, lane_y)
            )

        future_xs = _sample_range(x_start, x_end, step_m)
        for i, x in enumerate(future_xs):
            point_type = 1 if i == 0 else (2 if i == len(future_xs) - 1 else 0)
            msg.points.append(
                self._make_point(
                    x,
                    lane_y,
                    heading_deg,
                    max_speed_kmh,
                    lane_y,
                    point_type=point_type,
                )
            )

        if not msg.points:
            msg.points.append(
                self._make_point(x_start, lane_y, heading_deg, max_speed_kmh, lane_y, 1)
            )
            msg.points.append(
                self._make_point(
                    min(spec.length, x_start + 10.0),
                    lane_y,
                    heading_deg,
                    max_speed_kmh,
                    lane_y,
                    2,
                )
            )

        goal_x = msg.points[-1].x / 100.0
        goal_y = msg.points[-1].y / 100.0
        msg.last_point_guass.gauss_x = goal_x * 100.0
        msg.last_point_guass.gauss_y = goal_y * 100.0
        msg.last_point_guass.azimuth = heading_deg

        msg.task_distance = max(0.0, x_end - x_start)
        msg.first_turn_info = self.TurnInfo.FORWARD_TURN
        msg.first_turn_distance = max(msg.task_distance, 200.0)
        msg.first_bus_station_distance = 0.0

        try:
            road_id = int(spec.road_id)
        except ValueError:
            road_id = 1
        msg.road_identifiers.append(road_id)

        seg = msg.turn_info_segments.add()
        seg.seg_begin_distance = 0.0
        seg.seg_end_distance = max(msg.task_distance, 1.0)
        seg.seg_turn_info = self.TurnInfo.FORWARD_TURN

        msg.avoid_obstacle_type = self.AvoidObstacleType.AVOID_OBSTACLE
        msg.obstacle_safe_distance = 80.0
        msg.obstacle_avoid_distance = 120.0
        msg.allow_back = self.AvoidAllowBackType.AVOID_NOT_ALLOW_BACK
        msg.task_commond.req_id = 0

        return msg.SerializeToString()

    def point_count_hint(self, ego_x: float, forward_m: float = 350.0, step_m: float = 5.0) -> Tuple[int, int]:
        x_start = max(0.0, float(ego_x))
        x_end = min(self.spec.length, x_start + forward_m)
        n_future = len(_sample_range(x_start, x_end, step_m))
        return n_future, n_future
