"""straight_500m.xodr → xsproto.ldmap.Map / FovHdMap / MapPosition（P2）。"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import List, Tuple

from xsproto_builder import _ensure_xsproto_path


@dataclass(frozen=True)
class StraightRoadSpec:
    """与 AEB-test.xosc 使用的 straight_500m.xodr 对齐的简化几何。"""

    road_id: str = "1"
    section_id: str = "sec_1"
    length: float = 500.0
    lane_width: float = 3.07
    lane_center_y: Tuple[float, float, float] = (1.535, 0.0, -1.535)
    lane_ids: Tuple[str, str, str] = ("lane_1", "lane_0", "lane_-1")


def _import_map_proto():
    _ensure_xsproto_path()
    from common.geometry_pb2 import Point2D  # type: ignore
    from hdmap.fov_hdmap_pb2 import FovHdMap, FovLanemarking, FovRoadline  # type: ignore
    from ldmap.map_lane_pb2 import (  # type: ignore
        DASHED,
        CITY_DRIVING_LANE,
        COLOR_WHITE,
        FORWARD,
        Lane,
        LaneBoundary,
        LineAttribute,
        NO_TURN,
        STRAIGHT_TURN,
    )
    from ldmap.map_pb2 import Map  # type: ignore
    from ldmap.map_road_pb2 import CURB, Road, RoadBoundary, RoadSection, URBAN_ROAD  # type: ignore
    from ldmap.navi_route_pb2 import (  # type: ignore
        LINK_CLASS_REGULAR,
        LINK_DIRECTION_POSITIVE,
        LINK_TYPE_MAIN,
        NaviRoute,
        NaviSection,
        NaviPosition,
    )
    from perception.map_position_pb2 import MapPosition  # type: ignore

    return (
        Point2D,
        FovHdMap,
        FovLanemarking,
        FovRoadline,
        Map,
        Lane,
        LaneBoundary,
        LineAttribute,
        Road,
        RoadSection,
        RoadBoundary,
        NaviRoute,
        NaviSection,
        NaviPosition,
        CITY_DRIVING_LANE,
        FORWARD,
        NO_TURN,
        STRAIGHT_TURN,
        DASHED,
        COLOR_WHITE,
        URBAN_ROAD,
        CURB,
        LINK_CLASS_REGULAR,
        LINK_TYPE_MAIN,
        LINK_DIRECTION_POSITIVE,
        MapPosition,
    )


def _x_window(ego_x: float, spec: StraightRoadSpec, back: float, forward: float) -> Tuple[float, float]:
    x0 = max(0.0, ego_x - back)
    x1 = min(spec.length, ego_x + forward)
    if x1 <= x0:
        x1 = min(spec.length, x0 + 10.0)
    return x0, x1


def _sample_line(x0: float, x1: float, y: float, step: float = 5.0) -> Tuple[List[float], List[float]]:
    if x1 < x0:
        x0, x1 = x1, x0
    xs: List[float] = []
    ys: List[float] = []
    x = x0
    while x <= x1 + 1e-6:
        xs.append(float(x))
        ys.append(float(y))
        x += step
    if len(xs) < 2:
        xs = [float(x0), float(x1)]
        ys = [float(y), float(y)]
    return xs, ys


def _points_from_xy(Point2D, xs: List[float], ys: List[float]):
    return [Point2D(x=x, y=y) for x, y in zip(xs, ys)]


class StraightRoadMapBuilder:
    """沿 OSI 全局坐标生成 straight_500m 局部地图 proto。"""

    def __init__(self, spec: StraightRoadSpec | None = None):
        (
            self.Point2D,
            self.FovHdMap,
            self.FovLanemarking,
            self.FovRoadline,
            self.Map,
            self.Lane,
            self.LaneBoundary,
            self.LineAttribute,
            self.Road,
            self.RoadSection,
            self.RoadBoundary,
            self.NaviRoute,
            self.NaviSection,
            self.NaviPosition,
            self.CITY_DRIVING_LANE,
            self.FORWARD,
            self.NO_TURN,
            self.STRAIGHT_TURN,
            self.DASHED,
            self.COLOR_WHITE,
            self.URBAN_ROAD,
            self.CURB,
            self.LINK_CLASS_REGULAR,
            self.LINK_TYPE_MAIN,
            self.LINK_DIRECTION_POSITIVE,
            self.MapPosition,
        ) = _import_map_proto()
        self.spec = spec or StraightRoadSpec()

    def _lane_boundary(self, boundary_id: str, xs: List[float], ys: List[float]):
        lb = self.LaneBoundary()
        lb.id = boundary_id
        attr = self.LineAttribute()
        attr.start_s = 0.0
        attr.end_s = max(xs[-1] - xs[0], 1.0)
        attr.line_type = self.DASHED
        attr.line_color = self.COLOR_WHITE
        lb.line_attributes.append(attr)
        lb.points.extend(_points_from_xy(self.Point2D, xs, ys))
        return lb

    def build_ld_map_bytes(
        self,
        timestamp: float,
        ego_x: float,
        ego_y: float,
        ego_yaw: float,
        seq_id: int,
        back: float = 40.0,
        forward: float = 180.0,
    ) -> bytes:
        """xsproto.ldmap.Map → ld_map_bytes（PlanningWrapper 期望格式）。"""
        spec = self.spec
        x0, x1 = _x_window(ego_x, spec, back, forward)
        half_w = spec.lane_width * 0.5

        msg = self.Map()
        hdr = msg.header
        hdr.timestamp = float(timestamp)
        hdr.seq_id = int(seq_id)

        boundary_specs = [
            ("lb_outer_left", spec.lane_center_y[0] + half_w),
            ("lb_lane1_lane0", spec.lane_center_y[1]),
            ("lb_lane0_lane-1", spec.lane_center_y[2] + half_w),
            ("lb_outer_right", spec.lane_center_y[2] - half_w),
        ]
        for bid, by in boundary_specs:
            xs, ys = _sample_line(x0, x1, by)
            msg.lane_boundary.append(self._lane_boundary(bid, xs, ys))

        left_edge_id = "rb_left"
        right_edge_id = "rb_right"
        for rid, by in (
            (left_edge_id, boundary_specs[0][1] + half_w),
            (right_edge_id, boundary_specs[3][1] - half_w),
        ):
            xs, ys = _sample_line(x0, x1, by)
            rb = self.RoadBoundary()
            rb.id = rid
            rb.type = self.CURB
            rb.width = 0.15
            rb.points.extend(_points_from_xy(self.Point2D, xs, ys))
            msg.road_boundary.append(rb)

        lane_defs = [
            ("lane_1", spec.lane_center_y[0], "lb_outer_left", "lb_lane1_lane0", None, "lane_0", left_edge_id, None),
            ("lane_0", spec.lane_center_y[1], "lb_lane1_lane0", "lb_lane0_lane-1", "lane_1", "lane_-1", None, None),
            ("lane_-1", spec.lane_center_y[2], "lb_lane0_lane-1", "lb_outer_right", "lane_0", None, None, right_edge_id),
        ]
        for lane_id, cy, left_id, right_id, left_neighbor, right_neighbor, left_rb, right_rb in lane_defs:
            xs, ys = _sample_line(x0, x1, cy)
            lane = self.Lane()
            lane.id = lane_id
            lane.road_id = spec.road_id
            lane.section_id = spec.section_id
            lane.center_line.extend(_points_from_xy(self.Point2D, xs, ys))
            lane.length = max(x1 - x0, 1.0)
            lane.speed_limit = 120.0
            lane.type = self.CITY_DRIVING_LANE
            lane.current_turn = self.STRAIGHT_TURN
            lane.next_turn.append(self.NO_TURN)
            lane.direction = self.FORWARD
            lane.prefer_level = 1
            lane.is_virtual = False
            if left_neighbor:
                lane.left_lane_ids.append(left_neighbor)
            if right_neighbor:
                lane.right_lane_ids.append(right_neighbor)
            lane.left_lane_boundary_ids.append(left_id)
            lane.right_lane_boundary_ids.append(right_id)
            if left_rb:
                lane.left_road_boundary_ids.append(left_rb)
            if right_rb:
                lane.right_road_boundary_ids.append(right_rb)
            msg.lane.append(lane)

        section = self.RoadSection()
        section.id = spec.section_id
        section.lane_id.extend(spec.lane_ids)
        section.road_left_boundary_ids.append(left_edge_id)
        section.road_right_boundary_ids.append(right_edge_id)
        section.road_center_line_points.extend(
            _points_from_xy(self.Point2D, *_sample_line(x0, x1, 0.0))
        )

        road = self.Road()
        road.id = spec.road_id
        road.road_type = self.URBAN_ROAD
        road.road_sections.append(section)
        msg.road.append(road)

        navi_section = self.NaviSection()
        navi_section.id = spec.section_id
        navi_section.length = spec.length
        navi_section.lane_count = len(spec.lane_ids)
        navi_section.lane_ids.extend(spec.lane_ids)
        navi_section.link_class = self.LINK_CLASS_REGULAR
        navi_section.link_type = self.LINK_TYPE_MAIN
        navi_section.link_direction = self.LINK_DIRECTION_POSITIVE

        navi_start = self.NaviPosition()
        navi_start.section_id = spec.section_id
        navi_start.s_offset = max(0.0, float(ego_x))
        navi_start.class_type = self.LINK_CLASS_REGULAR
        navi_start.link_type = self.LINK_TYPE_MAIN

        routes = self.NaviRoute()
        routes.id = spec.section_id
        routes.navi_start.CopyFrom(navi_start)
        routes.sections.append(navi_section)
        routes.raw_sections.append(navi_section)
        msg.routes.CopyFrom(routes)

        return msg.SerializeToString()

    def build_fovhdmap_bytes(
        self,
        timestamp: float,
        ego_x: float,
        ego_y: float,
        ego_yaw: float,
        seq_id: int,
        back: float = 40.0,
        forward: float = 180.0,
    ) -> bytes:
        spec = self.spec
        x0, x1 = _x_window(ego_x, spec, back, forward)
        msg = self.FovHdMap()
        hdr = msg.header
        hdr.timestamp = float(timestamp)
        hdr.seq_id = int(seq_id)

        lane_odr_ids = (1, 0, -1)
        for idx, (odr_lane, cy) in enumerate(zip(lane_odr_ids, spec.lane_center_y)):
            xs, ys = _sample_line(x0, x1, cy)
            rl = msg.roadlines.add()
            rl.id = 100 + idx
            rl.lane = int(odr_lane)
            rl.line = idx
            rl.midlane_id = rl.id
            rl.direction = 1
            rl.max_speed = 120
            rl.lane_type = 0
            rl.turn_type = 0
            rl.drv_pry = 1
            rl.followGps = 1
            rl.gpsDirect = 0
            setattr(rl, "class", 1)
            rl.type = 0
            rl.obs_avoid = 0
            rl.ramp_way = 0
            rl.x.extend(xs)
            rl.y.extend(ys)
            rl.z.extend([0.0] * len(xs))

        half_w = spec.lane_width * 0.5
        boundary_ys = (
            spec.lane_center_y[0] + half_w,
            spec.lane_center_y[1],
            spec.lane_center_y[2] + half_w,
            spec.lane_center_y[2] - half_w,
        )
        for idx, by in enumerate(boundary_ys):
            xs, ys = _sample_line(x0, x1, by)
            lm = msg.lanemarkings.add()
            lm.id = 200 + idx
            lm.x.extend(xs)
            lm.y.extend(ys)
            lm.z.extend([0.0] * len(xs))
            lm.marking = 3
            lm.lane_chang = 0 if idx != 1 else 3
            lm.type = 0
            lm.function = 0
            lm.direct_s = 1

        msg.map_hash_val = 1
        return msg.SerializeToString()

    def build_map_position_bytes(
        self,
        timestamp: float,
        ego_x: float,
        ego_y: float,
        ego_yaw: float,
        seq_id: int,
    ) -> bytes:
        msg = self.MapPosition()
        hdr = msg.header
        hdr.timestamp = float(timestamp)
        hdr.seq_id = int(seq_id)
        msg.global_x = float(ego_x)
        msg.global_y = float(ego_y)
        msg.azimuth = math.degrees(float(ego_yaw))
        msg.x = float(ego_x)
        msg.y = float(ego_y)
        msg.yaw = float(ego_yaw)
        msg.work_well = True
        msg.loc_status = 1
        msg.map_mode = 0
        return msg.SerializeToString()

    def build_all(
        self,
        timestamp: float,
        ego_x: float,
        ego_y: float,
        ego_yaw: float,
        seq_id: int,
    ) -> Tuple[bytes, bytes, bytes]:
        return (
            self.build_ld_map_bytes(timestamp, ego_x, ego_y, ego_yaw, seq_id),
            self.build_fovhdmap_bytes(timestamp, ego_x, ego_y, ego_yaw, seq_id),
            self.build_map_position_bytes(timestamp, ego_x, ego_y, ego_yaw, seq_id),
        )


def default_xodr_path() -> str:
    from assets import DEFAULT_XODR

    return DEFAULT_XODR
