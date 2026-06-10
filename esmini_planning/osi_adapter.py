"""OSI GroundTruth → PlanningInput（P1–P4：位姿、预测、地图、感知、TaskList）。"""

from __future__ import annotations

import math
from collections import deque
from typing import Any, Deque, Dict, List, Optional

from map_builder import StraightRoadMapBuilder
from task_list_builder import StraightRoadTaskListBuilder
from xsproto_builder import (
    XsProtoBuilder,
    build_local_pose_history,
    world_to_ego,
    world_vel_to_ego,
)


def _longitudinal_speed(vx: float, vy: float, yaw: float) -> float:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return vx * c + vy * s


def _moving_by_id(moving_objects, id_value: int):
    for o in moving_objects:
        if o.id.value == id_value:
            return o
    return None


def extract_ego(msg, object_id: int = 0):
    """从 OSI 取 Ego moving_object。"""
    objs = msg.moving_object
    if not objs:
        return None
    ego = _moving_by_id(objs, object_id)
    if ego is not None:
        return ego
    hid = msg.host_vehicle_id
    if hid is not None and hid.value != 0:
        ego = _moving_by_id(objs, hid.value)
        if ego is not None:
            return ego
    return objs[0]


def osi_timestamp(msg) -> float:
    ts = msg.timestamp.seconds + msg.timestamp.nanos * 1e-9
    return ts if ts > 0 else 0.001


def build_e2e_trajectory(
    x: float,
    y: float,
    heading: float,
    velocity: float,
    num_points: int = 50,
    point_spacing: float = 0.5,
    dt: float = 0.1,
) -> List[dict]:
    """沿当前航向生成 E2E 参考线。"""
    points: List[dict] = []
    for i in range(num_points):
        s = i * point_spacing
        points.append(
            {
                "x": x + s * math.cos(heading),
                "y": y + s * math.sin(heading),
                "heading": heading,
                "kappa": 0.0,
                "v": velocity,
                "a": 0.0,
                "s": s,
                "t": i * dt,
            }
        )
    return points


def extract_obstacles_for_prediction(
    msg,
    ego,
    object_id: int = 0,
) -> List[Dict[str, Any]]:
    """OSI 非 ego 车辆 → TrajPrediction 障碍物列表（ego 坐标系）。"""
    if ego is None:
        return []

    ex = ego.base.position.x
    ey = ego.base.position.y
    eh = ego.base.orientation.yaw
    ego_oid = ego.id.value
    out: List[Dict[str, Any]] = []

    for obj in msg.moving_object:
        oid = obj.id.value
        if oid == ego_oid or oid == object_id:
            continue

        cx, cy = world_to_ego(ex, ey, eh, obj.base.position.x, obj.base.position.y)
        vx, vy = world_vel_to_ego(eh, obj.base.velocity.x, obj.base.velocity.y)
        rel_yaw = obj.base.orientation.yaw - eh
        while rel_yaw > math.pi:
            rel_yaw -= 2 * math.pi
        while rel_yaw < -math.pi:
            rel_yaw += 2 * math.pi

        dim = obj.base.dimension
        length = dim.length if dim.length > 0.1 else 4.5
        width = dim.width if dim.width > 0.1 else 1.8

        out.append(
            {
                "track_id": oid,
                "center_x": cx,
                "center_y": cy,
                "length": length,
                "width": width,
                "velocity_x": vx,
                "velocity_y": vy,
                "speed": math.hypot(vx, vy),
                "r_angle": rel_yaw,
            }
        )

    return out


def estimate_forward_ttc(
    msg,
    ego,
    object_id: int = 0,
    vehicle_margin: float = 3.0,
    lane_half_width: float = 2.5,
) -> float:
    """估算前方最近同车道目标的 TTC（秒），无威胁时返回 inf。"""
    return _estimate_ttc(msg, ego, object_id, vehicle_margin, lane_half_width)


def estimate_threat_ttc(
    msg,
    ego,
    object_id: int = 0,
    vehicle_margin: float = 3.0,
    lane_half_width: float = 4.5,
) -> float:
    """前方威胁 TTC（含邻道切入），横向窗口更宽。"""
    return _estimate_ttc(msg, ego, object_id, vehicle_margin, lane_half_width)


def _estimate_ttc(
    msg,
    ego,
    object_id: int,
    vehicle_margin: float,
    lane_half_width: float,
) -> float:
    """估算前方目标 TTC（秒）。"""
    if ego is None:
        return math.inf

    ex = ego.base.position.x
    ey = ego.base.position.y
    ego_yaw = ego.base.orientation.yaw
    vx_e = ego.base.velocity.x
    vy_e = ego.base.velocity.y
    ego_oid = ego.id.value

    best_lon = None
    target = None
    for obj in msg.moving_object:
        oid = obj.id.value
        if oid == ego_oid or oid == object_id:
            continue
        dx = obj.base.position.x - ex
        dy = obj.base.position.y - ey
        c = math.cos(ego_yaw)
        s = math.sin(ego_yaw)
        lon = dx * c + dy * s
        lat = -dx * s + dy * c
        if lon <= 0.0 or abs(lat) > lane_half_width:
            continue
        if best_lon is None or lon < best_lon:
            best_lon = lon
            target = obj

    if target is None or best_lon is None:
        return math.inf

    gap = best_lon - vehicle_margin
    if gap <= 0.0:
        return 0.0

    c = math.cos(ego_yaw)
    s = math.sin(ego_yaw)
    ego_along = vx_e * c + vy_e * s
    target_along = target.base.velocity.x * c + target.base.velocity.y * s
    closing = ego_along - target_along
    if closing <= 0.05:
        return math.inf
    return gap / closing


class OsminiPlanningAdapter:
    """
    将 esmini OSI 帧转为 PlanningInput。

    P1：local_pose_bytes + vehicle_status_bytes + e2e_trajectory
    P1.5：prediction_bytes（TrajPrediction，时间戳对齐）
    P2：ld_map_bytes + fovhdmap_bytes + map_position_bytes（straight_500m）
    P3：perception_object_info_bytes（与 TrajPrediction 配对，C++ 才识别 obstacles）
    P4：task_list_bytes（straight_500m 直道路由，消除 NAVIGATION_ERROR / all_lane_seqs）
    """

    def __init__(
        self,
        planning_input_cls: Any,
        num_e2e_points: int = 50,
        point_spacing: float = 0.5,
        default_speed_ms: float = 20.0,
        pose_history_len: int = 10,
    ):
        self._PlanningInput = planning_input_cls
        self.num_e2e_points = num_e2e_points
        self.point_spacing = point_spacing
        self.default_speed_ms = default_speed_ms
        self.pose_history_len = pose_history_len
        self._proto = XsProtoBuilder()
        self._map = StraightRoadMapBuilder()
        self._task = StraightRoadTaskListBuilder()
        self._pose_history: Deque[Dict] = deque(maxlen=pose_history_len)
        self._frame_seq = 0
        self.last_obstacle_count = 0
        self.last_task_point_count = 0

    def feed_osi(
        self,
        msg,
        object_id: int = 0,
        override_speed_ms: Optional[float] = None,
    ) -> None:
        """bootstrap 阶段只累积 LocalPose 历史，不跑 planning。"""
        ego = extract_ego(msg, object_id)
        if ego is None:
            return

        x = ego.base.position.x
        y = ego.base.position.y
        z = ego.base.position.z
        heading = ego.base.orientation.yaw
        vx = ego.base.velocity.x
        vy = ego.base.velocity.y
        speed = _longitudinal_speed(vx, vy, heading)
        if override_speed_ms is not None:
            speed = override_speed_ms
        elif speed < 0.5:
            speed = self.default_speed_ms

        ts = osi_timestamp(msg)
        self._frame_seq += 1
        self._pose_history.append(
            {
                "timestamp": ts,
                "x": x,
                "y": y,
                "z": z,
                "yaw": heading,
                "speed": speed,
                "seq_id": self._frame_seq,
            }
        )

    def _resolve_speed(
        self,
        speed: float,
        override_speed_ms: Optional[float],
        max_speed_ms: Optional[float],
    ) -> float:
        if override_speed_ms is not None:
            return override_speed_ms
        if speed < 0.5:
            return self.default_speed_ms
        cap = max_speed_ms if max_speed_ms is not None else self.default_speed_ms
        # stateXYH bootstrap 后 OSI 速度偶发飙到 80+ m/s，需钳位避免 planning stitch 崩
        if speed > cap * 1.15:
            return cap
        return speed

    def build_from_osi(
        self,
        msg,
        object_id: int = 0,
        override_speed_ms: Optional[float] = None,
        max_speed_ms: Optional[float] = None,
    ):
        ego = extract_ego(msg, object_id)
        if ego is None:
            return None

        x = ego.base.position.x
        y = ego.base.position.y
        z = ego.base.position.z
        heading = ego.base.orientation.yaw
        vx = ego.base.velocity.x
        vy = ego.base.velocity.y
        speed = _longitudinal_speed(vx, vy, heading)
        speed = self._resolve_speed(speed, override_speed_ms, max_speed_ms)

        ts = osi_timestamp(msg)

        self._frame_seq += 1
        self._pose_history.append(
            {
                "timestamp": ts,
                "x": x,
                "y": y,
                "z": z,
                "yaw": heading,
                "speed": speed,
                "seq_id": self._frame_seq,
            }
        )

        local_pose_bytes = build_local_pose_history(
            self._proto, list(self._pose_history)
        )
        vehicle_status_bytes = self._proto.build_vehicle_status_bytes(
            ts, speed, seq_id=self._frame_seq
        )
        obstacles = extract_obstacles_for_prediction(msg, ego, object_id)
        self.last_obstacle_count = len(obstacles)
        prediction_bytes = self._proto.build_prediction_bytes(
            ts,
            x,
            y,
            heading,
            speed,
            obstacles=obstacles,
            seq_id=self._frame_seq,
        )
        perception_object_info_bytes = self._proto.build_perception_object_info_bytes(
            ts,
            x,
            y,
            heading,
            speed,
            obstacles=obstacles,
            seq_id=self._frame_seq,
        )
        ld_map_bytes, fovhdmap_bytes, map_position_bytes = self._map.build_all(
            ts, x, y, heading, self._frame_seq
        )
        task_list_bytes = self._task.build_task_list_bytes(
            ts,
            x,
            y,
            heading,
            self._frame_seq,
            max_speed_kmh=max(30.0, min(120.0, speed * 3.6)),
        )
        future_n, _ = self._task.point_count_hint(x)
        self.last_task_point_count = future_n

        return self._PlanningInput(
            timestamp=ts,
            local_pose_bytes=local_pose_bytes,
            vehicle_status_bytes=vehicle_status_bytes,
            ld_map_bytes=ld_map_bytes,
            prediction_bytes=prediction_bytes,
            perception_object_info_bytes=perception_object_info_bytes,
            fovhdmap_bytes=fovhdmap_bytes,
            map_position_bytes=map_position_bytes,
            task_list_bytes=task_list_bytes,
            e2e_trajectory_points=build_e2e_trajectory(
                x,
                y,
                heading,
                speed,
                num_points=self.num_e2e_points,
                point_spacing=self.point_spacing,
            ),
        )
