"""OSI 状态 → xsproto LocalPose / VehicleStatus / TrajPrediction / PerceptionObjectInfo 序列化。"""

from __future__ import annotations

import math
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

# xs_proto 的 pb2 使用 `from base import ...`，需把 xs_proto 根目录加入 path
_XSPROTO_CANDIDATES = [
    os.environ.get("XS_PROTO_PYTHON", ""),
    os.path.expanduser("~/.local/lib/python3.10/site-packages/xs_proto"),
    "/home/xs/.local/lib/python3.10/site-packages/xs_proto",
]


def _ensure_xsproto_path() -> str:
    for p in _XSPROTO_CANDIDATES:
        if not p:
            continue
        base_pb = os.path.join(p, "base", "local_pose_pb2.py")
        if os.path.isfile(base_pb):
            if p not in sys.path:
                sys.path.insert(0, p)
            return p
    raise ImportError(
        "未找到 xs_proto Python 包。请安装或设置 XS_PROTO_PYTHON 指向 xs_proto 目录 "
        "（需含 base/local_pose_pb2.py，Python 3.10）。"
    )


def _import_proto():
    _ensure_xsproto_path()
    from base.local_pose_pb2 import LocalPose  # type: ignore
    from base.vehicle_status_pb2 import VehicleStatus  # type: ignore

    return LocalPose, VehicleStatus


def _import_prediction_proto():
    _ensure_xsproto_path()
    from perception.prediction_pb2 import (  # type: ignore
        PredictionVehicleInfoMsg,
        PredictionVehicleObject,
    )

    return PredictionVehicleInfoMsg, PredictionVehicleObject


def _import_perception_proto():
    _ensure_xsproto_path()
    from perception.perception_object_info_pb2 import (  # type: ignore
        ObjSubType,
        ObjType,
        PerceptionObjectInfo,
        TrackState,
    )

    return PerceptionObjectInfo, TrackState, ObjType, ObjSubType


def _append_cv_prediction_trajectory(pvo, obs: Dict[str, Any], num_points: int = 10, dt: float = 0.1) -> None:
    """为 PredictionVehicleObject 添加匀速外推轨迹（ego 坐标系）。"""
    traj = pvo.trajectories.add()
    traj.probablity = 1.0
    traj.intention = 0
    cx = float(obs.get("center_x", 0.0))
    cy = float(obs.get("center_y", 0.0))
    vx = float(obs.get("velocity_x", 0.0))
    vy = float(obs.get("velocity_y", 0.0))
    for i in range(num_points):
        pt = traj.points.add()
        pt.timestamp = i * dt
        pt.x = cx + vx * pt.timestamp
        pt.y = cy + vy * pt.timestamp
        pt.z = 0.0


def world_to_ego(
    ego_x: float,
    ego_y: float,
    ego_yaw: float,
    wx: float,
    wy: float,
) -> Tuple[float, float]:
    """全局坐标 → ego 车体坐标（x 前、y 左）。"""
    dx = wx - ego_x
    dy = wy - ego_y
    c = math.cos(ego_yaw)
    s = math.sin(ego_yaw)
    return dx * c + dy * s, -dx * s + dy * c


def world_vel_to_ego(
    ego_yaw: float,
    vx: float,
    vy: float,
) -> Tuple[float, float]:
    """全局速度 → ego 车体坐标。"""
    c = math.cos(ego_yaw)
    s = math.sin(ego_yaw)
    return vx * c + vy * s, -vx * s + vy * c


class XsProtoBuilder:
    """从 esmini OSI ego 构造 LocalPose / VehicleStatus / TrajPrediction / PerceptionObjectInfo 字节。"""

    def __init__(self):
        self.LocalPose, self.VehicleStatus = _import_proto()
        self.PredictionVehicleInfoMsg, self.PredictionVehicleObject = (
            _import_prediction_proto()
        )
        (
            self.PerceptionObjectInfo,
            self.TrackState,
            self.ObjType,
            self.ObjSubType,
        ) = _import_perception_proto()
        self._seq = 0

    def build_local_pose_bytes(
        self,
        timestamp: float,
        x: float,
        y: float,
        z: float,
        yaw_rad: float,
        speed_ms: float,
        seq_id: Optional[int] = None,
    ) -> bytes:
        VS = self.VehicleStatus
        if seq_id is None:
            self._seq += 1
            seq_id = self._seq

        msg = self.LocalPose()
        msg.timestamp = float(timestamp)
        msg.seq_id = int(seq_id)
        msg.dr_x = float(x)
        msg.dr_y = float(y)
        msg.dr_z = float(z)
        msg.dr_roll = 0.0
        msg.dr_pitch = 0.0
        msg.dr_heading = math.degrees(float(yaw_rad))
        msg.linear_velocity = float(speed_ms)
        msg.gear = VS.GEAR_D
        msg.driving_mode = VS.DRIVING_MODE_AUTO
        msg.motion_model = VS.MOTION_MODEL_ACKERMAN

        imu = msg.imu
        imu.mode = VS.Imu.IMU_MODE_AXIS_9
        imu.heading = msg.dr_heading
        imu.roll = 0.0
        imu.pitch = 0.0

        return msg.SerializeToString()

    def build_vehicle_status_bytes(
        self,
        timestamp: float,
        speed_ms: float,
        seq_id: Optional[int] = None,
    ) -> bytes:
        VS = self.VehicleStatus
        if seq_id is None:
            seq_id = self._seq

        msg = VS()
        msg.timestamp = float(timestamp)
        msg.seq_id = int(seq_id)
        msg.gear = VS.GEAR_D
        msg.driving_mode = VS.DRIVING_MODE_AUTO
        msg.motion_model = VS.MOTION_MODEL_ACKERMAN
        msg.linear_velocity = float(speed_ms)
        msg.diving_enable = True
        return msg.SerializeToString()

    def build_from_ego(
        self,
        timestamp: float,
        x: float,
        y: float,
        z: float,
        yaw_rad: float,
        speed_ms: float,
    ) -> Tuple[bytes, bytes]:
        """返回 (local_pose_bytes, vehicle_status_bytes)，共用 seq_id。"""
        self._seq += 1
        sid = self._seq
        lp = self.build_local_pose_bytes(
            timestamp, x, y, z, yaw_rad, speed_ms, seq_id=sid
        )
        vs = self.build_vehicle_status_bytes(timestamp, speed_ms, seq_id=sid)
        return lp, vs

    def build_prediction_bytes(
        self,
        timestamp: float,
        ego_x: float,
        ego_y: float,
        ego_yaw: float,
        ego_speed_ms: float,
        obstacles: Optional[List[Dict[str, Any]]] = None,
        seq_id: Optional[int] = None,
    ) -> bytes:
        """
        构造 TrajPrediction（PredictionVehicleInfoMsg）字节。

        header.timestamp 与 LocalPose 对齐，避免 input_gateway 报 MSG_OBSTACLE_DELAY。
        obstacles 元素字段：track_id, center_x, center_y, length, width, velocity_x,
        velocity_y, speed, r_angle（均为 ego 坐标系，除 track_id 外）。
        """
        if seq_id is None:
            seq_id = self._seq

        msg = self.PredictionVehicleInfoMsg()
        hdr = msg.header
        hdr.timestamp = float(timestamp)
        hdr.seq_id = int(seq_id)

        lp = hdr.local_pose
        lp.timestamp = float(timestamp)
        lp.seq_id = int(seq_id)
        lp.dr_x = float(ego_x)
        lp.dr_y = float(ego_y)
        lp.dr_heading = math.degrees(float(ego_yaw))
        lp.linear_velocity = float(ego_speed_ms)
        lp.gear = self.VehicleStatus.GEAR_D
        lp.driving_mode = self.VehicleStatus.DRIVING_MODE_AUTO

        for obs in obstacles or []:
            pvo = msg.prediction_vehicle_object.add()
            obj = pvo.obstacle_obj
            obj.track_id = int(obs.get("track_id", 0))
            obj.track_state = 1
            obj.track_count = 1
            obj.obj_type = 0
            obj.obj_subtype = 2
            obj.score = 1.0
            obj.center_x = float(obs.get("center_x", 0.0))
            obj.center_y = float(obs.get("center_y", 0.0))
            obj.length = float(obs.get("length", 4.5))
            obj.width = float(obs.get("width", 1.8))
            obj.r_angle = float(obs.get("r_angle", 0.0))
            obj.front_heading = float(obs.get("r_angle", 0.0))
            obj.velocity_x = float(obs.get("velocity_x", 0.0))
            obj.velocity_y = float(obs.get("velocity_y", 0.0))
            obj.speed = float(obs.get("speed", 0.0))
            obj.motion_prob = 1.0
            pvo.in_road_flag = True
            _append_cv_prediction_trajectory(pvo, obs)

        return msg.SerializeToString()

    def build_perception_object_info_bytes(
        self,
        timestamp: float,
        ego_x: float,
        ego_y: float,
        ego_yaw: float,
        ego_speed_ms: float,
        obstacles: Optional[List[Dict[str, Any]]] = None,
        seq_id: Optional[int] = None,
    ) -> bytes:
        """
        构造 LpLateFusionObjectInfo / PerceptionObjectInfo 字节。

        Planning C++ 的 PredictionAdapter 需要 TrajPrediction 与本消息同时存在，
        才会把 OSI 障碍车融合为 prediction obstacles。
        """
        if seq_id is None:
            seq_id = self._seq

        msg = self.PerceptionObjectInfo()
        hdr = msg.header
        hdr.timestamp = float(timestamp)
        hdr.seq_id = int(seq_id)

        lp = hdr.local_pose
        lp.timestamp = float(timestamp)
        lp.seq_id = int(seq_id)
        lp.dr_x = float(ego_x)
        lp.dr_y = float(ego_y)
        lp.dr_heading = math.degrees(float(ego_yaw))
        lp.linear_velocity = float(ego_speed_ms)
        lp.gear = self.VehicleStatus.GEAR_D
        lp.driving_mode = self.VehicleStatus.DRIVING_MODE_AUTO

        for obs in obstacles or []:
            obj = msg.obs_objs.add()
            obj.track_id = int(obs.get("track_id", 0))
            obj.track_state = self.TrackState.TRACK_STATE_TRACKING
            obj.confidence = 1.0
            obj.tracking_stability = 1.0
            obj.obj_type = self.ObjType.OBJ_TYPE_VEHICLE
            obj.obj_subtype = self.ObjSubType.OBJ_ST_CAR
            obj.center.x = float(obs.get("center_x", 0.0))
            obj.center.y = float(obs.get("center_y", 0.0))
            obj.length = float(obs.get("length", 4.5))
            obj.width = float(obs.get("width", 1.8))
            obj.height = float(obs.get("height", 1.5))
            obj.height_min = 0.0
            obj.r_angle = float(obs.get("r_angle", 0.0))
            speed = float(obs.get("speed", 0.0))
            obj.linear_speed = speed
            vx = float(obs.get("velocity_x", 0.0))
            vy = float(obs.get("velocity_y", 0.0))
            if speed > 0.05:
                obj.linear_speed_angle = math.atan2(vy, vx)
            else:
                obj.linear_speed_angle = float(obs.get("r_angle", 0.0))

        return msg.SerializeToString()


def build_local_pose_history(
    builder: XsProtoBuilder,
    frames: List[dict],
) -> List[bytes]:
    """多帧历史 → local_pose_bytes 列表（与 MCAP loader 行为接近）。"""
    out: List[bytes] = []
    for fr in frames:
        out.append(
            builder.build_local_pose_bytes(
                fr["timestamp"],
                fr["x"],
                fr["y"],
                fr.get("z", 0.0),
                fr["yaw"],
                fr["speed"],
                seq_id=fr.get("seq_id"),
            )
        )
    return out
