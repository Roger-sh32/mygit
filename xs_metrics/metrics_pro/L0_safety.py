"""
l0_safety.py

Perfect Control 模式 L0 Safety Gate 指标实现模板。

核心假设：
1. 输入数据已经从 MCAP / DBF / protobuf 中解析成 Python dict。
2. 所有坐标已经在同一坐标系下，例如 local frame / map frame。
3. ego pose 使用 LocalPose。
4. 障碍物使用 LpLateFusionObjectInfo。
5. 实线信息使用 LocalHDMap。

作者建议：
- L0 指标触发即 FAIL。
- 不仅输出 bool，还要输出 trigger_time、actual_value、object_id 等诊断信息。
"""

from dataclasses import dataclass, asdict
from typing import Any, Dict, List, Optional, Tuple
import math


Point = Tuple[float, float]
Polygon = List[Point]


# ============================================================
# 1. 数据结构
# ============================================================

@dataclass
class VehicleParams:
    """
    车辆外包矩形参数。

    imu_to_front: 以车体参考点/IMU为原点，到车头的距离
    imu_to_back: 以车体参考点/IMU为原点，到车尾的距离
    half_width: 半车宽
    """
    imu_to_front: float
    imu_to_back: float
    half_width: float


@dataclass
class L0Thresholds:
    """
    L0 指标阈值配置。
    """
    collision_distance: float = 0.2

    vehicle_lon_distance: float = 1.0
    vehicle_lat_distance: float = 0.5

    pedestrian_lon_distance: float = 1.0
    pedestrian_lat_distance: float = 0.5


@dataclass
class MetricResult:
    """
    单个指标结果。
    """
    metric_name: str
    result: str                       # PASS / FAIL / WARNING
    triggered: bool
    threshold: Any = None
    actual_value: Optional[float] = None
    trigger_time: Optional[float] = None
    trigger_duration: Optional[float] = None
    related_channel: Optional[str] = None
    related_object_id: Optional[int] = None
    related_object_type: Optional[str] = None
    debug_message: str = ""


# ============================================================
# 2. 基础几何工具
# ============================================================

def rotate_point(x: float, y: float, yaw: float) -> Point:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        x * cos_yaw - y * sin_yaw,
        x * sin_yaw + y * cos_yaw,
    )


def transform_local_to_world(
    local_x: float,
    local_y: float,
    ego_x: float,
    ego_y: float,
    ego_yaw: float,
) -> Point:
    rx, ry = rotate_point(local_x, local_y, ego_yaw)
    return ego_x + rx, ego_y + ry


def transform_world_to_ego(
    world_x: float,
    world_y: float,
    ego_x: float,
    ego_y: float,
    ego_yaw: float,
) -> Point:
    """
    将世界坐标点转换到 ego 坐标系下。
    ego x 轴：车头方向
    ego y 轴：车体左侧方向
    """
    dx = world_x - ego_x
    dy = world_y - ego_y

    cos_yaw = math.cos(-ego_yaw)
    sin_yaw = math.sin(-ego_yaw)

    x_ego = dx * cos_yaw - dy * sin_yaw
    y_ego = dx * sin_yaw + dy * cos_yaw

    return x_ego, y_ego


def build_ego_polygon(
    pose: Dict[str, float],
    vehicle_params: VehicleParams,
) -> Polygon:
    """
    根据 LocalPose 和车辆参数生成 ego 外包矩形。

    pose 需要包含：
    {
        "x": float,
        "y": float,
        "yaw": float
    }
    """
    ego_x = pose["x"]
    ego_y = pose["y"]
    ego_yaw = pose["yaw"]

    front = vehicle_params.imu_to_front
    back = vehicle_params.imu_to_back
    half_width = vehicle_params.half_width

    # ego坐标系下，车头为 +x，车左为 +y
    corners_local = [
        (front, half_width),
        (front, -half_width),
        (-back, -half_width),
        (-back, half_width),
    ]

    return [
        transform_local_to_world(x, y, ego_x, ego_y, ego_yaw)
        for x, y in corners_local
    ]


def point_in_polygon(point: Point, polygon: Polygon) -> bool:
    """
    射线法判断点是否在多边形内。
    """
    x, y = point
    inside = False

    n = len(polygon)
    if n < 3:
        return False

    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]

        intersect = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-9) + xi
        )
        if intersect:
            inside = not inside

        j = i

    return inside


def ccw(a: Point, b: Point, c: Point) -> bool:
    return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])


def segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    return ccw(a, c, d) != ccw(b, c, d) and ccw(a, b, c) != ccw(a, b, d)


def point_to_segment_distance(p: Point, a: Point, b: Point) -> float:
    px, py = p
    ax, ay = a
    bx, by = b

    ab_x = bx - ax
    ab_y = by - ay

    ap_x = px - ax
    ap_y = py - ay

    ab_len_sq = ab_x * ab_x + ab_y * ab_y
    if ab_len_sq == 0:
        return math.hypot(px - ax, py - ay)

    t = (ap_x * ab_x + ap_y * ab_y) / ab_len_sq
    t = max(0.0, min(1.0, t))

    closest_x = ax + t * ab_x
    closest_y = ay + t * ab_y

    return math.hypot(px - closest_x, py - closest_y)


def polygons_intersect(poly1: Polygon, poly2: Polygon) -> bool:
    if len(poly1) < 3 or len(poly2) < 3:
        return False

    # 边相交
    for i in range(len(poly1)):
        a1 = poly1[i]
        a2 = poly1[(i + 1) % len(poly1)]

        for j in range(len(poly2)):
            b1 = poly2[j]
            b2 = poly2[(j + 1) % len(poly2)]

            if segments_intersect(a1, a2, b1, b2):
                return True

    # 包含关系
    if point_in_polygon(poly1[0], poly2):
        return True

    if point_in_polygon(poly2[0], poly1):
        return True

    return False


def polygon_distance(poly1: Polygon, poly2: Polygon) -> float:
    """
    计算两个多边形的最小距离。
    若相交，则距离为0。
    """
    if len(poly1) < 3 or len(poly2) < 3:
        return float("inf")

    if polygons_intersect(poly1, poly2):
        return 0.0

    min_dist = float("inf")

    # poly1点到poly2边
    for p in poly1:
        for i in range(len(poly2)):
            a = poly2[i]
            b = poly2[(i + 1) % len(poly2)]
            min_dist = min(min_dist, point_to_segment_distance(p, a, b))

    # poly2点到poly1边
    for p in poly2:
        for i in range(len(poly1)):
            a = poly1[i]
            b = poly1[(i + 1) % len(poly1)]
            min_dist = min(min_dist, point_to_segment_distance(p, a, b))

    return min_dist


# ============================================================
# 3. 字段解析工具
# ============================================================

def extract_polygon_from_object(obj: Dict[str, Any]) -> Polygon:
    """
    从目标障碍物中提取 polygon。

    兼容几种常见结构：
    1. obj["border"] = [{"x":..., "y":...}, ...]
    2. obj["corner"] = [{"x":..., "y":...}, ...]
    3. obj["center"] + length/width/r_angle 生成矩形
    """
    if "border" in obj and obj["border"]:
        return [(p["x"], p["y"]) for p in obj["border"]]

    if "corner" in obj and obj["corner"]:
        return [(p["x"], p["y"]) for p in obj["corner"]]

    if "corners" in obj and obj["corners"]:
        return [(p["x"], p["y"]) for p in obj["corners"]]

    # 兜底：通过中心点和长宽朝向生成外接矩形
    if all(k in obj for k in ["center_x", "center_y", "length", "width"]):
        cx = obj["center_x"]
        cy = obj["center_y"]
        length = obj["length"]
        width = obj["width"]
        yaw = obj.get("r_angle", obj.get("yaw", 0.0))

        half_l = length / 2.0
        half_w = width / 2.0

        local_corners = [
            (half_l, half_w),
            (half_l, -half_w),
            (-half_l, -half_w),
            (-half_l, half_w),
        ]

        polygon = []
        for lx, ly in local_corners:
            rx, ry = rotate_point(lx, ly, yaw)
            polygon.append((cx + rx, cy + ry))

        return polygon

    return []


def extract_polygon_from_area(area: Dict[str, Any]) -> Polygon:
    """
    从 obstacle_areas 中提取 polygon。
    """
    if "points" in area and area["points"]:
        return [(p["x"], p["y"]) for p in area["points"]]

    if "corner" in area and area["corner"]:
        return [(p["x"], p["y"]) for p in area["corner"]]

    if "corners" in area and area["corners"]:
        return [(p["x"], p["y"]) for p in area["corners"]]

    return []


def normalize_obj_type(obj_type: Any) -> str:
    """
    将 obj_type 标准化。
    实际项目里需要根据 xsproto 枚举值做映射。
    """
    if obj_type is None:
        return "UNKNOWN"

    if isinstance(obj_type, str):
        return obj_type.upper()

    # 这里是示例映射，实际要替换成你们 xsproto 的枚举值
    enum_mapping = {
        1: "VEHICLE",
        2: "PERSON",
        3: "CYCLIST",
        4: "CONE",
        5: "UNKNOWN",
    }

    return enum_mapping.get(obj_type, f"UNKNOWN_{obj_type}")


def get_track_id(obj: Dict[str, Any]) -> Optional[int]:
    return obj.get("track_id", obj.get("id"))


# ============================================================
# 4. L0指标实现
# ============================================================

def check_static_obstacle_collision(
    frames: List[Dict[str, Any]],
    vehicle_params: VehicleParams,
    thresholds: L0Thresholds,
) -> MetricResult:
    """
    通用静态障碍碰撞检测。

    frames 每帧建议结构：
    {
        "timestamp": float,
        "local_pose": {"x":..., "y":..., "yaw":...},
        "lp_late_fusion": {
            "obstacle_areas": [...]
        }
    }
    """
    min_distance = float("inf")
    first_trigger_time = None

    for frame in frames:
        timestamp = frame["timestamp"]
        pose = frame["local_pose"]
        fusion = frame.get("lp_late_fusion", {})

        ego_poly = build_ego_polygon(pose, vehicle_params)

        for area_idx, area in enumerate(fusion.get("obstacle_areas", [])):
            obstacle_poly = extract_polygon_from_area(area)
            if len(obstacle_poly) < 3:
                continue

            dist = polygon_distance(ego_poly, obstacle_poly)
            min_distance = min(min_distance, dist)

            if dist < thresholds.collision_distance:
                first_trigger_time = timestamp
                return MetricResult(
                    metric_name="Collision_StaticObstacle",
                    result="FAIL",
                    triggered=True,
                    threshold=thresholds.collision_distance,
                    actual_value=dist,
                    trigger_time=first_trigger_time,
                    related_channel="CHANNEL_LpLateFusionObjectInfo",
                    related_object_id=area_idx,
                    related_object_type="STATIC_OBSTACLE_AREA",
                    debug_message=(
                        f"Ego polygon distance to static obstacle area "
                        f"is {dist:.3f}m, below threshold."
                    ),
                )

    return MetricResult(
        metric_name="Collision_StaticObstacle",
        result="PASS",
        triggered=False,
        threshold=thresholds.collision_distance,
        actual_value=min_distance if min_distance != float("inf") else None,
        related_channel="CHANNEL_LpLateFusionObjectInfo",
        debug_message="No static obstacle collision detected.",
    )


def check_object_obstacle_collision(
    frames: List[Dict[str, Any]],
    vehicle_params: VehicleParams,
    thresholds: L0Thresholds,
) -> MetricResult:
    """
    目标类障碍碰撞检测。
    """
    min_distance = float("inf")
    min_obj_id = None
    min_obj_type = None

    for frame in frames:
        timestamp = frame["timestamp"]
        pose = frame["local_pose"]
        fusion = frame.get("lp_late_fusion", {})

        ego_poly = build_ego_polygon(pose, vehicle_params)

        for obj in fusion.get("obs_objs", []):
            obj_poly = extract_polygon_from_object(obj)
            if len(obj_poly) < 3:
                continue

            obj_type = normalize_obj_type(obj.get("obj_type"))
            obj_id = get_track_id(obj)

            dist = polygon_distance(ego_poly, obj_poly)

            if dist < min_distance:
                min_distance = dist
                min_obj_id = obj_id
                min_obj_type = obj_type

            if dist < thresholds.collision_distance:
                return MetricResult(
                    metric_name="Collision_ObjectObstacle",
                    result="FAIL",
                    triggered=True,
                    threshold=thresholds.collision_distance,
                    actual_value=dist,
                    trigger_time=timestamp,
                    related_channel="CHANNEL_LpLateFusionObjectInfo",
                    related_object_id=obj_id,
                    related_object_type=obj_type,
                    debug_message=(
                        f"Ego polygon distance to object {obj_id} "
                        f"({obj_type}) is {dist:.3f}m, below threshold."
                    ),
                )

    return MetricResult(
        metric_name="Collision_ObjectObstacle",
        result="PASS",
        triggered=False,
        threshold=thresholds.collision_distance,
        actual_value=min_distance if min_distance != float("inf") else None,
        related_channel="CHANNEL_LpLateFusionObjectInfo",
        related_object_id=min_obj_id,
        related_object_type=min_obj_type,
        debug_message="No object obstacle collision detected.",
    )


def compute_ego_frame_gaps(
    ego_pose: Dict[str, float],
    obj_poly_world: Polygon,
    vehicle_params: VehicleParams,
) -> Tuple[float, float]:
    """
    计算目标多边形相对 ego 外包矩形的纵向/横向边界距离。

    返回：
    lon_gap: 纵向边界距离，沿 ego x 方向
    lat_gap: 横向边界距离，沿 ego y 方向

    若某方向有重叠，则该方向 gap = 0。
    """
    ego_x = ego_pose["x"]
    ego_y = ego_pose["y"]
    ego_yaw = ego_pose["yaw"]

    obj_poly_ego = [
        transform_world_to_ego(x, y, ego_x, ego_y, ego_yaw)
        for x, y in obj_poly_world
    ]

    obj_xs = [p[0] for p in obj_poly_ego]
    obj_ys = [p[1] for p in obj_poly_ego]

    obj_min_x = min(obj_xs)
    obj_max_x = max(obj_xs)
    obj_min_y = min(obj_ys)
    obj_max_y = max(obj_ys)

    ego_min_x = -vehicle_params.imu_to_back
    ego_max_x = vehicle_params.imu_to_front
    ego_min_y = -vehicle_params.half_width
    ego_max_y = vehicle_params.half_width

    # 区间距离：若重叠则为0
    lon_gap = max(0.0, max(ego_min_x - obj_max_x, obj_min_x - ego_max_x))
    lat_gap = max(0.0, max(ego_min_y - obj_max_y, obj_min_y - ego_max_y))

    return lon_gap, lat_gap


def check_object_distance_too_small(
    frames: List[Dict[str, Any]],
    vehicle_params: VehicleParams,
    target_obj_type: str,
    lon_threshold: float,
    lat_threshold: float,
    metric_name: str,
) -> MetricResult:
    """
    通用目标距离过小检测。

    target_obj_type:
    - VEHICLE
    - PERSON
    """
    min_lon_gap = float("inf")
    min_lat_gap = float("inf")
    min_obj_id = None

    for frame in frames:
        timestamp = frame["timestamp"]
        pose = frame["local_pose"]
        fusion = frame.get("lp_late_fusion", {})

        for obj in fusion.get("obs_objs", []):
            obj_type = normalize_obj_type(obj.get("obj_type"))
            if obj_type != target_obj_type:
                continue

            obj_poly = extract_polygon_from_object(obj)
            if len(obj_poly) < 3:
                continue

            obj_id = get_track_id(obj)
            lon_gap, lat_gap = compute_ego_frame_gaps(
                ego_pose=pose,
                obj_poly_world=obj_poly,
                vehicle_params=vehicle_params,
            )

            if lon_gap + lat_gap < min_lon_gap + min_lat_gap:
                min_lon_gap = lon_gap
                min_lat_gap = lat_gap
                min_obj_id = obj_id

            if lon_gap <= lon_threshold and lat_gap <= lat_threshold:
                return MetricResult(
                    metric_name=metric_name,
                    result="FAIL",
                    triggered=True,
                    threshold={
                        "lon_threshold": lon_threshold,
                        "lat_threshold": lat_threshold,
                    },
                    actual_value=max(lon_gap, lat_gap),
                    trigger_time=timestamp,
                    related_channel="CHANNEL_LpLateFusionObjectInfo",
                    related_object_id=obj_id,
                    related_object_type=target_obj_type,
                    debug_message=(
                        f"{target_obj_type} object distance too small. "
                        f"lon_gap={lon_gap:.3f}m, lat_gap={lat_gap:.3f}m."
                    ),
                )

    return MetricResult(
        metric_name=metric_name,
        result="PASS",
        triggered=False,
        threshold={
            "lon_threshold": lon_threshold,
            "lat_threshold": lat_threshold,
        },
        actual_value=None
        if min_lon_gap == float("inf")
        else max(min_lon_gap, min_lat_gap),
        related_channel="CHANNEL_LpLateFusionObjectInfo",
        related_object_id=min_obj_id,
        related_object_type=target_obj_type,
        debug_message=(
            f"No {target_obj_type} distance-too-small event detected."
        ),
    )


def check_vehicle_distance_too_small(
    frames: List[Dict[str, Any]],
    vehicle_params: VehicleParams,
    thresholds: L0Thresholds,
) -> MetricResult:
    return check_object_distance_too_small(
        frames=frames,
        vehicle_params=vehicle_params,
        target_obj_type="VEHICLE",
        lon_threshold=thresholds.vehicle_lon_distance,
        lat_threshold=thresholds.vehicle_lat_distance,
        metric_name="DistanceTooSmall_Vehicle",
    )


def check_pedestrian_distance_too_small(
    frames: List[Dict[str, Any]],
    vehicle_params: VehicleParams,
    thresholds: L0Thresholds,
) -> MetricResult:
    return check_object_distance_too_small(
        frames=frames,
        vehicle_params=vehicle_params,
        target_obj_type="PERSON",
        lon_threshold=thresholds.pedestrian_lon_distance,
        lat_threshold=thresholds.pedestrian_lat_distance,
        metric_name="DistanceTooSmall_Pedestrian",
    )


def check_solid_line_violation(
    frames: List[Dict[str, Any]],
    vehicle_params: VehicleParams,
    forbidden_roles: Optional[List[Any]] = None,
) -> MetricResult:
    """
    主车压实线检测。

    逻辑：
    如果 LocalHDMap 中 lane marking 点落入 ego 外包矩形内，
    且该点 lane_change_role 为 BI_FORBIDDEN，则触发。

    frames 每帧建议结构：
    {
        "timestamp": float,
        "local_pose": {"x":..., "y":..., "yaw":...},
        "local_hdmap": {
            "future_lanemarkings": [
                {
                    "id": "...",
                    "ext_points": [
                        {"x":..., "y":..., "lane_change_role": "BI_FORBIDDEN"}
                    ]
                }
            ]
        }
    }
    """
    if forbidden_roles is None:
        forbidden_roles = ["BI_FORBIDDEN", 3]

    for frame in frames:
        timestamp = frame["timestamp"]
        pose = frame["local_pose"]
        hdmap = frame.get("local_hdmap", {})

        ego_poly = build_ego_polygon(pose, vehicle_params)

        for marking in hdmap.get("future_lanemarkings", []):
            marking_id = marking.get("id")

            for point in marking.get("ext_points", []):
                role = point.get("lane_change_role")

                if role not in forbidden_roles:
                    continue

                p = (point["x"], point["y"])

                if point_in_polygon(p, ego_poly):
                    return MetricResult(
                        metric_name="SolidLineViolation",
                        result="FAIL",
                        triggered=True,
                        threshold="lane_change_role == BI_FORBIDDEN",
                        actual_value=None,
                        trigger_time=timestamp,
                        related_channel="CHANNEL_LocalHDMap",
                        related_object_id=marking_id,
                        related_object_type="LANE_MARKING",
                        debug_message=(
                            f"Ego polygon overlaps forbidden lane marking. "
                            f"marking_id={marking_id}, role={role}."
                        ),
                    )

    return MetricResult(
        metric_name="SolidLineViolation",
        result="PASS",
        triggered=False,
        threshold="lane_change_role == BI_FORBIDDEN",
        related_channel="CHANNEL_LocalHDMap",
        debug_message="No solid line violation detected.",
    )


# ============================================================
# 5. L0统一入口
# ============================================================

def evaluate_l0_safety(
    frames: List[Dict[str, Any]],
    vehicle_params: VehicleParams,
    thresholds: Optional[L0Thresholds] = None,
) -> List[MetricResult]:
    """
    L0 Safety Gate 统一评价入口。

    frames 是已经时间同步后的数据帧列表。

    返回：
    List[MetricResult]
    """
    if thresholds is None:
        thresholds = L0Thresholds()

    results = [
        check_static_obstacle_collision(
            frames=frames,
            vehicle_params=vehicle_params,
            thresholds=thresholds,
        ),
        check_object_obstacle_collision(
            frames=frames,
            vehicle_params=vehicle_params,
            thresholds=thresholds,
        ),
        check_vehicle_distance_too_small(
            frames=frames,
            vehicle_params=vehicle_params,
            thresholds=thresholds,
        ),
        check_pedestrian_distance_too_small(
            frames=frames,
            vehicle_params=vehicle_params,
            thresholds=thresholds,
        ),
        check_solid_line_violation(
            frames=frames,
            vehicle_params=vehicle_params,
        ),
    ]

    return results


def l0_overall_result(results: List[MetricResult]) -> str:
    """
    L0 总结果：
    任意一个 FAIL，则整体 FAIL。
    """
    return "FAIL" if any(r.result == "FAIL" for r in results) else "PASS"


# ============================================================
# 6. 示例
# ============================================================

if __name__ == "__main__":
    vehicle_params = VehicleParams(
        imu_to_front=2.8,
        imu_to_back=1.2,
        half_width=0.9,
    )

    thresholds = L0Thresholds(
        collision_distance=0.2,
        vehicle_lon_distance=1.0,
        vehicle_lat_distance=0.5,
        pedestrian_lon_distance=1.0,
        pedestrian_lat_distance=0.5,
    )

    # 示例帧数据：实际项目中应由 MCAP / DBF / proto 解析得到
    frames = [
        {
            "timestamp": 0.0,
            "local_pose": {
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            },
            "lp_late_fusion": {
                "obstacle_areas": [
                    {
                        "points": [
                            {"x": 10.0, "y": -2.0},
                            {"x": 12.0, "y": -2.0},
                            {"x": 12.0, "y": -1.0},
                            {"x": 10.0, "y": -1.0},
                        ]
                    }
                ],
                "obs_objs": [
                    {
                        "track_id": 1001,
                        "obj_type": "VEHICLE",
                        "center_x": 5.0,
                        "center_y": 0.0,
                        "length": 4.5,
                        "width": 1.8,
                        "r_angle": 0.0,
                    }
                ],
            },
            "local_hdmap": {
                "future_lanemarkings": [
                    {
                        "id": "lane_boundary_001",
                        "ext_points": [
                            {
                                "x": 0.0,
                                "y": 1.2,
                                "lane_change_role": "BI_FORBIDDEN",
                            }
                        ],
                    }
                ]
            },
        }
    ]

    results = evaluate_l0_safety(
        frames=frames,
        vehicle_params=vehicle_params,
        thresholds=thresholds,
    )

    print("===== L0 Safety Results =====")
    for result in results:
        print(asdict(result))

    print("Overall:", l0_overall_result(results))