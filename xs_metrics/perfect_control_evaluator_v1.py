"""
perfect_control_evaluator_v1.py

按规控原始指标需求实现（Perfect Control 模式）
---------------------------------------------------
覆盖指标：
1. 通用障碍碰撞检测
2. 目标类障碍碰撞检测
3. 规划速度过大
4. 规划速度过小
5. 规划加速度过大
6. 规划加速度过小
7. 规划加加速度（Jerk）过大
8. 规划加加速度（Jerk）过小
9. 车辆目标类障碍距离过小
10. 路径规划无结果
11. 主车压实线
12. 速度规划无结果
13. 转向角变化大
14. 转向角变化率大

核心字段来源：
CHANNEL_LocalPathInfo
CHANNEL_LocalPose
CHANNEL_LpLateFusionObjectInfo
CHANNEL_LocalHDMap
"""

import math
import pandas as pd
from shapely.geometry import Polygon, Point
from shapely.ops import nearest_points


class PerfectControlEvaluator:
    def __init__(self):
        self.thresholds = {
            "collision": 0.2,
            "speed_max": 12.0,
            "speed_min": 1.0,
            "acc_max": 2.0,
            "acc_min": -2.0,
            "jerk_max": 2.0,
            "jerk_min": -2.0,
            "vehicle_lon_min": 1.0,
            "vehicle_lat_min": 0.5,
            "path_point_min": 10,
            "plan_speed_min": 0.01,
            "ego_speed_min": 0.01,
            "front_wheel_angle_max_deg": 20.0,
            "front_wheel_angle_rate_max_deg": 10.0,
        }

        self.results = []

    # =============================
    # 基础工具
    # =============================
    def add_result(self, name, passed, value, threshold, timestamp=None, detail=""):
        self.results.append({
            "metric": name,
            "passed": passed,
            "value": value,
            "threshold": threshold,
            "timestamp": timestamp,
            "detail": detail,
        })

    @staticmethod
    def build_ego_polygon(x, y, yaw, front, back, half_width):
        """
        以车辆中心构造外包矩形
        """
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        corners_local = [
            (front, half_width),
            (front, -half_width),
            (-back, -half_width),
            (-back, half_width),
        ]

        corners_world = []
        for px, py in corners_local:
            wx = x + px * cos_y - py * sin_y
            wy = y + px * sin_y + py * cos_y
            corners_world.append((wx, wy))

        return Polygon(corners_world)

    # =============================
    # 1. 通用静态障碍碰撞
    # =============================
    def evaluate_static_collision(self, ego_df, static_obs_df):
        min_dist = float("inf")
        trigger_time = None

        for _, ego in ego_df.iterrows():
            ego_poly = self.build_ego_polygon(
                ego.x, ego.y, ego.yaw,
                ego.front, ego.back, ego.half_width
            )

            current_obs = static_obs_df[static_obs_df.timestamp == ego.timestamp]

            for _, obs in current_obs.iterrows():
                obs_poly = Polygon(eval(obs.polygon))
                d = ego_poly.distance(obs_poly)

                if d < min_dist:
                    min_dist = d
                    trigger_time = ego.timestamp

        self.add_result(
            "static_collision",
            min_dist >= self.thresholds["collision"],
            round(min_dist, 4),
            self.thresholds["collision"],
            trigger_time
        )

    # =============================
    # 2. 动态目标类障碍碰撞
    # =============================
    def evaluate_dynamic_collision(self, ego_df, dynamic_obs_df):
        min_dist = float("inf")
        trigger_time = None
        trigger_obj = None

        for _, ego in ego_df.iterrows():
            ego_poly = self.build_ego_polygon(
                ego.x, ego.y, ego.yaw,
                ego.front, ego.back, ego.half_width
            )

            current_obs = dynamic_obs_df[dynamic_obs_df.timestamp == ego.timestamp]

            for _, obs in current_obs.iterrows():
                obs_poly = Polygon(eval(obs.border))
                d = ego_poly.distance(obs_poly)

                if d < min_dist:
                    min_dist = d
                    trigger_time = ego.timestamp
                    trigger_obj = obs.obj_id

        self.add_result(
            "dynamic_collision",
            min_dist >= self.thresholds["collision"],
            round(min_dist, 4),
            self.thresholds["collision"],
            trigger_time,
            f"obj_id={trigger_obj}"
        )

    # =============================
    # 3-8 Planning Speed/Acc/Jerk
    # =============================
    def evaluate_planning_dynamics(self, path_df):
        # speed
        max_speed = path_df.speed.max()
        min_speed = path_df.speed.min()

        self.add_result(
            "planning_speed_too_high",
            max_speed <= self.thresholds["speed_max"],
            round(max_speed, 4),
            self.thresholds["speed_max"]
        )

        self.add_result(
            "planning_speed_too_low",
            min_speed >= self.thresholds["speed_min"],
            round(min_speed, 4),
            self.thresholds["speed_min"]
        )

        # acc
        max_acc = path_df.acceleration.max()
        min_acc = path_df.acceleration.min()

        self.add_result(
            "planning_acc_too_high",
            max_acc <= self.thresholds["acc_max"],
            round(max_acc, 4),
            self.thresholds["acc_max"]
        )

        self.add_result(
            "planning_acc_too_low",
            min_acc >= self.thresholds["acc_min"],
            round(min_acc, 4),
            self.thresholds["acc_min"]
        )

        # jerk
        path_df = path_df.sort_values("timestamp").copy()
        path_df["dt"] = path_df["timestamp"].diff()
        path_df["jerk"] = path_df["acceleration"].diff() / path_df["dt"]

        max_jerk = path_df["jerk"].max()
        min_jerk = path_df["jerk"].min()

        self.add_result(
            "planning_jerk_too_high",
            max_jerk <= self.thresholds["jerk_max"],
            round(max_jerk, 4),
            self.thresholds["jerk_max"]
        )

        self.add_result(
            "planning_jerk_too_low",
            min_jerk >= self.thresholds["jerk_min"],
            round(min_jerk, 4),
            self.thresholds["jerk_min"]
        )

    # =============================
    # 9. 车辆目标类障碍距离过小
    # =============================
    def evaluate_vehicle_distance(self, ego_df, dynamic_obs_df):
        triggered = False
        trigger_time = None

        for _, ego in ego_df.iterrows():
            current_obs = dynamic_obs_df[dynamic_obs_df.timestamp == ego.timestamp]

            for _, obs in current_obs.iterrows():
                dx = obs.center_x - ego.x
                dy = obs.center_y - ego.y

                if abs(dx) < self.thresholds["vehicle_lon_min"] and abs(dy) < self.thresholds["vehicle_lat_min"]:
                    triggered = True
                    trigger_time = ego.timestamp
                    break

            if triggered:
                break

        self.add_result(
            "vehicle_distance_too_close",
            not triggered,
            "triggered" if triggered else "normal",
            f"lon<{self.thresholds['vehicle_lon_min']}, lat<{self.thresholds['vehicle_lat_min']}",
            trigger_time
        )

    # =============================
    # 10. 路径规划无结果
    # =============================
    def evaluate_path_validity(self, path_df):
        min_points = path_df.path_point_size.min()

        self.add_result(
            "path_no_result",
            min_points >= self.thresholds["path_point_min"],
            min_points,
            self.thresholds["path_point_min"]
        )

    # =============================
    # 11. 主车压实线
    # =============================
    def evaluate_lane_crossing(self, hdmap_df):
        trigger = hdmap_df[hdmap_df.cross_solid_line == True]

        self.add_result(
            "cross_solid_line",
            trigger.empty,
            len(trigger),
            0,
            None if trigger.empty else trigger.iloc[0].timestamp
        )

    # =============================
    # 12. 速度规划无结果
    # =============================
    def evaluate_speed_validity(self, ego_df, path_df):
        merged = pd.merge(ego_df, path_df, on="timestamp", suffixes=("_ego", "_path"))

        trigger = merged[
            (merged.linear_velocity.abs() < self.thresholds["ego_speed_min"]) &
            (merged.speed.abs() < self.thresholds["plan_speed_min"])
        ]

        self.add_result(
            "speed_no_result",
            trigger.empty,
            len(trigger),
            0,
            None if trigger.empty else trigger.iloc[0].timestamp
        )

    # =============================
    # 13-14 转向角
    # =============================
    def evaluate_steering(self, ego_df):
        max_angle_deg = ego_df.front_wheel_angle.abs().max()

        self.add_result(
            "front_wheel_angle_too_large",
            max_angle_deg <= self.thresholds["front_wheel_angle_max_deg"],
            round(max_angle_deg, 4),
            self.thresholds["front_wheel_angle_max_deg"]
        )

        ego_df = ego_df.sort_values("timestamp").copy()
        ego_df["dt"] = ego_df.timestamp.diff()
        ego_df["angle_rate"] = ego_df.front_wheel_angle.diff() / ego_df.dt

        max_rate = ego_df.angle_rate.abs().max()

        self.add_result(
            "front_wheel_angle_rate_too_large",
            max_rate <= self.thresholds["front_wheel_angle_rate_max_deg"],
            round(max_rate, 4),
            self.thresholds["front_wheel_angle_rate_max_deg"]
        )

    # =============================
    # 主入口
    # =============================
    def run(
        self,
        ego_csv,
        path_csv,
        static_obs_csv,
        dynamic_obs_csv,
        hdmap_csv,
        output_csv="evaluation_report.csv"
    ):
        ego_df = pd.read_csv(ego_csv)
        path_df = pd.read_csv(path_csv)
        static_obs_df = pd.read_csv(static_obs_csv)
        dynamic_obs_df = pd.read_csv(dynamic_obs_csv)
        hdmap_df = pd.read_csv(hdmap_csv)

        self.evaluate_static_collision(ego_df, static_obs_df)
        self.evaluate_dynamic_collision(ego_df, dynamic_obs_df)
        self.evaluate_planning_dynamics(path_df)
        self.evaluate_vehicle_distance(ego_df, dynamic_obs_df)
        self.evaluate_path_validity(path_df)
        self.evaluate_lane_crossing(hdmap_df)
        self.evaluate_speed_validity(ego_df, path_df)
        self.evaluate_steering(ego_df)

        result_df = pd.DataFrame(self.results)
        result_df.to_csv(output_csv, index=False, encoding="utf-8-sig")

        final_pass = result_df["passed"].all()

        print("========== Perfect Control Evaluation ==========")
        print("FINAL RESULT:", "PASS" if final_pass else "FAIL")
        print(f"Total Metrics: {len(result_df)}")
        print(f"Passed: {result_df['passed'].sum()}")
        print(f"Failed: {(~result_df['passed']).sum()}")
        print(f"Report saved to: {output_csv}")


if __name__ == "__main__":
    evaluator = PerfectControlEvaluator()

    evaluator.run(
        ego_csv="data/CHANNEL_LocalPose.csv",
        path_csv="data/CHANNEL_LocalPathInfo.csv",
        static_obs_csv="data/static_obstacle.csv",
        dynamic_obs_csv="data/CHANNEL_LpLateFusionObjectInfo.csv",
        hdmap_csv="data/CHANNEL_LocalHDMap.csv",
        output_csv="outputs/perfect_control_report.csv"
    )