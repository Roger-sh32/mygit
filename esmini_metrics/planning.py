"""
planning.py esmini仿真规划指标评估器
---------------------------------------------------
覆盖指标：
1. 规划速度过大
2. 规划速度过小
3. 规划加速度过大
4. 规划加速度过小
5. 规划加加速度（Jerk）过大
6. 规划加加速度（Jerk）过小
"""

import pandas as pd
from shapely.geometry import Polygon, Point
from shapely.ops import nearest_points

class PlanningEvaluator:
    def __init__(self):
        self.thresholds = {
            "speed_max": 12.0,
            "speed_min": 1.0,
            "acc_max": 2.0,
            "acc_min": -2.0,
            "jerk_max": 2.0,
            "jerk_min": -2.0,
        }

        self.results = []

    def add_result(self, name, passed, value, threshold, timestamp=None, detail=""):
        result = {
            "metric_name": name,
            "result": "PASS" if passed else "FAIL",
            "triggered": passed,
            "threshold": threshold,
            "actual_value": value,
            "trigger_time": timestamp,
            "detail": detail,
        }
        self.results.append(result)
        return result

    def evaluate_speed(self, path_df):
        max_speed = path_df.speed.max()
        min_speed = path_df.speed.min()

        self.add_result("speed_too_high", max_speed > self.thresholds["speed_max"], max_speed, self.thresholds["speed_max"])
        self.add_result("speed_too_low", min_speed < self.thresholds["speed_min"], min_speed, self.thresholds["speed_min"])

    def evaluate_acc(self, path_df):
        max_acc = path_df.acceleration.max()
        min_acc = path_df.acceleration.min()
        self.add_result("acc_too_high", max_acc > self.thresholds["acc_max"], max_acc, self.thresholds["acc_max"])
        self.add_result("acc_too_low", min_acc < self.thresholds["acc_min"], min_acc, self.thresholds["acc_min"])

    def evaluate_jerk(self, path_df):
        max_jerk = path_df.jerk.max()
        min_jerk = path_df.jerk.min()
        self.add_result("jerk_too_high", max_jerk > self.thresholds["jerk_max"], max_jerk, self.thresholds["jerk_max"])
        self.add_result("jerk_too_low", min_jerk < self.thresholds["jerk_min"], min_jerk, self.thresholds["jerk_min"])

    def evaluate_planning(self, path_df):
        self.evaluate_speed(path_df)
        self.evaluate_acc(path_df)
        self.evaluate_jerk(path_df)

    def run(self, path_csv):
        path_df = pd.read_csv(path_csv)
        self.evaluate_planning(path_df)
        return self.results

if __name__ == "__main__":
    evaluator = PlanningEvaluator()
    results = evaluator.run("/home/xs/Sunhan/Program_files/esmini-demo_Linux/esmini-demo/run/esmini/log.csv")
    print(results)                     # 打印结果                                                       
    print(f"Total Metrics: {len(results)}")
    print(f"Passed: {sum(result['result'] == 'PASS' for result in results)}")
    print(f"Failed: {sum(result['result'] == 'FAIL' for result in results)}")  
    print(f"Warning: {sum(result['result'] == 'WARNING' for result in results)}")  
    print(f"Error: {sum(result['result'] == 'ERROR' for result in results)}")  
    print(f"Info: {sum(result['result'] == 'INFO' for result in results)}")  
    print(f"Debug: {sum(result['result'] == 'DEBUG' for result in results)}")  
    print(f"Trace: {sum(result['result'] == 'TRACE' for result in results)}")  
    print(f"Fatal: {sum(result['result'] == 'FATAL' for result in results)}")  
    print(f"Unknown: {sum(result['result'] == 'UNKNOWN' for result in results)}")  
    print(f"Total: {len(results)}")
    print(f"Passed: {sum(result['result'] == 'PASS' for result in results)}")
    print(f"Failed: {sum(result['result'] == 'FAIL' for result in results)}")  
    print(f"Warning: {sum(result['result'] == 'WARNING' for result in results)}")  
    print(f"Error: {sum(result['result'] == 'ERROR' for result in results)}")  
    print(f"Info: {sum(result['result'] == 'INFO' for result in results)}")  