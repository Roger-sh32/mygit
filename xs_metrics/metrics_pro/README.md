````markdown
# L0 Safety Evaluation Module（Perfect Control 模式）

## 一、模块定位

`l0_safety.py` 是自动驾驶仿真评测框架中的 **L0（Safety Gate）基础安全层** 指标模块。

### 核心目标：
对单个场景回放结果进行 **最低安全门槛（Gate）判断**：

> “这个场景是否发生了绝对不可接受的安全问题？”

---

## 二、适用范围

### 当前适配模式：
### **Perfect Control（完美控制模式）**

即：
- Simulator 不模拟真实控制器误差
- 不关注 throttle / brake / steer 控制品质
- Vehicle 直接理想跟踪规划轨迹
- 核心关注：
  - Planning 决策是否安全
  - 感知 / 预测输入是否导致危险结果
  - 是否发生碰撞 / 压线 / 安全距离不足

---

## 三、L0 指标定义（当前版本）

---

### 1. Collision_StaticObstacle（通用静态障碍碰撞）
### 定义：
主车外包框（ego polygon）与静态障碍区域（obstacle_areas）最小距离小于阈值。

### 默认阈值：
```python
collision_distance = 0.2m
````

### 判定：

* `< threshold` → FAIL
* `>= threshold` → PASS

### 典型场景：

* 撞路沿
* 撞锥桶
* 撞隔离带
* 撞 OCC 障碍区

---

### 2. Collision_ObjectObstacle（目标类障碍碰撞）

### 定义：

主车与动态 / 静态目标对象（obs_objs）碰撞。

### 默认阈值：

```python
collision_distance = 0.2m
```

### 典型场景：

* 撞车
* 撞人
* 撞骑行者

---

### 3. DistanceTooSmall_Vehicle（车辆距离过小）

### 定义：

主车与车辆目标纵向 / 横向边界距离同时低于阈值。

### 默认阈值：

```python
vehicle_lon_distance = 1.0m
vehicle_lat_distance = 0.5m
```

### 用途：

碰撞前高风险预警指标。

---

### 4. DistanceTooSmall_Pedestrian（行人距离过小）

### 定义：

主车与行人目标纵向 / 横向边界距离同时低于阈值。

### 默认阈值：

```python
pedestrian_lon_distance = 1.0m
pedestrian_lat_distance = 0.5m
```

### 用途：

弱势交通参与者保护。

---

### 5. SolidLineViolation（压实线）

### 定义：

ego polygon 与禁止跨越车道线（BI_FORBIDDEN）重叠。

### 数据来源：

```text
CHANNEL_LocalHDMap.future_lanemarkings.ext_points.lane_change_role
```

### 用途：

法规底线。

---

## 四、总体判定逻辑

### L0 Overall：

```python
任意一个指标 FAIL
→ Overall FAIL
否则 PASS
```

---

## 五、输入数据要求

---

### 推荐输入：

时间同步后的 frame list：

```python
frames = [
    {
        "timestamp": float,
        "local_pose": {},
        "lp_late_fusion": {},
        "local_hdmap": {}
    }
]
```

---

### 必需字段：

## 1. LocalPose

```python
x
y
yaw
```

---

## 2. LpLateFusionObjectInfo

```python
obs_objs
obstacle_areas
```

---

## 3. LocalHDMap

```python
future_lanemarkings
```

---

## 注意：

### 当前模块默认：

### 所有数据已统一到同一坐标系（建议 local frame）

否则：

* 距离错误
* 压线错误
* 误判 FAIL

---

## 六、输出结构

每个指标返回：

```python
MetricResult
```

### 示例：

```python
{
    "metric_name": "Collision_ObjectObstacle",
    "result": "FAIL",
    "triggered": True,
    "threshold": 0.2,
    "actual_value": 0.08,
    "trigger_time": 12.53,
    "related_object_id": 2947,
    "related_object_type": "VEHICLE"
}
```

---

## 七、典型接入方式

---

### Step1：

解析 DBF / MCAP / protobuf

```text
DBF / MCAP
↓
Proto Decode
↓
统一 Frame Builder（按 timestamp 对齐）
↓
frames
```

---

### Step2：

调用：

```python
results = evaluate_l0_safety(
    frames,
    vehicle_params,
    thresholds
)
```

---

### Step3：

汇总：

```python
overall = l0_overall_result(results)
```

---

## 八、车辆参数配置

```python
VehicleParams(
    imu_to_front=2.8,
    imu_to_back=1.2,
    half_width=0.9
)
```

---

### 说明：

必须根据真实车型修改。

否则：

* 碰撞距离失真
* 压线误判
* 安全距离错误

---

## 九、后续扩展建议（L1 / L2）

---

## L1（功能表现层）

建议增加：

* TTC
* Min Clearance
* Path Deviation
* Comfort（jerk）
* Lane Change Success
* Stop Accuracy

---

## L2（体验层）

建议增加：

* Smoothness
* Efficiency
* User Comfort
* Strategy Stability

---

## 十、已知限制

---

### 当前版本未覆盖：

* OCC / Freespace 缩短根因
* TLR（红绿灯）
* 控制器延迟
* 轮胎动力学
* 实车闭环控制误差
* 多帧持续时间判定（如持续压线 > x 秒）

---

## 十一、开发规范建议

---

### 推荐目录：

```text
metrics/
 ├── l0_safety.py
 ├── l1_function.py
 ├── l2_comfort.py
```

---

### 推荐原则：

### 单指标单函数：

```python
check_xxx()
```

优点：

* 易维护
* 易单测
* 易快速定位 badcase

---

## 十二、Badcase 分析建议链路

---

```text
L0 FAIL
↓
碰撞 / 压线 / 距离问题
↓
回看：
LpLateFusion
↓
LateFusion
↓
PerceptionObjectsInfo
↓
OD / OCC / Freespace
↓
Planner Path
↓
根因定位
```

---

# 十三、一句话总结

> **L0 是“先判断有没有出事”，不是“为什么出事”。**

---

# 作者备注（交接建议）

首次接手同事建议优先理解：

### 必看：

* LocalPose
* LpLateFusionObjectInfo
* LocalHDMap
* LocalPathInfo

### 暂缓：

* ROS
* CARLA
* 完整控制器
* 动力学模型

---

# Version

### v0.1

* 初版
* Perfect Control 模式
* L0 Safety Gate 五指标

```
```
