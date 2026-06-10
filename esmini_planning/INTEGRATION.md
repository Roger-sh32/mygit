# Planning 被测对象接入说明

本文档回答：**Planning 作为 SUT 是否被完整接入**——关注 channel 供给、输出消费与执行链路，不评价算法决策优劣。

## 架构一览

```
esmini (xsscenarios + xmap)
    │ OSI GroundTruth
    ▼
osi_adapter  ──► PlanningInput (xsproto bytes + E2E)
    │
    ▼
PlanningWrapper (release_x86-planning / libplanning_base.so)
    │
    ▼
PlanningOutput.trajectory
    │
    ▼
control_adapter ──► UDP driverInput ──► esmini Ego
```

**非实车路径**：未接 `CHANNEL_LocalPathInfo` → 实车 Control 模块；当前用 Python 读 wrapper 轨迹 + OSI TTC 转 UDP。

---

## 输入：PlanningWrapper 需要的 16 路

| # | PlanningInput 字段 | xsproto / 数据 | AEB 直道场景 | 来源 |
|---|-------------------|----------------|-------------|------|
| 1 | `local_pose_bytes` | LocalPose | ✅ 已供 | OSI ego → `xsproto_builder` |
| 2 | `vehicle_status_bytes` | VehicleStatus | ✅ 已供 | OSI 速度 + AUTO/GEAR_D |
| 3 | `ld_map_bytes` | ldmap.Map | ✅ 已供 | `map_builder` 在线生成（对齐 `xmap/straight_500m.xodr`） |
| 4 | `e2e_trajectory_points` | 参考线 dict | ✅ 已供 | 沿航向匀速点列 |
| 5 | `prediction_bytes` | TrajPrediction | ✅ 已供 | OSI 他车 → CV 预测 |
| 6 | `task_list_bytes` | TaskList | ✅ 已供 | `task_list_builder` 沿车道 forward 路由 |
| 7 | `ultrasonic_data_bytes` | UltraSonicData | ⬜ 空 | AEB 场景未用 |
| 8 | `remote_operate_bytes` | RemoteOperate | ⬜ 空 | 未用 |
| 9 | `remote_cooperate_bytes` | RemoteCooperate | ⬜ 空 | 未用 |
| 10 | `perception_object_info_bytes` | PerceptionObjectInfo | ✅ 已供 | OSI 他车（与 prediction 配对才出 obstacles） |
| 11 | `traffic_light_info_bytes` | TrafficLightInfo | ⬜ 空 | 无信号灯场景 |
| 12 | `rubbish_info_bytes` | RubbishInfo | ⬜ 空 | 未用 |
| 13 | `curb_info_bytes` | CurbInfoMsg | ⬜ 空 | 未用 |
| 14 | `traffic_sign_info_bytes` | TrafficSignInfo | ⬜ 空 | 未用 |
| 15 | `fovhdmap_bytes` | FovHdMap | ✅ 已供 | `map_builder` |
| 16 | `map_position_bytes` | MapPosition | ✅ 已供 | `map_builder` |

**结论（输入）**：当前 AEB / straight_500m 场景下，Planning **运行所必需**的位姿、地图、感知、预测、路由 channel **均已供给**；其余为空在直道 AEB 测试中可接受。若测红绿灯、超声、远程驾驶等场景，需补对应 builder。

C++ 验收（`local_planner/*.log`）：

- `odometry timestamp > 0`、`obstacles size 1`、`final lane size : 3`
- 无 `NAVIGATION_ERROR` / `all_lane_seqs is nullptr`（P4 后）

---

## 输出：Planning 产物谁在用

| Planning 输出 | 实车 channel | 当前 esmini 闭环 | 状态 |
|---------------|-------------|-----------------|------|
| `trajectory[]` (v/a/t/x/y/heading) | LocalPathInfo.path | `control_adapter` 读 wrapper 结果 | ✅ 已消费（非 proto 通道） |
| `vehicle_command` ES/ST | LocalPathInfo.vehicle_command | 未读 | ⬜ 未接 |
| `control_steer_angle` | 内部 / Control | 未读；转向用轨迹 heading 误差 | ⚠️ 部分（启发式） |
| vis_data / debug | 可视化 | 未用 | ⬜ 可选 |

**结论（输出）**：Planning **有稳定轨迹输出**并被 Python 控制层读取；**未**按实车方式订阅 `LocalPathInfo` proto，也**未**接正式 Control.so。

---

## 执行：输出是否作用到仿真车

| 环节 | 实现 | 状态 |
|------|------|------|
| bootstrap | `stateXYH` 注入初速 | ✅ |
| planning 阶段 | UDP `driverInput` throttle/brake/steer | ✅ |
| 制动逻辑 | 轨迹 v/a/t 多窗口 + 同车道/邻道 TTC 兜底（`max_brake=1.0`，`max_decel=8 m/s²`） | ✅（2026-06-10 加强） |
| 控制日志 | `ctrl ... brk=...` + 摘要最小 TTC / 最大 brake | ✅ |
| 上一帧轨迹回灌 | 实车 LocalPathInfo 闭环；仿真无 | ⚠️ 偶发 `trajectory_stitch` 失败 |

**结论（执行）**：Planning 输出**能驱动 esmini**，但执行器是 **UDP 启发式**，不是实车 Control；stitch 与 OSI 实车状态不一致时属**集成层**问题，不是 Planning 算法本身。

---

## 资产目录（路网 / 场景 / 被测库）

| 目录 | 内容 |
|------|------|
| `mygit/xmap/` | OpenDRIVE：`straight_500m.xodr`、`straight_500m_signs.xodr`、`curve_r100.xodr`（弯道未接 P2） |
| `mygit/xsscenarios/` | Planning 场景库 + `Catalogs/Vehicles/VehicleCatalog.xosc`（`car_gl_t6`、`car_red`） |
| `mygit/Controllers/release_x86-planning/` | 被测 Planning 二进制与参数（本地部署，GL-T6 车参见 `bin/parameters/planner/GL-T6/`） |
| `mygit/esmini_planning/` | 适配器 + 闭环脚本 |

### Planning 场景一览

| `--scenario` | xosc | 地图 | 说明 |
|--------------|------|------|------|
| `aeb` | `AEB-test-planning.xosc` | straight_500m | Target 变道/加减速 |
| `slow-lead` | `slow-lead-vehicle-planning.xosc` | straight_500m_signs | 前方 1 m/s 慢车 |
| `cut-in` | `cut-in_simple-planning.xosc` | straight_500m | 邻道切入 + 急刹 |
| `alks-cut-in` | `alks-cut-in-planning.xosc` | straight_500m | 对向车道切入 + 急刹 |

Ego 均为 **UDP synchronous + `car_gl_t6`**（3.69×1.4 m，maxSteering 0.551 rad，MaxDeceleration 8 m/s²）。

路径常量：`esmini_planning/assets.py`（`PLANNING_SCENARIOS`、`DEFAULT_EGO_VEHICLE`）。

---

## 仍可忽略的 vs 建议跟进的

**可视为 Planning 算法行为（测试价值所在，可忽略）**

- multi-task 变道分支 `Unsafe to initiate lane change to lane_-1`（每帧噪声，主任务仍 OK）
- 具体减速度、是否撞 Target、选道策略

**建议作为集成改进（非算法）**

1. ~~会话日志增加 `brake/throttle/source`~~ → **已完成**（2026-06-10）
2. 实车对标：接 `LocalPathInfo` + Control，或至少使用 C++ `control_steer_angle` 执行横向避障
3. 降低 stitch 失败：planning 阶段速度与 OSI 更一致，或回灌上一帧轨迹给 wrapper
4. 扩展 P2/P4：`curve_r100` 等弯道 xodr 的 online map / TaskList Spec

---

## 快速自检

```bash
cd mygit/esmini_planning && ./run_planning.sh
```

Python 日志应含：`[OK] P1…P4`、`obs=1`、`task_pts>0`、`产出非空轨迹次数 ≈ PlanningWrapper.run 次数`。

C++ 日志应无：`NAVIGATION_ERROR`、`Cant get nearest lane`（P2 后）。
