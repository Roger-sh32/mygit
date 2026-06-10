# esmini_planning — 正式 Planning 接入（与 esmini_operator 隔离）

本目录用于 **esmini OSI ↔ PlanningWrapper ↔ UDP** 闭环，不影响 `esmini_operator/` 的最小 TTC-AEB 验证通路。

## 快速开始

```bash
cd /home/xs/桌面/git/mygit/esmini_planning
chmod +x run_planning.sh
./run_planning.sh
```

或指定初速（默认 **20 m/s**，AEB 场景比 33.33 更安全）：

```bash
./run_planning.sh --initial-speed-ms 20
/usr/bin/python3.10 esmini_planning/esmini_run.py --initial-speed-ms 20
```

默认场景为 **`xsscenarios/AEB-test-planning.xosc`**（Ego=`car_gl_t6`，路网 **`xmap/straight_500m.xodr`**）。

更多场景见 `--scenario aeb|slow-lead|cut-in|alks-cut-in`（`assets.PLANNING_SCENARIOS`）。

详细 channel 接入矩阵见 **[INTEGRATION.md](./INTEGRATION.md)**（Planning 作为被测对象的输入/输出/执行评估）。

## mygit 资产布局

```
mygit/
├── xmap/                    # 路网（OpenDRIVE）
│   ├── straight_500m.xodr
│   ├── straight_500m_signs.xodr
│   └── curve_r100.xodr      # 备用，P2 未接
├── xsscenarios/             # 场景库 + car_gl_t6 VehicleCatalog
│   ├── AEB-test-planning.xosc
│   ├── cut-in_simple-planning.xosc
│   ├── slow-lead-vehicle-planning.xosc
│   └── alks-cut-in-planning.xosc
├── Controllers/release_x86-planning/   # 被测 Planning 库（本地，需自备）
└── esmini_planning/         # 适配器 + 闭环（本目录）
```

路径常量：`esmini_planning/assets.py`。

## 环境要求

| 项目 | 说明 |
|------|------|
| Python | **3.10**（`libpdv_planning.so` 编译版本） |
| **xs_proto** | P1 必需：`base/local_pose_pb2.py`（默认 `~/.local/lib/python3.10/site-packages/xs_proto`） |
| esmini | 同 mygit 预编译包，须带 OSI |
| Planning 包 | `Controllers/release_x86-planning/` |

可选：`export XS_PROTO_PYTHON=/path/to/xs_proto`

`esmini_operator` + `sim_eval` 仍用于最小 AEB 闭环；本目录单独用 **python3.10**。

## 目录说明

```
esmini_planning/
├── assets.py               # xmap / xsscenarios / Planning 路径
├── INTEGRATION.md          # Planning 被测对象 channel 接入说明
├── env_setup.py            # LD_LIBRARY_PATH、PDV 懒加载
├── xsproto_builder.py      # LocalPose / VehicleStatus / TrajPrediction / PerceptionObjectInfo
├── map_builder.py          # straight_500m → xsproto.ldmap.Map / FovHdMap / MapPosition（P2）
├── task_list_builder.py    # straight_500m → xsproto.globalpath.TaskList（P4）
├── osi_adapter.py          # OSI → PlanningInput；OSI 前方 TTC 估算
├── control_adapter.py      # PlanningOutput 轨迹 → UDP driverInput（制动/巡航）
├── planning_output_util.py # 从 wrapper 结果提取轨迹（estop 时仍可读）
├── planning_controller.py  # 主循环
├── session_log.py          # Python 会话日志
├── esmini_run.py           # 一键起 esmini + 控制器
├── run_planning.sh         # 推荐入口
└── logs/                   # planning_session_*.log（Python）；C++ glog 见下表
```

## 当前进度

- [x] **P0**：OSI → E2E → PlanningWrapper → UDP
- [x] **P1**：`local_pose_bytes` + `vehicle_status_bytes`（档位 D、AUTO 模式）
- [x] **P1.5**：`prediction_bytes`（TrajPrediction，时间戳与 LocalPose 对齐）
- [x] **P2**：`ld_map_bytes`（`xsproto.ldmap.Map`）+ `fovhdmap_bytes` + `map_position_bytes`（`straight_500m.xodr` 直道三车道）
- [x] **P3**：OSI 障碍车 → `TrajPrediction` + **`PerceptionObjectInfo`**（二者缺一不可，C++ `PredictionAdapter` 才会产生 obstacles）
- [x] **P4**：`task_list_bytes`（`TaskList` 沿当前车道向前路由，对齐 `straight_500m` / `ldmap.Map`）
- [x] **场景库 / 车模**：`car_gl_t6`（对齐 GL-T6 车参）+ 4 个 planning 场景（见下）
- [x] **控制层 v2**：轨迹 v/a 跟踪 + 同车道/邻道 TTC 兜底，`max_brake=1.0`、`max_decel=8 m/s²`；会话日志含 `ctrl ... brk=...`
- [ ] **实车 Control**：暂不接；横向仍仅 heading 误差 steer，无 Planning 变道执行
- [ ] **弯道地图**：`xmap/curve_r100.xodr` 已备，online map proto 未扩展

### 场景与车模（2026-06-10）

| 短名 | 场景 | 地图 | Ego |
|------|------|------|-----|
| `aeb` | `xsscenarios/AEB-test-planning.xosc` | `straight_500m` | `car_gl_t6` |
| `slow-lead` | `slow-lead-vehicle-planning.xosc` | `straight_500m_signs` | `car_gl_t6` |
| `cut-in` | `cut-in_simple-planning.xosc` | `straight_500m` | `car_gl_t6` |
| `alks-cut-in` | `alks-cut-in-planning.xosc` | `straight_500m` | `car_gl_t6` |

```bash
./run_planning.sh --scenario cut-in
./run_planning.sh --scenario slow-lead --initial-speed-ms 15
```

车模定义：`xsscenarios/Catalogs/Vehicles/VehicleCatalog.xosc`（3.69×1.4 m，轴距 2.4 m，maxSteering 0.551 rad，MaxDeceleration 8 m/s²）。

### 已知限制

| 现象 | 原因 | 后续 |
|------|------|------|
| 减速度曾偏小 | 旧版 `max_brake=0.55` + 制动渐变过慢 | 已改 v2；仍非实车 Control |
| 刹不停 / 撞 Target | Python 启发式执行，未跟 LocalPathInfo | 接 Control 或轨迹速度闭环 |
| 无横向避障 | 未读规划横向轨迹 / 变道指令 | 接 `control_steer_angle` 或 LocalPathInfo |
| `trajectory_stitch` 偶发失败 | 无上一帧 LocalPathInfo 回灌 | 集成层改进 |
| C++ 每帧 `Unsafe lane change to lane_-1` | multi-task 噪声，主任务仍 OK | 可忽略（算法行为） |

P2/P4 在线地图仍按 **`straight_500m` 直道 Spec** 生成；`straight_500m_signs` 车道拓扑相同可直接跑。

### 各阶段验收要点

| 阶段 | Python 日志 | C++ 日志 |
|------|-------------|----------|
| P1 | `[OK] P1 xsproto ...` | `odometry timestamp > 0` |
| P2 | `[OK] P2 straight_500m ...` | `final lane size : 3`、`map_activate_state = 1`，不再 `Cant get nearest lane` |
| P3 | `obs=1`、`traj_pts>0` | `obstacles size 1`（非全程 0）、`final traj point` 含减速 `v/a` |
| P4 | `task_pts>0`（约 70+） | 不再 `NAVIGATION_ERROR` / `all_lane_seqs is nullptr`；`scene reasoning` 正常 |

**P3 关键结论**：仅发 `TrajPrediction` 不够，必须同时发 `PerceptionObjectInfo`，C++ 才会识别障碍并规划减速轨迹。

**P4 关键结论**：`TaskList.points` 坐标为 **LocalPose 系厘米**（`x_m * 100`），沿 ego 最近车道中心每 5 m 采样至路端；需配合 P2 地图的 `road_identifiers=[1]` 与 `follow_type=LANE`。

## 控制层（`control_adapter.py`）

Planning 阶段**不用** `stateXYH`；bootstrap 40 帧用 `stateXYH` 注入初速，之后只用 `driverInput`。

制动依据 **PlanningWrapper `trajectory`（v/a/t）+ OSI TTC 兜底**（同车道 + 邻道威胁取 min TTC）：

| 参数 | 默认 | 说明 |
|------|------|------|
| `max_brake` | 1.0 | UDP 踏板上限，对齐 esmini `MaxDeceleration` |
| `max_decel_ms2` | 8.0 | GL-T6 / `car_gl_t6` catalog |
| `ttc_threshold` | 2.0 s | 低于此值加强制动 |
| `vehicle_margin` | 3.0 m | TTC 净距（GL-T6 前悬 ~2.92 m） |

- 0.5 s / 1.5 s / 5 s 多窗口取轨迹 min(v) 与 a；`target_v<1` 或 TTC<1.2 s 时 **紧急模式**（无渐变上限）
- 会话日志输出 `ctrl ... brk=... | brake:emergency/traj/ttc ...`

CLI：`--max-brake 1.0 --max-decel-ms2 8 --ttc-threshold 2.0`

未使用：`LocalPathInfo.vehicle_command`、实车 `Control.so`（横向仍仅 heading 误差 steer）。

## 日志在哪里？

| 类型 | 路径 |
|------|------|
| **Python 会话日志**（推荐看这个） | `esmini_planning/logs/planning_session_*.log` |
| **Planning C++ 详细日志** | `Controllers/release_x86-planning/log/local_planner/*.log` |

`run_planning.sh` 会把 `XS_LOG_PATH` 指到 `release_x86-planning/log/`；`esmini_planning/logs/` 存 Python 会话摘要。

### 如何确认 planning 已接上？

跑 `./run_planning.sh` 后，在会话日志或终端里应看到：

```
[OK] PlanningWrapper 已初始化 — planning 库已接入
[OK] P3 PerceptionObjectInfo + TrajPrediction 障碍车融合已启用
[OK] P4 TaskList 直道路由（straight_500m lane_seq）已启用
[OK] OSI 监听 48198 / UDP 发送 53995
planning.run #1 frame=40 ... traj_pts=79 obs=1 task_pts=71
```

会话结束摘要：`产出非空轨迹次数` 应接近 `PlanningWrapper.run 调用次数`；另含 **最小 TTC**、**最大 brake 指令**。

制动阶段日志示例：

```
ctrl frame=120 v=18.2 thr=0.00 brk=0.85 steer=0.02 | brake:emergency v=18.2 tgt=5.0 a=-3.2 min_v=4.1 b=0.85 ttc=1.35
```

- C++ `obstacles size > 0` → **P3 感知生效**
- C++ `final traj point` 在 t=2~4s 出现 `v` 下降、`a<0` → **规划减速轨迹生效**
- C++ 无 `NAVIGATION_ERROR`、`all_lane_seqs is nullptr` → **P4 TaskList 路由生效**

## 与 esmini_operator 对比

| | esmini_operator | esmini_planning |
|--|-----------------|-----------------|
| 控制器 | `Controllers/AEB_controller.py` | `planning_controller.py` |
| 算法 | TTC 规则 AEB | 正式 PlanningWrapper |
| 制动依据 | OSI TTC | 规划轨迹 v/a/t + OSI TTC 兜底 |
| Python | sim_eval 3.11 | **3.10** |
| 用途 | 最小闭环验证 | Planning 接入开发 |
