# xmap — 路网资产

与 **Planning P2**（`ldmap.Map` / `FovHdMap` / `MapPosition`）及 **P4 TaskList** 几何对齐的 OpenDRIVE 文件。

| 文件 | 说明 |
|------|------|
| `straight_500m.xodr` | 直道 500 m、三车道（lane 1/0/-1），与 `map_builder.py` 中 `StraightRoadSpec` 一致；多数 planning 场景使用 |
| `straight_500m_signs.xodr` | 同上直道 + 交通标志；`slow-lead-vehicle-planning` 使用，车道拓扑与 `straight_500m` 相同 |
| `curve_r100.xodr` | R=100 m 弯道单车道（esmini `lane_change_simple` 源场景）；**尚未**接入 online map proto |

**被测对象侧**：Planning 不直接读 xodr，而是由 `osi_adapter` 按 OSI 位姿在线生成 xsproto 地图；xodr 供 **esmini** 物理仿真与 OSI 几何一致。

新增路网：放入本目录，并在 `map_builder.py` / `task_list_builder.py` 增加对应 Spec 或解析逻辑。
