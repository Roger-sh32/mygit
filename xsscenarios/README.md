# xsscenarios — 场景库

OpenSCENARIO 场景 canonical 存放目录，供 `esmini_planning` 与后续评测共用。

**Ego 车辆**：`car_gl_t6`（`Catalogs/Vehicles/VehicleCatalog.xosc`），几何/转向对齐 `Controllers/.../GL-T6/GL-T6_vehicle_params.pb.txt`。3D 模型复用 esmini `car_white.osgb` + `BBToModel`，碰撞与 OSI 尺寸以 BoundingBox 为准。

| 场景 | 地图 | 说明 |
|------|------|------|
| `AEB-test-planning.xosc` | `../xmap/straight_500m.xodr` | Target 变道/加减速序列 |
| `slow-lead-vehicle-planning.xosc` | `../xmap/straight_500m_signs.xodr` | 前方 Lead 1 m/s 慢车 |
| `cut-in_simple-planning.xosc` | `../xmap/straight_500m.xodr` | 邻道车切入本车道后急刹 |
| `alks-cut-in-planning.xosc` | `../xmap/straight_500m.xodr` | 对向车道切入 + 急刹（ALKS R157 变体） |

运行示例：

```bash
cd mygit/esmini_planning && ./run_planning.sh
# 默认 AEB 场景

./run_planning.sh --xosc ../xsscenarios/cut-in_simple-planning.xosc
./run_planning.sh --xosc ../xsscenarios/slow-lead-vehicle-planning.xosc --initial-speed-ms 15
./run_planning.sh --xosc ../xsscenarios/alks-cut-in-planning.xosc
```

或按短名（见 `assets.PLANNING_SCENARIOS`）：

```bash
./run_planning.sh --scenario cut-in
./run_planning.sh --scenario slow-lead
```

**说明**：在线 `map_builder` / `task_list_builder` 仍按 `straight_500m` 几何生成 Planning 地图 proto；`straight_500m_signs` 与 `straight_500m` 车道拓扑一致，可直接复用。弯道地图（如 `curve_r100.xodr`）已放入 `xmap/` 备用，接入需扩展 map proto。

控制器 Catalog 仍引用 `esmini-demo_Linux/.../Catalogs/Controllers/`（`UDPDriverController`）。
