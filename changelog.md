# Changelog

本文件记录 `mygit` 仓库的重要变更。格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/)。

---

## [Unreleased]

### Added (2026-06-10)
- **xmap/**：`straight_500m`、`straight_500m_signs`、`curve_r100`（弯道 P2 未接）。
- **xsscenarios/**：Planning 场景库（AEB / slow-lead / cut-in / alks-cut-in）+ `car_gl_t6` VehicleCatalog（对齐 GL-T6 车参）。
- **esmini_planning/** 完整接入：P0–P4（OSI → xsproto → PlanningWrapper → UDP）、`INTEGRATION.md` channel 矩阵。
- 控制层 v2：`max_brake=1.0`、`max_decel_ms2=8`、同车道 + 邻道 TTC 兜底；会话日志 `ctrl ... brk=...` 与摘要最小 TTC / 最大 brake。
- `esmini_run.py --scenario` 短名切换场景；`assets.PLANNING_SCENARIOS` 注册表。

### Changed (2026-06-10)
- `control_adapter.py`：由轻制动启发式（max 0.55）改为轨迹 v/a 跟踪 + 紧急制动模式。
- `osi_adapter.py`：TTC `vehicle_margin` 3.0 m（GL-T6）；新增 `estimate_threat_ttc`（邻道切入）。
- 更新 `README.md` / `esmini_planning/README.md` / `INTEGRATION.md` 现状与已知限制。

### Added
- P1：`xsproto_builder.py` + `osi_adapter` 注入 `LocalPose` / `VehicleStatus`。
- `esmini_planning/`：独立 Planning 接入骨架（OSI 适配、控制回写、`run_planning.sh`），与 `esmini_operator` 隔离。
- `esmini_run.py --mode aeb`：单终端一键启动 esmini + `Controllers/AEB_controller.py`。

### Changed
- 更新 `README.md`：补充已验证的一键运行命令、`sim_eval` 环境说明与当前默认路径。

### Added (2026-06-09)
- 新增 `README.md`：汇总 AEB 控制链路、目录结构、运行方式与场景说明。
- 新增 `changelog.md`：记录仓库变更历史。

---

## [2026-05-12] — 测试运行与提交 accfa5c

### Changed
- 提交信息 `111`（accfa5c），与此前 AEB/评测相关改动一并合入。

---

## [2026-05-09] — AEB 控制器与评测指标 faf5bb5

### Added
- `Controllers/AEB_controller.py`：基于 OSI GroundTruth 计算 TTC，经 UDP `driverInput` / `stateXYH` 控制 Ego；含 bootstrap 初速注入。
- `Controllers/Send_udp_driver_control.py`：最小 UDP 发送示例（端口 53995）。
- `log.txt`：esmini 仿真运行日志。
- `xs_metrics/metrics_normal/perfect_control_evaluator_v1.py`：Perfect Control 模式 14 项规控指标。
- `xs_metrics/metrics_pro/L0_safety.py` 与 `metrics_pro/README.md`：L0 安全门控指标模块。

### Changed
- `esmini-demo_Linux/.../resources/xosc/AEB-test.xosc`：由 acc-test 改编的 AEB 测试场景（Ego=UDPDriver，Target 脚本变道/制动）。

### Verified
- `esmini_operator/test_runs/20260512_200118/`：AEB 触发成功，无碰撞，`final_result: PASS`；HTML 报告已生成。

---

## [2026-05-08] — esmini 启动器重构 b6932b4

### Changed
- `esmini_operator/esmini_run.py`：重构为播放 / 测试双模式；测试模式支持自动起 esmini、跑算法、解析评测 JSON 并生成 HTML 报告。

---

## [2026-04-30] — 引入 esmini 预编译包 234cc20

### Added
- `esmini-demo_Linux/`：esmini v3.0.3 预编译包（bin、resources、udp_driver 脚本等）。

---

## [2023-06-06 ~ 2023-06-07] — 仓库初始化

### Added
- `esmini_operator/` 初始脚本（`esmini_run.py`、`odrview.py`、`replay.py` 等）。
- Git 仓库首次提交（b2dc61b → 03ce5ed → e21cc72）。

---

## 后续变更请在此追加

每次修改建议按以下模板追加到 `[Unreleased]` 或新建日期区块：

```markdown
## [YYYY-MM-DD] — 简短标题

### Added / Changed / Fixed / Removed
- 变更说明
```
