#!/usr/bin/env python3
"""
esmini + 正式 Planning 闭环控制器（与 esmini_operator / AEB_controller 隔离）。

P0：OSI → PlanningInput(E2E) → PlanningWrapper → UDP
P1/P2：local_pose、感知、地图 proto（见 osi_adapter.py 注释）

依赖：
  - Python 3.10（libpdv_planning 编译版本）
  - protobuf==3.20.2（若用 3.10 系统解释器需自行安装）
  - esmini 带 --osi_receiver_ip
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import sys
from socket import timeout

_MODULE_DIR = os.path.dirname(os.path.abspath(__file__))
_MYGIT_ROOT = os.path.dirname(_MODULE_DIR)

if _MODULE_DIR not in sys.path:
    sys.path.insert(0, _MODULE_DIR)

from control_adapter import PlanningControlAdapter  # noqa: E402
from env_setup import (  # noqa: E402
    PLANNING_LIB,
    apply_runtime_env,
    load_pdv_planning,
)
from session_log import SessionLog  # noqa: E402
from osi_adapter import (
    OsminiPlanningAdapter,
    estimate_forward_ttc,
    estimate_threat_ttc,
    extract_ego,
    osi_timestamp,
)
from planning_output_util import trajectory_point_count  # noqa: E402

_UDP_DRIVER_DIR = os.path.join(
    _MYGIT_ROOT,
    "esmini-demo_Linux",
    "esmini-demo",
    "scripts",
    "udp_driver",
)
if os.path.isdir(_UDP_DRIVER_DIR) and _UDP_DRIVER_DIR not in sys.path:
    sys.path.insert(0, _UDP_DRIVER_DIR)

from udp_osi_common import OSIReceiver, UdpSender, base_port, input_modes  # noqa: E402

_frame_number = 0


def _longitudinal_speed(vx: float, vy: float, yaw: float) -> float:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return vx * c + vy * s


def send_driver_input(udp_sender, throttle, brake, steering_angle, object_id=0):
    global _frame_number
    udp_sender.send(
        struct.pack(
            "iiiiddd",
            1,
            input_modes["driverInput"],
            object_id,
            _frame_number,
            throttle,
            brake,
            -steering_angle,
        )
    )
    _frame_number += 1


def send_state_xyh(
    udp_sender,
    x,
    y,
    h,
    speed_ms,
    steering_angle=0.0,
    object_id=0,
):
    global _frame_number
    udp_sender.send(
        struct.pack(
            "iiiidddddB",
            1,
            input_modes["stateXYH"],
            object_id,
            _frame_number,
            float(x),
            float(y),
            float(h),
            float(speed_ms),
            float(-steering_angle),
            int(0) & 0xFF,
        )
    )
    _frame_number += 1


def apply_command(udp_sender, cmd, object_id):
    if cmd.use_state_xyh:
        send_state_xyh(
            udp_sender,
            cmd.x,
            cmd.y,
            cmd.heading,
            cmd.speed_ms,
            steering_angle=cmd.steering_angle,
            object_id=object_id,
        )
    else:
        send_driver_input(
            udp_sender,
            cmd.throttle,
            cmd.brake,
            cmd.steering_angle,
            object_id=object_id,
        )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="esmini Planning 闭环（OSI → PlanningWrapper → UDP）"
    )
    parser.add_argument("--ip", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--object-id", type=int, default=0)
    parser.add_argument("--max-loops", type=int, default=4000)
    parser.add_argument("--plan-every", type=int, default=1, help="每 N 帧跑一次 planning")
    parser.add_argument("--bootstrap-frames", type=int, default=40)
    parser.add_argument(
        "--initial-speed-ms",
        type=float,
        default=20.0,
        help="bootstrap 注入初速 (m/s)，默认 20≈72km/h（原 33.33 易追尾 AEB 红车）",
    )
    parser.add_argument(
        "--planning-lib",
        default=PLANNING_LIB,
        help="libplanning_base.so 路径",
    )
    parser.add_argument(
        "--max-brake",
        type=float,
        default=1.0,
        help="UDP 制动踏板上限 0~1（对齐 esmini MaxDeceleration）",
    )
    parser.add_argument(
        "--max-decel-ms2",
        type=float,
        default=8.0,
        help="轨迹跟踪参考最大减速度 m/s²（GL-T6 / car_gl_t6 catalog=8）",
    )
    parser.add_argument(
        "--ttc-threshold",
        type=float,
        default=2.0,
        help="TTC 制动阈值（秒）",
    )
    args = parser.parse_args(argv)

    apply_runtime_env()
    log = SessionLog()
    PlanningInput, _, PlanningWrapper, _create_test = load_pdv_planning()

    log.line("Planning 闭环控制器启动")
    log.line("  planning lib: {}".format(args.planning_lib))
    log.line(
        "  UDP port: {}".format(
            args.port if args.port is not None else base_port + args.object_id
        )
    )
    log.line(
        "  bootstrap: {} frames @ {} m/s（planning 阶段 driverInput + 轨迹/TTC 制动）".format(
            args.bootstrap_frames, args.initial_speed_ms
        )
    )
    log.line(
        "  控制: max_brake={:.2f} max_decel={:.1f}m/s² ttc_threshold={:.1f}s".format(
            args.max_brake, args.max_decel_ms2, args.ttc_threshold
        )
    )

    wrapper = PlanningWrapper(args.planning_lib)
    if not wrapper.init():
        log.line("错误: PlanningWrapper.init() 失败: {}".format(wrapper.get_last_error()))
        log.close()
        return 1

    log.line("[OK] PlanningWrapper 已初始化 — planning 库已接入")

    adapter = OsminiPlanningAdapter(PlanningInput, default_speed_ms=args.initial_speed_ms)
    control = PlanningControlAdapter(
        cruise_speed_ms=args.initial_speed_ms,
        max_brake=args.max_brake,
        max_decel_ms2=args.max_decel_ms2,
        ttc_brake_threshold_s=args.ttc_threshold,
    )
    log.line("[OK] P1 xsproto LocalPose + VehicleStatus 适配器已加载")
    log.line("[OK] P1.5 TrajPrediction 时间戳对齐已启用")
    log.line("[OK] P2 straight_500m 地图 proto（ldmap.Map + FovHdMap + MapPosition）")
    log.line("[OK] P3 PerceptionObjectInfo + TrajPrediction 障碍车融合已启用")
    log.line("[OK] P4 TaskList 直道路由（straight_500m lane_seq）已启用")
    obj_port = args.port if args.port is not None else base_port + args.object_id

    udp_sender = UdpSender(ip=args.ip, port=obj_port)
    osi_receiver = OSIReceiver()
    osi_receiver.udp_receiver.sock.settimeout(2.0)
    log.line("[OK] OSI 监听 48198 / UDP 发送 {}".format(obj_port))

    last_output = None
    counter = 0
    plan_calls = 0
    traj_nonempty = 0
    first_plan_logged = False
    max_brake_applied = 0.0
    min_ttc_seen = math.inf
    rc = 0

    try:
        while counter < args.max_loops:
            try:
                msg = osi_receiver.receive()
            except timeout:
                log.line(
                    "OSI recv timeout — esmini 可视化窗口可能已关闭/崩溃，或仿真已停（2s 无 OSI）"
                )
                break

            ego = extract_ego(msg, args.object_id)
            if ego is None:
                counter += 1
                continue

            ex = ego.base.position.x
            ey = ego.base.position.y
            eh = ego.base.orientation.yaw
            ev = _longitudinal_speed(
                ego.base.velocity.x, ego.base.velocity.y, eh
            )

            if counter < args.bootstrap_frames:
                adapter.feed_osi(
                    msg,
                    object_id=args.object_id,
                    override_speed_ms=args.initial_speed_ms,
                )
                cmd = control.bootstrap_command(ex, ey, eh, args.initial_speed_ms)
                apply_command(udp_sender, cmd, args.object_id)
                if counter % 10 == 0:
                    log.line(
                        "bootstrap {}/{}  esmini pos=({:.1f},{:.1f})".format(
                            counter + 1, args.bootstrap_frames, ex, ey
                        )
                    )
            else:
                if counter % args.plan_every == 0:
                    inp = adapter.build_from_osi(
                        msg,
                        object_id=args.object_id,
                        max_speed_ms=args.initial_speed_ms,
                    )
                    if inp is not None:
                        last_output = wrapper.run(inp)
                        plan_calls += 1
                        n = trajectory_point_count(last_output)
                        if n > 0:
                            traj_nonempty += 1
                        if not first_plan_logged or counter % 100 == 0:
                            first_plan_logged = True
                            log.line(
                                "planning.run #{} frame={} osi_ts={:.2f}s "
                                "wrapper.success={} traj_pts={} obs={} task_pts={}".format(
                                    plan_calls,
                                    counter,
                                    osi_timestamp(msg),
                                    getattr(last_output, "success", False),
                                    n,
                                    adapter.last_obstacle_count,
                                    adapter.last_task_point_count,
                                )
                            )

                ttc_lane = estimate_forward_ttc(msg, ego, args.object_id)
                ttc_threat = estimate_threat_ttc(msg, ego, args.object_id)
                ttc = min(ttc_lane, ttc_threat)
                if ttc < min_ttc_seen:
                    min_ttc_seen = ttc
                cmd = control.from_planning_output(
                    last_output, ex, ey, eh, ev, ttc=ttc
                )
                apply_command(udp_sender, cmd, args.object_id)
                if cmd.brake > max_brake_applied:
                    max_brake_applied = cmd.brake
                if cmd.brake > 0.05:
                    if counter % 10 == 0:
                        log.line(
                            "ctrl frame={} v={:.1f} thr={:.2f} brk={:.2f} steer={:.2f} | {}".format(
                                counter,
                                ev,
                                cmd.throttle,
                                cmd.brake,
                                cmd.steering_angle,
                                cmd.source,
                            )
                        )
                elif counter % 100 == 0:
                    log.line(
                        "ctrl frame={} v={:.1f} cruise thr={:.2f} | {}".format(
                            counter, ev, cmd.throttle, cmd.source
                        )
                    )

            counter += 1
    except KeyboardInterrupt:
        log.line("用户 Ctrl+C 中断")
        rc = 130
    finally:
        udp_sender.close()
        osi_receiver.close()
        log.line("--- 会话摘要 ---")
        log.line("总 OSI 帧: {}".format(counter))
        log.line("PlanningWrapper.run 调用次数: {}".format(plan_calls))
        log.line("产出非空轨迹次数: {}".format(traj_nonempty))
        if min_ttc_seen < 900:
            log.line("最小 TTC: {:.2f}s".format(min_ttc_seen))
        log.line("最大 brake 指令: {:.2f}".format(max_brake_applied))
        if plan_calls == 0:
            log.line("[WARN] 未进入 planning 阶段（bootstrap 未完成或 OSI 无数据）")
        elif traj_nonempty == 0:
            log.line(
                "[WARN] planning 已调用但未产出轨迹 — 检查 C++ 日志是否仍有 Cant get nearest lane"
            )
        else:
            log.line("[OK] planning 已产出轨迹")
        log.close()

    return rc


if __name__ == "__main__":
    sys.exit(main())
