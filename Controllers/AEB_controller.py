'''
适配 esmini 的 AEB 控制：OSI 读状态，UDPDriver 发 UDP。
典型场景（Ego 已挂 UDPDriverController，BasePort 53995）：
  - resources/xosc/cut-in_simple.xosc
  - resources/xosc/AEB-test.xosc（由 acc-test 改编，红车 scripted 变道/制动）

启动后先用若干帧 stateXYH 注入「初始纵向速度」（与 testUDPDriver 一致：线速度为 m/s），
再切换为 driverInput 闭环；否则仅靠小油门自车可能长时间起不来。
AEB-test 中参数 EgoSpeed=120 km/h 时，可用 --initial-speed-ms 33.33 与历史 ACC 设定对齐。

运行顺序：
  1) 启动 esmini（须带 OSI），例如：
     <esmini>/bin/esmini --osc .../resources/xosc/AEB-test.xosc --osi_receiver_ip 127.0.0.1
  2) 再运行本脚本：
     python Controllers/AEB_controller.py [--initial-speed-ms 33.33]

OSI GroundTruth（本脚本用到的字段）简要说明：
- msg.host_vehicle_id：主车在 OSI 中的 id（若有）
- moving_object[]：id / base.position / base.velocity / base.orientation.yaw
- TTC：ego 航向下前车净距 / 接近速度（简化模型）
'''
import argparse
import math
import os
import struct
import sys
from socket import timeout

# 与 esmini-demo 中 scripts/udp_driver 共用 udp_osi_common（含 OSIReceiver、UdpSender、base_port）
_CONTROLLERS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_CONTROLLERS_DIR)
_UDP_DRIVER_DIR = os.path.join(
    _REPO_ROOT, 'esmini-demo_Linux', 'esmini-demo', 'scripts', 'udp_driver'
)
if os.path.isdir(_UDP_DRIVER_DIR) and _UDP_DRIVER_DIR not in sys.path:
    sys.path.insert(0, _UDP_DRIVER_DIR)

from udp_osi_common import *  # noqa: E402


def set_udp_driver(udp_sender, throttle=0.0, brake=0.0, steering_angle=0.0, object_id=0):
    """向 UDPDriver 发送一帧 driverInput（与 testUDPDriver-print-osi-info.py 一致）。"""
    global _frame_number
    udp_sender.send(
        struct.pack(
            'iiiiddd',
            1,
            input_modes['driverInput'],
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
    dead_reckon=0,
    object_id=0,
):
    """
    stateXYH：x,y,h 为当前位姿；speed 为纵向速度标量（m/s），与 testUDPDriver.Object.sendMessage 一致。
    dead_reckon=0 时 esmini 会将 z/俯仰等贴到路面。
    """
    global _frame_number
    udp_sender.send(
        struct.pack(
            'iiiidddddB',
            1,
            input_modes['stateXYH'],
            object_id,
            _frame_number,
            float(x),
            float(y),
            float(h),
            float(speed_ms),
            float(-steering_angle),
            int(dead_reckon) & 0xFF,
        )
    )
    _frame_number += 1


def _ego_from_msg(msg, ego_id=None):
    objs = msg.moving_object
    if not objs:
        return None
    if ego_id is None:
        hid = msg.host_vehicle_id
        ego_id = hid.value if hid is not None and hid.value != 0 else None
    ego = _moving_by_id(objs, ego_id) if ego_id is not None else None
    return ego if ego is not None else objs[0]


def _ego_for_control(msg, object_id):
    """优先按 object_id 找实体，避免 moving_object 顺序与 host_vehicle 不一致。"""
    objs = msg.moving_object
    if not objs:
        return None
    e = _moving_by_id(objs, object_id)
    return e if e is not None else _ego_from_msg(msg, None)


def _moving_by_id(moving_objects, id_value):
    for o in moving_objects:
        if o.id.value == id_value:
            return o
    return None


def _longitudinal_lateral(ex, ey, ego_yaw, px, py):
    """ego 坐标系：纵向 x 向前为正，横向 y 向左为正。"""
    dx = px - ex
    dy = py - ey
    c = math.cos(ego_yaw)
    s = math.sin(ego_yaw)
    lon = dx * c + dy * s
    lat = -dx * s + dy * c
    return lon, lat


def get_ttc(
    msg,
    ego_id=None,
    target_id=None,
    lane_half_width=2.0,
    vehicle_margin=5.0,
):
    """
    根据 OSI GroundTruth 估算与前方 relevant target 的碰撞时间（秒）。
    未构成威胁时返回 math.inf。
    """
    objs = msg.moving_object
    if len(objs) == 0:
        return math.inf

    if ego_id is None:
        hid = msg.host_vehicle_id
        ego_id = hid.value if hid is not None and hid.value != 0 else None

    ego = _moving_by_id(objs, ego_id) if ego_id is not None else None
    if ego is None:
        ego = objs[0]

    ex = ego.base.position.x
    ey = ego.base.position.y
    ego_yaw = ego.base.orientation.yaw
    vx_e = ego.base.velocity.x
    vy_e = ego.base.velocity.y

    target = None
    if target_id is not None:
        target = _moving_by_id(objs, target_id)
    else:
        best_lon = None
        for o in objs:
            if o.id.value == ego.id.value:
                continue
            lon, lat = _longitudinal_lateral(ex, ey, ego_yaw, o.base.position.x, o.base.position.y)
            if lon <= 0.0 or abs(lat) > lane_half_width:
                continue
            if best_lon is None or lon < best_lon:
                best_lon = lon
                target = o

    if target is None:
        return math.inf

    lon, lat = _longitudinal_lateral(
        ex, ey, ego_yaw, target.base.position.x, target.base.position.y
    )
    if lon <= 0.0 or abs(lat) > lane_half_width:
        return math.inf

    gap = lon - vehicle_margin
    if gap <= 0.0:
        return 0.0

    c = math.cos(ego_yaw)
    s = math.sin(ego_yaw)
    vx_t = target.base.velocity.x
    vy_t = target.base.velocity.y
    ego_along = vx_e * c + vy_e * s
    target_along = vx_t * c + vy_t * s
    closing = ego_along - target_along
    if closing <= 1e-3:
        return math.inf

    return gap / closing


def print_osi_stuff(msg):
    print("OSI message timestamp: {:.2f} seconds".format(msg.timestamp.seconds + msg.timestamp.nanos * 1e-9))
    print("{} lanes".format(len(msg.lane)))
    print('{} moving objects'.format(len(msg.moving_object)))
    for i, o in enumerate(msg.moving_object):
        print('  [{}] id {}'.format(i, o.id.value))
        print('    pos.x {:.2f} pos.y {:.2f} rot.h {:.2f}'.format(o.base.position.x, o.base.position.y, o.base.orientation.yaw))
        print('    vel.x {:.2f} vel.y {:.2f}'.format(o.base.velocity.x, o.base.velocity.y))


_frame_number = 0
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='AEB demo for esmini (UDPDriver Ego, e.g. cut-in_simple / AEB-test)')
    parser.add_argument('--ip', default='127.0.0.1', help='esmini 所在主机')
    parser.add_argument('--port', type=int, default=None, help='UDP driver 端口（默认 base_port+object_id）')
    parser.add_argument('--object-id', type=int, default=0, help='被控实体 id，与 xosc 中 Ego 一致时为 0')
    parser.add_argument('--max-loops', type=int, default=3000, help='主循环最大次数')
    parser.add_argument('--cruise-throttle', type=float, default=0.18, help='TTC>阈值时的巡航油门 0~1')
    parser.add_argument('--brake', type=float, default=0.55, help='TTC<=阈值时的制动 0~1')
    parser.add_argument('--ttc-threshold', type=float, default=2.0, help='触发制动的 TTC 阈值（秒）')
    parser.add_argument(
        '--initial-speed-ms',
        type=float,
        default=20.0,
        help='启动阶段注入的纵向速度（m/s），与原先 cut-in_simple 中 Ego 的 20 m/s 一致',
    )
    parser.add_argument(
        '--bootstrap-frames',
        type=int,
        default=40,
        help='前若干帧用 stateXYH 锁位姿+初速，再切 driverInput；0=不注入初速',
    )
    args = parser.parse_args()

    obj_port = (args.port if args.port is not None else base_port + args.object_id)

    print('典型场景: cut-in_simple.xosc 或 AEB-test.xosc（Ego=UDPDriver, BasePort 53995）')
    print('UDP driver -> {}:{}  object_id={}'.format(args.ip, obj_port, args.object_id))
    print(
        '初速: {:.2f} m/s × {} 帧 stateXYH，再切 driverInput'.format(
            args.initial_speed_ms,
            args.bootstrap_frames,
        )
    )
    print('请确认 esmini 已加: --osi_receiver_ip 127.0.0.1')
    print('---')

    udp_sender = UdpSender(ip=args.ip, port=obj_port)
    osi_receiver = OSIReceiver()
    done = False
    counter = 0

    while not done and counter < args.max_loops:
        print('======== Counter: {} ========'.format(counter))
        try:
            msg = osi_receiver.receive()
            ego = _ego_for_control(msg, args.object_id)
            if ego is None:
                print('无 moving_object，跳过本帧')
                counter += 1
                continue

            if args.bootstrap_frames > 0 and counter < args.bootstrap_frames:
                send_state_xyh(
                    udp_sender,
                    ego.base.position.x,
                    ego.base.position.y,
                    ego.base.orientation.yaw,
                    args.initial_speed_ms,
                    steering_angle=0.0,
                    dead_reckon=0,
                    object_id=args.object_id,
                )
                ttc = get_ttc(msg)
                print(
                    'bootstrap {}/{}  TTC: {:.2f} s'.format(
                        counter + 1,
                        args.bootstrap_frames,
                        ttc,
                    )
                )
            else:
                ttc = get_ttc(msg)
                print('TTC: {:.2f} seconds'.format(ttc))
                if ttc <= args.ttc_threshold:
                    print('TTC <= {:.1f} s, brake'.format(args.ttc_threshold))
                    set_udp_driver(
                        udp_sender,
                        throttle=0.0,
                        brake=args.brake,
                        steering_angle=0.0,
                        object_id=args.object_id,
                    )
                else:
                    set_udp_driver(
                        udp_sender,
                        throttle=args.cruise_throttle,
                        brake=0.0,
                        steering_angle=0.0,
                        object_id=args.object_id,
                    )
        except timeout:
            print('OSI recv timeout（检查 esmini 是否仍在运行、是否带 --osi_receiver_ip）')
            done = True
        except KeyboardInterrupt:
            print('Ctrl+C, quit')
            done = True

        counter += 1

    udp_sender.close()
    osi_receiver.close()
