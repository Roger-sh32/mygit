'''
适配esmini的AEB控制算法,当TTC小于等于2秒时，施加制动，防止碰撞。

OSI GroundTruth（本脚本用到的字段）简要说明：
- msg.host_vehicle_id：仿真定义的「主车 / ego」在 OSI 里的 id（若有），用于在 moving_object 里找对的车。
- moving_object[]：动态物体列表；每个元素的 id / base.position / base.velocity / base.orientation.yaw
  分别为全局坐标下的位置 (x,y)、速度 (vx,vy) 与航向 yaw（弧度）。
- AEB 用的 TTC：在 ego 航向坐标系下算「前车纵向间距」与「沿航向的接近速度」，近似为 gap / closing_speed。
'''
import math
import struct
from socket import timeout

from udp_osi_common import *


def set_udp_driver(udp_sender, throttle=0.0, brake=0.0, steering_angle=0.0, object_id=0):
    """向 UDPDriver 发送一帧 driverInput（格式与 testUDPDriver-print-osi-info.py 一致）。"""
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


def _moving_by_id(moving_objects, id_value):
    for o in moving_objects:
        if o.id.value == id_value:
            return o
    return None


def _longitudinal_lateral(ex, ey, ego_yaw, px, py):
    """ego 坐标系：纵向 x 向前为正，横向 y 向左为正（与常见车辆坐标一致）。"""
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
    根据 OSI GroundTruth 估算与「前方 relevant target」的碰撞时间（秒）。

    :param msg: osi3 GroundTruth（已由 OSIReceiver.receive() 解析）
    :param ego_id: 主车 OSI id；默认 None 时使用 msg.host_vehicle_id（若无效则用 moving_object[0]）
    :param target_id: 目标车 OSI id；默认 None 时在主车前方、车道宽度内的最近一辆车
    :param lane_half_width: 认为「同车道」的最大横向偏移（米）
    :param vehicle_margin: 两车几何与安全裕度近似（米），从中心距中扣除，相当于粗略保险杠间距
    :return: TTC（秒）；未构成威胁时返回 math.inf
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

    # Print some static content typically only available in first message
    print("{} lanes".format(len(msg.lane)))
    for i, l in enumerate(msg.lane):
        clf = l.classification
        print("  [{}] id {} type: {}".format(i, l.id.value, clf.type))
        print("    centerline:")
        for c_line in clf.centerline:
            print("    x: {:.2f} y: {:.2f}".format(c_line.x, c_line.y))

    print('{} stationary objects'.format(len(msg.stationary_object)))
    for i, s in enumerate(msg.stationary_object):
        print('  [{}] id {} type {}'.format(i, s.id.value, s.classification.type))
        print('    pos.x {:.2f} pos.y {:.2f} rot.h {:.2f}'.format(s.base.position.x, s.base.position.y, s.base.orientation.yaw))

    # Print some dynamic content from the message
    print('{} moving objects'.format(len(msg.moving_object)))
    for i, o in enumerate(msg.moving_object):
        print('  [{}] id {}'.format(i, o.id.value))
        print('    pos.x {:.2f} pos.y {:.2f} rot.h {:.2f}'.format(o.base.position.x, o.base.position.y, o.base.orientation.yaw))
        print('    vel.x {:.2f} vel.y {:.2f} rot_rate.h {:.2f}'.format(o.base.velocity.x, o.base.velocity.y, o.base.orientation_rate.yaw))
        print('    acc.x {:.2f} acc.y {:.2f} rot_acc.h {:.2f}'.format(o.base.acceleration.x, o.base.acceleration.y, o.base.orientation_acceleration.yaw))

        lane_id = o.assigned_lane_id[0].value if len(msg.lane) > 0 and len(o.assigned_lane_id) > 0 else -1
        left_lane_id = -1
        right_lane_id = -1
        for l in msg.lane:
            if l.id.value == o.assigned_lane_id[0].value:
                left_lane_id = l.classification.left_adjacent_lane_id[0].value if len(l.classification.left_adjacent_lane_id) > 0 else -1
                right_lane_id = l.classification.right_adjacent_lane_id[0].value if len(l.classification.right_adjacent_lane_id) > 0 else -1
                break
        print('    lane id {} left adj lane id {} right adj lane id {}'.format(lane_id, left_lane_id, right_lane_id))


_frame_number = 0
if __name__ == "__main__":

    # Create UDP socket objects
    udpSender0 = UdpSender(port = base_port + 0)
    osiReceiver = OSIReceiver()
    done = False
    counter = 0

    # # Press throttle to achieve some speed
    # set_udp_driver(udpSender0, throttle=0.1)


    # AEB control loop
    while not done and counter < 800:
        # Read OSI
        print(f"======== Counter: {counter} ========")
        try:
            msg = osiReceiver.receive()
            ttc = get_ttc(msg)
            print('TTC: {:.2f} seconds'.format(ttc))
            if ttc <= 2.0:
                print('TTC <= 2.0 seconds, applying brake!')
                set_udp_driver(udpSender0, throttle=0.0, brake=0.5, steering_angle=0.0)
            else:
                # 闭环需周期发送 driverInput；否则前车拉开后车速会掉光
                set_udp_driver(udpSender0, throttle=0.12, brake=0.0, steering_angle=0.0)
        except timeout:
            print('osiReceive Timeout（若需超时，请给 UdpReceiver 设置 timeout）')
            done = True
        except KeyboardInterrupt:
            print('Ctrl+C pressed, quit')
            done = True

        counter += 1

    # Close and quit
    udpSender0.close()
    osiReceiver.close()
