'''
适配esmini的AEB控制算法,当TTC小于等于2秒时，施加制动，防止碰撞。
'''
import math
import struct
from udp_osi_common import *

# def print_osi_stuff(msg):

#     print("OSI message timestamp: {:.2f} seconds".format(msg.timestamp.seconds + msg.timestamp.nanos * 1e-9))

#     # Print some static content typically only available in first message
#     print("{} lanes".format(len(msg.lane)))
#     for i, l in enumerate(msg.lane):
#         clf = l.classification
#         print("  [{}] id {} type: {}".format(i, l.id.value, clf.type))
#         print("    centerline:")
#         for c_line in clf.centerline:
#             print("    x: {:.2f} y: {:.2f}".format(c_line.x, c_line.y))

#     print('{} stationary objects'.format(len(msg.stationary_object)))
#     for i, s in enumerate(msg.stationary_object):
#         print('  [{}] id {} type {}'.format(i, s.id.value, s.classification.type))
#         print('    pos.x {:.2f} pos.y {:.2f} rot.h {:.2f}'.format(s.base.position.x, s.base.position.y, s.base.orientation.yaw))

#     # Print some dynamic content from the message
#     print('{} moving objects'.format(len(msg.moving_object)))
#     for i, o in enumerate(msg.moving_object):
#         print('  [{}] id {}'.format(i, o.id.value))
#         print('    pos.x {:.2f} pos.y {:.2f} rot.h {:.2f}'.format(o.base.position.x, o.base.position.y, o.base.orientation.yaw))
#         print('    vel.x {:.2f} vel.y {:.2f} rot_rate.h {:.2f}'.format(o.base.velocity.x, o.base.velocity.y, o.base.orientation_rate.yaw))
#         print('    acc.x {:.2f} acc.y {:.2f} rot_acc.h {:.2f}'.format(o.base.acceleration.x, o.base.acceleration.y, o.base.orientation_acceleration.yaw))

#         lane_id = o.assigned_lane_id[0].value if len(msg.lane) > 0 and len(o.assigned_lane_id) > 0 else -1
#         left_lane_id = -1
#         right_lane_id = -1
#         for l in msg.lane:
#             if l.id.value == o.assigned_lane_id[0].value:
#                 left_lane_id = l.classification.left_adjacent_lane_id[0].value if len(l.classification.left_adjacent_lane_id) > 0 else -1
#                 right_lane_id = l.classification.right_adjacent_lane_id[0].value if len(l.classification.right_adjacent_lane_id) > 0 else -1
#                 break
#         print('    lane id {} left adj lane id {} right adj lane id {}'.format(lane_id, left_lane_id, right_lane_id))


_frame_number = 0


def set_udp_driver(
    udp_sender,
    throttle=0.0,
    brake=0.0,
    steering_angle=0.0,
    object_id=0,
):
    """
    向 esmini UDPDriverController 发送 driverInput 控制量。

    throttle: 油门，范围 0.0 ~ 1.0
    brake: 制动，范围 0.0 ~ 1.0
    steering_angle: 方向盘/转角输入，单位一般按 esmini 示例理解为 rad
    object_id: 被控制对象 id，通常 ego 是 0
    """
    global _frame_number

    throttle = max(0.0, min(1.0, throttle))
    brake = max(0.0, min(1.0, brake))

    msg = struct.pack(
        "iiiiddd",
        1,                          # version
        input_modes["driverInput"], # input mode
        object_id,                  # object id
        _frame_number,              # frame number
        throttle,
        brake,
        -steering_angle             # 和 testUDPDriver.py 保持一致
    )

    udp_sender.send(msg)
    _frame_number += 1

def speed_2d(obj):
    """计算 moving object 的二维速度模长"""
    vx = obj.base.velocity.x
    vy = obj.base.velocity.y
    return math.sqrt(vx * vx + vy * vy)


def get_ttc(msg, ego_id=36, target_id=37, lane_width=3.8):
    """
    从 OSI GroundTruth 中计算 ego 与目标车之间的 TTC。

    默认：
    - ego_id=36
    - target_id=37

    你之前 cut-in log 里：
    id36 在 lane16，速度约 30m/s；
    id37 在 lane14，后续速度约 36m/s。
    所以这里先按你的 log 写默认值。

    TTC 计算逻辑：
    - 只在目标车位于 ego 前方时计算；
    - 如果目标车不在 ego 前方，返回 inf；
    - 如果两车没有接近趋势，返回 inf；
    - 如果横向距离过大，认为暂时不构成 AEB 目标，返回 inf。
    """
    ego = None
    target = None

    for obj in msg.moving_object:
        if obj.id.value == ego_id:
            ego = obj
        elif obj.id.value == target_id:
            target = obj

    if ego is None or target is None:
        return float("inf")

    ego_x = ego.base.position.x
    ego_y = ego.base.position.y
    target_x = target.base.position.x
    target_y = target.base.position.y

    ego_v = speed_2d(ego)
    target_v = speed_2d(target)

    dx = target_x - ego_x
    dy = target_y - ego_y

    lateral_gap = abs(dx)
    longitudinal_gap = dy

    # 这里按你的场景方向：车辆主要沿 y 正方向行驶
    # 目标车必须在 ego 前方，才计算前向 TTC
    if longitudinal_gap <= 0:
        return float("inf")

    # 横向距离太远，暂不认为是本车道直接碰撞目标
    # cut-in 场景可以适当放宽，例如 1.5 个车道宽
    if lateral_gap > lane_width * 1.5:
        return float("inf")

    # ego 比目标车快，才有追尾风险
    closing_speed = ego_v - target_v

    if closing_speed <= 0:
        return float("inf")

    return longitudinal_gap / closing_speed
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
        except timeout:
            print('osiReceive Timeout')
            done = True
        except KeyboardInterrupt:
            print('Ctrl+C pressed, quit')
            done = True

        counter += 1

    # Close and quit
    udpSender0.close()
    osiReceiver.close()
