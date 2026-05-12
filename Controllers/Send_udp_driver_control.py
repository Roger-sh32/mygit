import os
import sys

# 与仓库内 esmini/scripts/udp_driver 共用（从 mygit/Controllers 直接运行时常找不到模块）
_GIT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_UDP_DRIVER = os.path.join(_GIT_ROOT, "esmini", "scripts", "udp_driver")
if os.path.isdir(_UDP_DRIVER) and _UDP_DRIVER not in sys.path:
    sys.path.insert(0, _UDP_DRIVER)

from udp_osi_common import *  # noqa: E402

def send_udp_driver_control(udp_sender, throttle, brake, steering_angle):
    udp_sender.send(
        struct.pack(
            'iiiiddd',
            1,
            input_modes['driverInput'],
            0,
            0,
            throttle,
            brake,
            steering_angle
        )
    )

if __name__ == "__main__":
    udp_sender = UdpSender(port = 53995)
    send_udp_driver_control(udp_sender, throttle=0.1, brake=0.0, steering_angle=0.0)
    udp_sender.close()