# esmini是一个开源的仿真平台，可以用来模拟自动驾驶场景。下面是一个使用esmini执行xosc文件的示例代码：

import os

# 是否输出OSI信息的标志，如果为True，则会输出OSI信息到本机；如果为False，则不会输出OSI信息
osi_info_output = True

# 指定场景文件的路径
file_path = "/home/xs/Sunhan/Program_files/esmini-demo_Linux/esmini-demo/resources/xosc/cut-in.xosc"
# 指定esmini程序的路径
esmini_path = "/home/xs/Sunhan/Program_files/esmini-demo_Linux/esmini-demo/bin/esmini"
# 指定运行的指令
if osi_info_output:
    cmd = esmini_path + str(" --window 60 60 1024 576 --osc ") + file_path + str(" --osi_receiver_ip 127.0.0.1")
else:
    cmd = esmini_path + str(" --window 60 60 1024 576 --osc ") + file_path
# 运行指令
os.system(cmd)