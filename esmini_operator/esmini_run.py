import os

# esmini是一个开源的仿真平台，可以用来模拟自动驾驶场景
# 下面是一个使用esmini执行xosc文件的示例代码

# 是否输出OSI信息的标志
OSI_INFO_OUTPUT = True

# 文件和程序路径
FILE_PATH = "/home/xs/Sunhan/Program_files/esmini-demo_Linux/esmini-demo/resources/xosc/cut-in.xosc"
ESMINI_PATH = "/home/xs/Sunhan/Program_files/esmini-demo_Linux/esmini-demo/bin/esmini"

# 构建命令行指令
base_cmd = f"{ESMINI_PATH} --window 60 60 1024 576 --osc {FILE_PATH}"
cmd = base_cmd + " --osi_receiver_ip 127.0.0.1" if OSI_INFO_OUTPUT else base_cmd

# 执行指令
os.system(cmd)