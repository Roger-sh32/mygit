import os

# esmini是一个开源的仿真平台，可以用来模拟自动驾驶场景
# 下面是一个使用esmini回放仿真数据的示例代码

# 是否碰撞暂停 默认不暂停
Collision_Pause = True
# 仿真数据路径
SIM_DATA_PATH = "/home/xs/桌面/git/mygit/esmini_operator/record_data/sim.dat"
# 文件和程序路径
REPLAYER_PATH = "/home/xs/桌面/git/mygit/esmini-demo_Linux/esmini-demo/bin/replayer"

# 执行指令
if Collision_Pause:
    os.system(f"{REPLAYER_PATH} --file {SIM_DATA_PATH} --res_path /home/xs/桌面/git/mygit/esmini-demo_Linux/esmini-demo/resources --window 60 60 1024 576 --collision")
else:
    os.system(f"{REPLAYER_PATH} --file {SIM_DATA_PATH} --res_path /home/xs/桌面/git/mygit/esmini-demo_Linux/esmini-demo/resources --window 60 60 1024 576")