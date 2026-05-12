# odrviewer是esmini自带的一个工具，可以用来查看xodr格式的路网文件。下面是一个使用odrviewer查看xodr文件的示例代码：
import os


# 指定路网文件的路径
FILE_PATH = "/home/xs/桌面/git/mygit/esmini-demo_Linux/esmini-demo/resources/xodr/test2.xodr"
# 指定odrviewer程序的路径
ODRVIEWER_PATH = "/home/xs/桌面/git/mygit/esmini-demo_Linux/esmini-demo/bin/odrviewer"
# 指定运行的指令
CMD = ODRVIEWER_PATH + str(" --window 60 60 1024 576 --odr ") + FILE_PATH
# 运行指令
os.system(CMD)