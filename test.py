import threading

# 定义一个线程的执行函数
def thread_function(name):
    print("线程 {} 正在运行".format(name))

# 创建多个线程并启动它们
if __name__ == "__main__":
    # 创建线程对象
    threads = []
    for i in range(5):
        thread = threading.Thread(target=thread_function, args=(i,))
        threads.append(thread)
        thread.start()

    # 等待所有线程执行完毕
    for thread in threads:
        thread.join()

    print("所有线程执行完毕")
