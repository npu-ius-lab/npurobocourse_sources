#!/usr/bin/env python3

# 导入所需的库和模块
import rospy
from turtlesim.srv import Spawn, SpawnRequest

def main():
    rospy.init_node("turtle_spawn")

    # 创建服务客户端，连接到 "/spawn" 服务
    client = rospy.ServiceProxy("/spawn", Spawn)

    # 等待服务启动
    rospy.wait_for_service("/spawn")

    # 创建请求数据
    request = SpawnRequest()
    request.x = 1.0  # 机器人的初始 x 坐标
    request.y = 1.0  # 机器人的初始 y 坐标
    request.theta = 3.14  # 机器人的初始方向，以弧度表示
    request.name = "turtle2"  # 新生成的机器人的名称

    try:
        # 发送请求并处理响应
        response = client(request)

        # 打印成功信息和机器人的名称
        rospy.loginfo("乌龟创建成功，名字是: %s", response.name)
    except rospy.ServiceException as e:
        # 打印异常信息，如果出现问题
        rospy.logerr("服务调用失败: %s", e)

if __name__ == "__main__":
    main()
