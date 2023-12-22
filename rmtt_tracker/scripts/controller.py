#!/usr/bin/env python3

# 导入所需的库和模块
import rospy
import std_msgs.msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import RegionOfInterest as ROI
import numpy as np

# 定义图像的尺寸
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 270

# 偏航通道PID参数，控制无人机左右旋转
PID_YAW = [0.5, 0, 0]
# 油门通道PID参数，控制无人机上升下降
PID_UP = [0.8, 0, 0]
# 俯仰通道PID参数，控制无人机前进后退
PID_FORWARD = [0.8, 0, 0]

if __name__ == '__main__':
    # 初始化节点
    rospy.init_node('controller', anonymous=True)

    # 订阅ROI话题，接收目标位置和大小信息
    rospy.Subscriber("roi", ROI, callback)

    # 发布控制命令到cmd_vel话题，控制无人机转向和移动
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    
    # 等待接收消息
    rospy.spin()