#!/usr/bin/env python3

# 导入所需的库和模块
import rospy
import rospkg
import std_msgs.msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

# 定义图像的尺寸
IMAGE_WIDTH = 360
IMAGE_HEIGHT = 240

# 回调函数，处理接收到的图像消息
def image_callback(msg):
    # 将ROS图像消息转换为OpenCV图像
    image = bridge.imgmsg_to_cv2(msg)
    # 调整图像大小
    image = cv2.resize(image, (IMAGE_WIDTH, IMAGE_HEIGHT))

    # 在窗口中显示图像
    cv2.imshow('Frame', image)
    cv2.waitKey(1)

if __name__ == '__main__':

    # 获取ROS包的路径
    rp = rospkg.RosPack()
    package_path = rp.get_path("rmtt_tracker")

    # 创建CvBridge以便于ROS图像和OpenCV图像之间的转换
    bridge = CvBridge()

    # 初始化ROS节点
    rospy.init_node('face_tracker', anonymous=True)

    # 创建订阅者以接收图像消息
    image_subscriber = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    
    # 进入ROS循环以接收并处理图像消息
    rospy.spin()