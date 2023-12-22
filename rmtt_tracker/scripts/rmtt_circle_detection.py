#!/usr/bin/env python3

# 导入所需的模块
import rospy
import rospkg
import std_msgs.msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest as ROI
import cv2
from cv_bridge import CvBridge
import numpy as np
from math import pi  # 从math模块导入pi常数

# 设定图像的宽度和高度
IMAGE_WIDTH = 360
IMAGE_HEIGHT = 270

# 回调函数，处理接收到的图像消息
def image_callback(image_msg):
    # 将ROS图像消息转换为OpenCV图像
    image = bridge.imgmsg_to_cv2(image_msg)
    # 调整图像大小为指定宽度和高度
    resized_image = cv2.resize(image, (IMAGE_WIDTH, IMAGE_HEIGHT))

    # 在图像中找到圆
    processed_image, circle_info = find_circle(resized_image)
    # 跟踪圆并生成消息
    circle_msg = create_circle_roi(circle_info)
    
    # 打印圆的位置信息
    circle_position = f'cx: {circle_msg.x_offset} cy: {circle_msg.y_offset} radius: {circle_msg.width}'
    rospy.loginfo(circle_position)
    # 发布圆的信息
    roi_publisher.publish(circle_msg)
     
    # 在窗口中显示处理后的图像
    cv2.imshow('Frame', processed_image)
    cv2.waitKey(1)

# 在图像中找到圆
def find_circle(image):
    # 将图像转换为灰度图像并应用中值模糊
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred_image = cv2.medianBlur(gray_image, 5)

    # 使用霍夫圆变换检测圆
    circles = cv2.HoughCircles(blurred_image, cv2.HOUGH_GRADIENT, dp=1, minDist=70, param1=50, param2=50, minRadius=60, maxRadius=120)

    # 存储圆的中心坐标和面积信息
    circle_centers = []
    circle_areas = []

    if circles is not None:
        circles = np.uint16(np.around(circles))

        # 获取最大外接矩形的圆
        for (x, y, radius) in circles[0, :]:
            cv2.circle(image, (x, y), radius, (0, 255, 0), 2)
            cv2.circle(image, (x, y), 2, (0, 255, 255), 2)
            area = radius * radius * pi
            circle_areas.append(area)
            circle_centers.append([x, y])

        # 找到面积最大的圆
        max_area_index = circle_areas.index(max(circle_areas))
        return image, [circle_centers[max_area_index], circle_areas[max_area_index]]
    else:
        return image, [[0, 0], 0]

# 创建一个ROI消息以跟踪圆
def create_circle_roi(circle_info):
    circle_msg = ROI()
    if int(circle_info[1]) == 0:
        circle_msg.x_offset = 0
        circle_msg.y_offset = 0
        circle_msg.width = 0
        circle_msg.height = 0
    else:
        circle_msg.width = int(np.sqrt(circle_info[1] / pi))
        circle_msg.x_offset = int(circle_info[0][0])
        circle_msg.y_offset = int(circle_info[0][1])
        circle_msg.height = 0
    return circle_msg

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('circle_detector', anonymous=True)

    # 创建CvBridge实例
    bridge = CvBridge()

    # 创建一个订阅者，监听/image_raw主题的图像消息
    image_subscriber = rospy.Subscriber('/image_raw', Image, image_callback)

    # 创建一个发布者，将ROI消息发布到/circle_msg主题
    roi_publisher = rospy.Publisher('/circle_msg', ROI, queue_size=1)

    # 进入ROS循环
    rospy.spin()
