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

# 偏航通道PID参数，控制无人机左右旋转
PID_YAW = [0.5, 0, 0]
# 油门通道PID参数，控制无人机上升下降
PID_UP = [0.8, 0, 0]
# 俯仰通道PID参数，控制无人机前进后退
PID_FORWARD = [0.8, 0, 0]

# 用于标记是否已发布零速度的标志
zero_twist_published = False

# 回调函数，处理接收到的图像消息
def image_callback(msg):
    global zero_twist_published
    # 将ROS图像消息转换为OpenCV图像
    image = bridge.imgmsg_to_cv2(msg)
    # 调整图像大小
    image = cv2.resize(image, (IMAGE_WIDTH, IMAGE_HEIGHT))

    # 在图像中查找人脸
    img, face_info = find_face(image)
    
    # 跟踪人脸并获取控制信号
    yaw_velocity, up_velocity, forward_velocity = calculate_control_signals(face_info, IMAGE_WIDTH, IMAGE_HEIGHT, PID_YAW, PID_UP, PID_FORWARD)
    
    # 构建Twist消息以表示机器人速度
    rc = "左右: " + "0 " + "前进: " + str(forward_velocity) + " 上升: " + str(up_velocity) + "  偏航: " + str(yaw_velocity)
    velocity = Twist()
    
    if yaw_velocity != 0 or up_velocity != 0 or forward_velocity != 0:
        velocity.linear.x = -forward_velocity
        velocity.linear.y = 0.0
        velocity.linear.z = -up_velocity
        velocity.angular.x = 0.0
        velocity.angular.y = 0.0
        velocity.angular.z = -yaw_velocity
        rospy.loginfo(rc)
        velocity_publisher.publish(velocity)
        zero_twist_published = False
    else:
        if not zero_twist_published:
            velocity_publisher.publish(velocity)
            zero_twist_published = True
            rospy.loginfo("未检测到人脸")

    # 在窗口中显示图像
    cv2.imshow('Frame', img)
    cv2.waitKey(1)

# 在图像中查找人脸
def find_face(img):
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(img_gray, 1.1, 6)

    # 获取最大边界框内的人脸
    face_list_centers = []
    face_list_areas = []
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        center_x = x + w // 2
        center_y = y + h // 2
        area = w * h
        face_list_areas.append(area)
        face_list_centers.append([center_x, center_y])
    if len(face_list_areas) != 0:
        i = face_list_areas.index(max(face_list_areas))
        return img, [face_list_centers[i], face_list_areas[i]]
    else:
        return img, [[0, 0], 0]

# 跟踪人脸并计算控制信号
def calculate_control_signals(info, image_width, image_height, pid_yaw, pid_up, pid_forward):
    # 比例控制
    # 偏航通道
    error_yaw = info[0][0] - image_width // 2
    yaw_velocity = pid_yaw[0] * error_yaw
    yaw_velocity = int(np.clip(yaw_velocity, -100, 100)) / 100.0

    # 上升通道
    error_up = 0
    up_velocity = 0
    up_velocity = 0

    # 前进通道
    error_forward = np.sqrt(info[1]) - 70
    forward_velocity = pid_forward[0] * error_forward
    forward_velocity = int(np.clip(forward_velocity, -100, 100)) / 100.0

    if info[0][0] != 0:
        yaw_velocity = yaw_velocity
    else:
        yaw_velocity = 0
    if info[0][1] != 0:
        up_velocity = up_velocity
    else:
        up_velocity = 0
    if info[1] != 0:
        forward_velocity = forward_velocity
    else:
        forward_velocity = 0

    return yaw_velocity, up_velocity, forward_velocity

if __name__ == '__main__':
    # 获取ROS包的路径
    rp = rospkg.RosPack()
    package_path = rp.get_path("rmtt_tracker")
    
    # 加载人脸检测的Haar Cascade分类器
    face_cascade = cv2.CascadeClassifier(package_path + '/config/haarcascade_frontalface_default.xml')
    
    # 创建CvBridge以便于ROS图像和OpenCV图像之间的转换
    bridge = CvBridge()
    
    # 初始化ROS节点
    rospy.init_node('face_tracker', anonymous=True)
    
    # 创建订阅者以接收图像消息
    image_subscriber = rospy.Subscriber("/image_raw", Image, image_callback)
    
    # 创建发布者以发布Twist消息
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    # 进入ROS循环以接收并处理图像消息
    rospy.spin()