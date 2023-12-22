#!/usr/bin/env python3

# 导入所需的库
import numpy as np
from simple_pid import PID
from cv_bridge import CvBridge
import rospy
import tf2_ros
from dynamic_reconfigure.server import Server
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from rmtt_tracker.cfg import tracker_pidConfig

# PID控制器的参数设置
# 设置PID分别是KP=0.6, KI=0, KD=0，用于计算机器人x方向的速度
pid_x = PID(0.6, 0, 0) 
# 设置PID分别是KP=0.6, KI=0, KD=0，用于计算机器人y方向的速度
pid_y = PID(0.6, 0, 0) 
# 设置PID分别是KP=0.6, KI=0, KD=0，用于计算机器人z方向的速度
pid_z = PID(0.6, 0, 0) 
# 设置输出限制，保证输出在-0.8~0.8之间
pid_x.output_limits = pid_y.output_limits = pid_z.output_limits = (-0.8, 0.8) 
# 设置PID分别是KP=2, KI=0, KD=0，用于计算机器人绕z轴的角速度
pid_a = PID(2, 0, 0) 
# 设置输出限制，保证输出在-1.2~1.2之间
pid_a.output_limits = (-1.2, 1.2) 

# 创建一个Twist类型的对象，用于存储机器人的运动速度
vel = Twist() 
# 初始化标志，用于在没有检测到apriltag的情况下停止发布控制信号
zero_twist_published = False 
# 初始化标签ID
tag_id = 0 
# 初始化标志，表示是否检测到apriltag
tag_detected = False 

# 配置PID控制器
def pid_cb(config, level):
    rospy.loginfo("""Reconfigure Request: {linear_kp}, {linear_ki}, {linear_kd}, {angular_kp}, {angular_ki}, {angular_kd}""".format(**config))
    pid_x = [float(config.get(key)) for key in ['linear_kp', 'linear_ki', 'linear_kd']]
    pid_y = [float(config.get(key)) for key in ['linear_kp', 'linear_ki', 'linear_kd']]
    pid_z = [float(config.get(key)) for key in ['linear_kp', 'linear_ki', 'linear_kd']]
    pid_a = [float(config.get(key)) for key in ['angular_kp','angular_ki','angular_kd']]
    return config

# 监听apriltag的回调函数
def tag_callback(msg):
    # 全局变量，用于在函数中引用外部的全局变量
    global zero_twist_published
    global tag_detected
    global vel

    # 如果检测到apriltag
    if msg.detections:
        for tag in msg.detections:
            # 如果检测到的标签ID与所设定的ID相同
            if int(tag_id) in tag.id: 
                # 计算目标与朝向之间所需的偏航角
                alpha = np.arctan2(tag.pose.pose.pose.position.x, tag.pose.pose.pose.position.z)
                # 使用PID控制器计算绕z轴的角速度
                vel.angular.z = pid_a(alpha)
                # 使用PID控制器计算上下方向的速度
                vel.linear.z = pid_z(tag.pose.pose.pose.position.y)
                # 发布控制信号
                vel_pub.publish(vel)
                zero_twist_published = False
                tag_detected = True
            else:
                tag_detected = False
    else:
        tag_detected = False
        
    # 如果没有检测到apriltag
    if not tag_detected:
        if not zero_twist_published:
            # 停止发布控制信号
            zero_twist = Twist()
            vel_pub.publish(zero_twist)
            zero_twist_published = True


if __name__ == '__main__':

    # 初始化ros节点
    rospy.init_node('tag_tracker')

    # 获取标签的ID，默认是5
    tag_id = rospy.get_param('~tag_id', '5')
    tag_name = "tag_" + str(tag_id)
    
    # 获取跟踪距离，默认是0.8米
    track_distance = rospy.get_param("~track_distance", 0.8)

    # 订阅apriltag的话题
    tag_sub = rospy.Subscriber('tag_detections', AprilTagDetectionArray, tag_callback, queue_size=1)
    # 发布控制信号的话题
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # 动态配置PID控制器参数的服务器
    srv = Server(tracker_pidConfig, pid_cb)

    # 通过静态TF变换，发布apriltag到无人机目标位置的静态坐标变换
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    static_transformStamped = TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = tag_name
    target_frame = tag_name + "_target"
    static_transformStamped.child_frame_id = target_frame

    static_transformStamped.transform.translation.y += track_distance*np.tan(np.deg2rad(15))
    static_transformStamped.transform.translation.z += track_distance 

    quat = quaternion_from_euler(np.deg2rad(-90), np.deg2rad(90), 0)
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]
    broadcaster.sendTransform(static_transformStamped)

    # TF 监听器初始化
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # 定义频率为10Hz的定时器
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        # 如果检测到apriltag
        if tag_detected:
            try:
                # 获取当前无人机base_link到无人机目标位姿的变换
                trans = tfBuffer.lookup_transform(target_frame, (rospy.get_namespace()+'base_link').strip("/"), rospy.Time(), rospy.Duration(0.2))
                # 用于存储机器人的运动速度
                vel = Twist()
                # 使用PID控制器计算机器人x方向的速度
                vel.linear.x = pid_x(trans.transform.translation.x)
                # 使用PID控制器计算机器人y方向的速度
                vel.linear.y = pid_y(trans.transform.translation.y)
                # 使用PID控制器计算机器人z方向的速度
                vel.linear.z = pid_z(trans.transform.translation.z)
                # 延时，保持机器人按照设定速度运动
                rate.sleep()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(e)
                continue
    # 等待接收消息
    rospy.spin()