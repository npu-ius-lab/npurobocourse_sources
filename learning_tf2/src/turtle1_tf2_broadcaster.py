#!/usr/bin/env python3

# 导入所需的库和模块
import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import turtlesim.msg
from geometry_msgs.msg import TransformStamped

# 回调函数，用于处理海龟机器人的位置消息并广播Transforms消息
def handle_turtle_pose(msg, turtlename):
    # 创建一个 TransformBroadcaster 对象
    br = tf2_ros.TransformBroadcaster()

    # 创建一个 TransformStamped 对象，用于存储Transforms消息
    t = TransformStamped()

    # 从消息中读取数据并将其分配给相应的 tf 变量
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'world'  # 参考坐标系
    t.child_frame_id = turtlename  # 子坐标系的名称，通常是机器人的名称
    
    # 海龟机器人只存在于2D平面，因此从消息中获取x和y平移坐标，并将z坐标设置为0
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0

    # 由于海龟机器人只能绕一个轴旋转，所以我们将旋转坐标x和y设置为0，并从消息中获取z轴上的旋转
    q = quaternion_from_euler(0, 0, msg.theta)  # 创建四元数表示旋转
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    
    # 使用 TransformBroadcaster 发送Transforms消息
    br.sendTransform(t)

if __name__ == '__main__':
    # 初始化节点
    rospy.init_node('turtle1_tf2_broadcaster')
    turtlename = 'turtle1'  # 机器人的名称
    
    # 订阅海龟机器人的位置消息，当有消息时，调用 handle_turtle_pose 函数
    rospy.Subscriber('/%s/pose' % turtlename,
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     turtlename)

    # 保持节点运行，等待消息
    rospy.spin()  
