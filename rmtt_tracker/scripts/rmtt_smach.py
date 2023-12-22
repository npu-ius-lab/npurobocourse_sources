#!/usr/bin/env python3

# 导入所需的库和模块
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import RegionOfInterest as ROI
from sensor_msgs.msg import Range
from std_msgs.msg import UInt8
import smach
import smach_ros
import datetime
import numpy as np

# 初始化任务、PAD_ID、高度变量
mission = 1
pad_id = 0
height = 0

# 定义状态 Tag_track
class TagTrack(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['next', 'stay'])

    def execute(self, userdata):
        global pad_id, mission
        rospy.loginfo('执行状态 Tag_track')
        if pad_id == 1:
            zero_twist = Twist()
            pub.publish(zero_twist)
            mission = 2
            return 'next'
        else:
            return 'stay'

# 定义状态 Rise
class Rise(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['next', 'stay'])
        
    def execute(self, userdata):
        global height, mission
        rospy.loginfo('执行状态 Rise')  
        if height > 1.5:
            zero_twist = Twist()
            pub.publish(zero_twist)
            mission = 3
            return 'next'
        else:
            vel_rise = Twist()
            vel_rise.linear.z = 0.2
            pub.publish(vel_rise)
            return 'stay'

# 定义状态 Turn
class Turn(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['stay'])
        
    def execute(self, userdata):
        global mission, rad
        rospy.loginfo('执行状态 Turn')  

        vel_turn = Twist()  # 更正变量名
        vel_turn.angular.z = 0.2
        pub.publish(vel_turn)
        return 'stay'

# 回调函数，处理来自/cmd_vel_tag的消息
def callbackCmdTag(msg):
    if mission == 1:
        pub.publish(msg)

# 回调函数，处理来自/mission_pad_id的消息
def callbackMissionPad(msg):
    global pad_id
    pad_id = msg.data

# 回调函数，处理来自/tof_btm的消息
def callbackTofBtm(msg):
    global height
    height = msg.range

if __name__ == '__main__':
    
    # 初始化ROS节点
    rospy.init_node('state_machine')


    # 创建订阅者和发布者
    rospy.Subscriber('/mission_pad_id', UInt8, callbackMissionPad)
    rospy.Subscriber('/cmd_vel_tag', Twist, callbackCmdTag)
    rospy.Subscriber('/tof_btm', Range, callbackTofBtm)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # 创建一个 SMACH 状态机
    sm = smach.StateMachine(outcomes=['end'])

    # 打开容器
    with sm:    
        # 向容器中添加状态
        smach.StateMachine.add('TAG_TRACK', TagTrack(), 
                               transitions={'next':'RISE', 'stay':'TAG_TRACK'})
        smach.StateMachine.add('RISE', Rise(), 
                               transitions={'next':'TURN', 'stay':'RISE'})
        smach.StateMachine.add('TURN', Turn(), 
                               transitions={'stay':'TURN'})

    # 执行状态机
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        outcome = sm.execute()
        rate.sleep()
    
    # 进入ROS循环以接收并处理消息
    rospy.spin()