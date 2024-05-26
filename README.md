# npurobocourse_sources

rmtt_node.py is tested in Ubuntu 20.04/ROS Noetic and used for authorized TT drones.

rmtt_node18.py is tested in Ubuntu 18.04/ROS Melodic.

Please switch in rmtt_bringup.launch.

To run rmtt_node18.py in Python 3.8:
```
pip install robomaster==0.1.1.63
pip install rospkg
pip install pyyaml
pip install scipy
```
add "-i https://pypi.tuna.tsinghua.edu.cn/simple" when running "pip install" in China.

In a new terminal from the workspace of npurobocourse_sources:
```
catkin_make_isolated
source devel_isolated/setup.bash
```

Connect to a TT drone by wifi, and run:
```
roslaunch rmtt_driver rmtt_bringup.launch
```

RoboMaster Tello Talent ROS Driver ([RMTT](https://github.com/tianbot/rmtt_ros)) is originally developed for [NPU-RoboCourse](https://github.com/cavayangtao/npurobocourse) through the cooperation of NPU-USRI and Tianbot. We keep the original readme below.

# RoboMaster Tello Talent (RMTT)
RoboMaster TT (also know as DJI Tello Talent or Ryze Tello Talent) features improved hardware and an LED light. RoboMaster TT suports Wi-Fi 5G channel and a flying map, which can be used for low-cost drone swarm. 


## Specification
|  Items  | Details|
|    :---:    |     :---:     |
| Model  | TLW004 |
| Weight  | 87 g |
| Max Speed  | 28.8 kph |
| Max Flight Time  | 13 min |
| Camera  | 2592×1936 |
| Battery  | LiPo |
| Capacity  | 1100 mAh |
| Voltage  | 3.8 V |


## Installation Instructions
Only support authorized devices and the environment is pre-configured in ROS2GO Noetic version.

## Usage Instructions
Please read the RoboMaster TT User Manual carefully.
Switch RoboMaster TT to direct connection mode, connect to RMTT-XXXXXX network.

### Start rmtt_driver node

```
roslaunch rmtt_driver rmtt_bringup.launch
```

## Topics
Published topics:
  - ~imu_data [sensor_msgs/Imu] 
  imu data. 

  - ~pose [geometry_msgs/PoseStamped] 
  Drone pose on the mission pad.

  - ~mission_pad_id [std_msgs/UInt8] 
  Mission pad id. 1 - 12.

  - ~image_raw/compressed [sensor_msgs/CompressedImage] 
  Compressed image topic.

  - ~image_raw [sensor_msgs/Image] 
  Image topic.

  - ~camera_info [sensor_msgs/CameraInfo] 
  RMTT camera info.

  - ~tof_btm [sensor_msgs/Range] 
  Bottom tof. Distance from drone to the ground.

  - ~tof_ext [sensor_msgs/Range] 
  External tof. Distance to the obstacle in front of the drone.

  - ~altitude [std_msgs/Float32] 
  Barometer readings.

  - ~battery [std_msgs/Float32] 
  Battery percentage.

Subscribed topics:
  - ~cmd_vel [geometry_msgs/Twist] 
  Command velocity. Only be effected after taking off.

  - ~takeoff [std_msgs/Empty] 
  Takeoff.

  - ~land [std_msgs/Empty]
  Land.

  - ~flip [std_msgs/Empty] 
  Flip.

  - ~led [std_msgs/ColorRGBA] 
  Led on top of the external module.

  - ~mled [std_msgs/String] 
  Dot matrix display. Showing charactor or string.

Services:
  - ~set_downvision [std_srvs/SetBool]
  Change the front cam to downward cam

  - ~set_hdmap
  Change the positioning to HD map mode. Can be used with the mission pad or HD map.

## License
The ROS driver is proprietary. Other packages are under BSD Clause-3 open source license. RoboMaster SDK is under Apache License, version 2.0.
