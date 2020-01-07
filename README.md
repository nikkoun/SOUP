# SOUP Project

#### System architecture

![ROS_Diagram](/images/ros_diagram.png)

#### MoveIt / Robot Configuration
This Node implements a control interface for the UR10e Robot. Contains UR10e Robot Model, Attenuators Models, Kinematics Solvers and Motion Planning Libraries. A Custom Attenuator should be modeled and added to the existing model with a seperate dedicated controller. Collisions are handled by imposing constrains to the model. Can run in sim mode or with real robot. 
Environment and objects can also be modeled

#### C++ Integration / Tasks
C++ Interface for data processing and task implementation (scann, fog, harvest). Can be used to set goals (poses) / constrains manualy and control the interaction between the robot and the environment. Links to PCL and OpenCV Libraries for handling RGB-D datagenerated by the camera. Predefined Tasks (Scann, Fog, Harvest) are implemented in C++ and can be triggered remotely via (amazon services DB?).

#### RGB-D SLAM 
Real time Localization and mapping. RGB-D data (pointclouds) from the camera are used to build a 3D map of the environment.     
Implements odometry via the @@@@ sensor for localization based on the reference frame of the camera. The Map is in pointcloud format and can be accessed by other ros processes. 

#### Realsense SDK 
ROS Interface for Realsense D415 camera. Provides a ROS interface for data generated by camera sensors and many data processing tools. 

#### Gazebo 
Robot model can be vizualized and controlled in gazebo through the gazebo node for testing.

#### UR ROS Driver / URCap
UR Driver Node handles communication between ROS and the actual robot. URCap is installed on the robot side to establish communication with UR Driver node through TCP/IP (Remote Control).  




## MoveIt Controller and model for Universal Robots (included in ROS Industrial)
https://github.com/ros-industrial/universal_robot

#### Start gazebo and load arm model

    roslaunch ur_e_gazebo ur10e.launch

#### Load MoveIt contrrollers for robot

    roslaunch ur10_e_moveit_config ur10_e_moveit_planning_execution.launch sim:=true

#### Starts rviz gui to send commands to the controller

    roslaunch ur10_e_moveit_config moveit_rviz.launch config:=true

#### Setup Assistant for UR10e Model and MoveIt! Configuration

    roslaunch moveit_setup_assistant setup_assistant.launch


## Official Intel Realsense D415 ROS-SDK 
https://github.com/IntelRealSense/realsense-ros

Start Realsense viewer
    
    realsense-viewer

Start Ros Node with mapping depth to rgb
    
    roslaunch realsense2_camera rs_rgbd.launch


## Official Universal Robots ROS Driver
https://github.com/UniversalRobots/Universal_Robots_ROS_Driver

To actually start the robot driver use one of the existing launch files

    roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=192.168.56.101

If you calibrated the robot, pass that calibration to the launch file:

    roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=192.168.56.101 kinematics_config:=$(rospack find ur_calibration)/etc/ur10e_example_calibration.yaml

If the parameters in that file don't match the ones reported from the robot, the driver will output an error during startup, but will remain usable.

#### Start URCap
Once the robot driver is started, load the program on the robot panel and execute it. 
To control the robot using ROS, use the action server on /scaled_pos_traj_controller/follow_joint_trajectory

Use this with any client interface such as MoveIt! or simply the rqt_joint_trajectory_controller gui:

    rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller

Additionally, you can use MoveIt! to control the robot. For setting up the MoveIt! nodes to allow motion planning run:

    roslaunch ur10_e_moveit_config ur10_e_moveit_planning_execution.launch limited:=true

For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

    roslaunch ur10_e_moveit_config moveit_rviz.launch config:=true



## RtabMap_ros - RGBD SLAM
https://github.com/introlab/rtabmap_ros

![SLAM MAP](/images/os_tracking_results.gif)


Hand-held RGB-D SLAM For Intel Realsense D415 

Method 1:
    
    roslaunch realsense2_camera rs_camera.launch ( align_depth:=true)()
    
    roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" rviz:=true rtabmapviz:=false     depth_topic:=/camera/depth/image_rect_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info

Method 2:

    roslaunch realsense2_camera rs_aligned_depth.launch

    roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" rviz:=true rtabmapviz:=false depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info

It seems that Method 2 can give better results. Due to the poor quality of the depth data, rtabmap frequently gets lost. You may need to try many times to get a good map.
Alternative: https://github.com/felixendres/rgbdslam_v2


## C++ Integration
https://github.com/jmiseikis/ur5_inf3480


## PCL
http://wiki.ros.org/pcl

http://wiki.ros.org/pcl/Overview




## ROS help
Dislay the current ros graph

    rosrun rqt_graph rqt_graph




