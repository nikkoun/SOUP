# SOUP




![ROS_Diagram](/images/ros_diagram.png)




MoveIt Controller and model for Universal Robots (included in ROS Industrial)
[]  https://github.com/ros-industrial/universal_robot




Official Intel Realsense D415 ROS-SDK 
[]  https://github.com/IntelRealSense/realsense-ros

Official Universal Robots ROS Driver
[] https://github.com/UniversalRobots/Universal_Robots_ROS_Driver

PCL
http://wiki.ros.org/pcl
http://wiki.ros.org/pcl/Overview









# Start Realsense viewer
realsense-viewer


# Start Ros Node for mapping depth to rgb
roslaunch realsense2_camera rs_rgbd.launch


# Start gazebo and load arm model
roslaunch ur_e_gazebo ur10e.launch
# Load MoveIt contrrollers for arm
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true
# Starts rviz gui to send commands to the controller
roslaunch ur5_moveit_config moveit_rviz.launch config:=true


# Opens the current ros graph
rosrun rqt_graph rqt_graph

# Topic where Joints state is published  
rostopic echo /joint_states -n 1
# transforms for move commands
rostopic echo /tf -n 1
rostopic echo /tf_static -n 1
