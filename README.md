# dual-arm-kortex-ros
dual arm kortex ros

Note: You may need to install dependancies first. See below.

Steps to run the setup:
1.) catkin_make the workspace
  ```
  cd dual_arm_Kortex_ros_ws/
  catkin_make
  source devel/setup.sh
  ```

2.) Run Gazebo simulator and controllers
  from directory dual_arm_Kortex_ros_ws/
  ```
  source devel/setup.sh
  roslaunch kortex_two_arms gazebo_move_grp.launch 
  ```

3.) Run Move group node
  in new terminal
  from directory dual_arm_Kortex_ros_ws/
  ```
  source devel/setup.sh
  rosrun kortex_two_arms move_grp.py 
  ```
 
 Image Topic names
 ```
 /cam/camera/image_raw
 /cam/camera1/image_raw1
 ```
 
 # Dependancy installation steps for Ubuntu 16.04/ROS Kinetic
 1.) Install moveit package:
 ```
 sudo apt-get install ros-kinetic-moveit
 ```
 2.) Install this kinematics package:
 ```
 sudo apt-get install ros-kinetic-trac-ik-kinematics-plugin
 ```
 3.) Install these controller packages:
 ```
 sudo apt-get install ros-kinetic-effort-controllers
 sudo apt-get install ros-kinetic-joint-trajectory-controller
 sudo apt-get install ros-kinetic-joint-state-controller
 ```
 
 
