# dual-arm-kortex-ros
dual arm kortex ros

Steps to run the setup to:
1.) catkin_make the workspace
  >cd dual_arm_Kortex_ros_ws/
  >catkin_make
  >source devel/setup.sh

2.) Run Gazebo simulator and controllers
  from directory dual_arm_Kortex_ros_ws/
  >source devel/setup.sh
  >roslaunch kortex_two_arms gazebo_move_grp.launch 

3.) Run Move group node
  in new terminal
  from directory dual_arm_Kortex_ros_ws/
  >source devel/setup.sh
  >rosrun kortex_two_arms move_grp.py 
  
