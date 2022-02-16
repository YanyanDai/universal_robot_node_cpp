# universal_robot_node_cpp  
# move the robot from home to up, then to home  
  
$ cd ur/catkin_ws/src/ur_modern_driver/src  
$ gedit ur_test02.cpp  

Add content in CMakeLists.txt  
Add moveit_ros_planning_interface package in find_package of CMakeLists.txt  
Add moveit_ros_planning_interface in catkin_package part of CMakeLists.txt  
Add content in package.xml  
  
Run command  
One terminal:  
$ cd ur/catkin_ws  
$ source devel/setup.bash  
$ roslaunch ur_gazebo ur3.launch  
 Two terminal  
$ cd ur/catkin_ws  
$ source devel/setup.bash   
$ roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch sim:=true  
 Three terminal  
$ cd ur/catkin_ws  
$ source devel/setup.bash  
$ roslaunch ur3_moveit_config moveit_rviz.launch config:=true  
 Four terminal   
$ cd ur/catkin_ws  
$ source devel/setup.bash  
$ rosrun ur_modern_driver ur_test02   
