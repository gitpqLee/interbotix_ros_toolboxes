# /bin/bash

source /home/xbot/Desktop/xbot/catkin_ws/devel_isolated/setup.bash

roslaunch interbotix_xsarm_perception start-robot.launch dof:=6

python3 /home/xbot/Desktop/xbot/catkin_ws/src/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/scripts/test_sub.py