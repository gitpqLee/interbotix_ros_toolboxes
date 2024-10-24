# /bin/bash

source /home/xbot/Desktop/xbot/catkin_ws/devel/setup.bash

roslaunch interbotix_xsarm_perception start-robot.launch dof:=6
