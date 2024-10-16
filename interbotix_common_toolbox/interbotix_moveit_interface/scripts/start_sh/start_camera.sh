# /bin/bash

source /home/xbot/Desktop/xbot/catkin_ws/devel_isolated/setup.bash

roslaunch interbotix_xsarm_perception start-camera.launch use_actual:=true dof:=6
# roslaunch interbotix_xsarm_perception start-hw.launch use_fake:=true dof:=6
