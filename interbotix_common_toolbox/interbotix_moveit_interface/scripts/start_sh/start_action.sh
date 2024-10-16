# /bin/bash

source /home/xbot/Desktop/xbot/catkin_ws/devel_isolated/setup.bash

# keyboard 66
# orange 49
# banana 46
# apple 47

target_object_id=49
actiion=0

if [ $actiion -eq 0 ]; then
    # grip: pass action id and target id
    python3 /home/xbot/Desktop/xbot/catkin_ws/src/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/scripts/infer_pipeline/yolo-detection-pipeline.py 0 $target_object_id
else
    # ironing: pass action id only
    python3 /home/xbot/Desktop/xbot/catkin_ws/src/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/scripts/infer_pipeline/yolo-detection-pipeline.py 1
fi