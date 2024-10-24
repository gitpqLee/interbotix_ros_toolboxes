# /bin/bash

source /home/xbot/Desktop/xbot/catkin_ws/devel/setup.bash

# 0: grip
# 1: iron
actiion=0

# keyboard 66
# orange 49
# banana 46
# apple 47
target_object_id=46

# book 73
# phone 67
book_id=67

if [ $actiion -eq 0 ]; then
    # grip: pass target id
    python3 /home/xbot/Desktop/xbot/catkin_ws/src/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/scripts/infer_pipeline/yolo-detection-pipeline.py 0 $target_object_id
else
    # ironing: 
    python3 /home/xbot/Desktop/xbot/catkin_ws/src/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/scripts/infer_pipeline/yolo-detection-pipeline.py 1 $book_id
fi