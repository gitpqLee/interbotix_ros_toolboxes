#!/usr/bin/env python3

import os
os.environ["ROS_NAMESPACE"] = "/wx250s"

import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np

if __name__ == '__main__':
    rospy.init_node("test_publisher_node")
    rospy.loginfo("publisher started")

    pub = rospy.Publisher("test_topic", numpy_msg(Floats), queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # 0: grip
        # 1: iron
        array_data = np.array([-0.267, -0.325, 1.35, 0], dtype=np.float32)
        pub.publish(array_data)
        rate.sleep()