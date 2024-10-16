#!/usr/bin/env python3

import os
os.environ["ROS_NAMESPACE"] = "/wx250s"

import rospy
import math
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import sys

import tf
from geometry_msgs.msg import PointStamped, Pose

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from six.moves import input
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from moveit_commander.conversions import pose_to_list

from interbotix_xs_modules.arm import InterbotixManipulatorXS

import time
last_time = 0
last_position = [0, 0, 0]

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

def need_move(coord1, coord2, threshold=0.001):
    # Helper function to calculate the Euclidean distance between two points
    def euclidean_distance(point1, point2):
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(point1, point2)))
    
    # Calculate the distance between the two points
    distance = euclidean_distance(coord1, coord2)

    # Check if the distance is greater than the threshold
    return distance > threshold

def pose_transform(position=[], orientation=[]):
    pose = Pose()
    if position:
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
    else:
        # TODO:
        # if there is no postion, move to home pose
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 0

    if orientation:
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
    else:
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1

    return pose

def coordinate_transform(x_axis, y_axis, z_axis):
    # z_axis = z_axis - 0.1
    rospy.loginfo("Point in camera_color_optical_frame: %s %s %s", x_axis, y_axis, z_axis)

    listener = tf.TransformListener()
    # Define the point in the camera frame
    point_camera_frame = PointStamped()
    point_camera_frame.header.frame_id = "camera_color_optical_frame"  # Replace with your camera frame ID
    point_camera_frame.header.stamp = rospy.Time(0)

    point_camera_frame.point.x = x_axis
    point_camera_frame.point.y = y_axis 
    point_camera_frame.point.z = z_axis

    # Wait for the listener to get the first transform
    listener.waitForTransform("wx250s/base_link", "camera_color_optical_frame", rospy.Time(0), rospy.Duration(4.0))
    # Transform the point to the robot base link frame
    point_robot_base_frame = listener.transformPoint("wx250s/base_link", point_camera_frame)

    rospy.loginfo("Point in robot base link frame: %s", point_robot_base_frame.point)

    position_list = [float(point_robot_base_frame.point.x), float(point_robot_base_frame.point.y), float(point_robot_base_frame.point.z)]
    orientation_list = [0, 0, 0, 1]
    pose_robot_base_frame = pose_transform(position_list, orientation_list)

    return pose_robot_base_frame

def pose_callback(msg):
    global last_time
    global last_position
    current_time = time.time()
    
    # sampling every 20s
    if current_time - last_time >= 20:
        python_list = msg.data.tolist()
        rospy.loginfo("Received data converted to list: %s", python_list)
        pose_goal = coordinate_transform(python_list[0], python_list[1], python_list[2])
        rospy.loginfo(pose_goal)

        target_position = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z]

        # 0: grip
        # 1: iron
        exec = python_list[3]

        # 1. if the next pose is same to current one, do not move
        if need_move(last_position, target_position):
        # 2. if move:
        #   2.1 move to the 10 cm above of the item
            target_position[2] += 0.1 
            bot.arm.set_ee_pose_components(x = target_position[0], y = target_position[1], z = target_position[2], pitch=0.5)
        #       pose_goal.position.z += 0.1
            if exec == 0:
            #   grip
            #   2.2 open the gripper
                bot.gripper.open()
            #   2.3 move to the item real coordinates
                target_position[2] -= 0.1
                bot.arm.set_ee_pose_components(x = target_position[0], y = target_position[1], z = target_position[2], pitch=0.5) 
            #       pose_goal.position.z -= 0.1
            #   2.4 close the gripper
                bot.gripper.close()
            #   2.5 move to the pre-defined coorinnate
                bot.arm.go_to_home_pose()
                bot.gripper.open()
            else:
            # ironing
                # default 90 degree
                bot.arm.set_ee_cartesian_trajectory(pitch=1.5)
                bot.arm.set_ee_cartesian_trajectory(z=-0.1)

                # move back/forth
                bot.arm.set_ee_cartesian_trajectory(x=0.1)
                for i in range(2):
                    bot.arm.set_ee_cartesian_trajectory(x=-0.2)
                    bot.arm.set_ee_cartesian_trajectory(x=0.2)
                
                bot.arm.go_to_home_pose()

        bot.arm.go_to_sleep_pose()
        last_position = target_position
        last_time = current_time

def go_to_pose_goal(group, pose_goal):

    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    current_pose = group.get_current_pose().pose

    return all_close(pose_goal, current_pose, 0.01)

def main():
    # moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node("test_subscriber")

    # rospy.get_namespace().strip("/")

    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()

    # group_name = "interbotix_arm"  # Change this to your robot arm's move group
    # global move_group
    # move_group = moveit_commander.MoveGroupCommander(group_name)

    global bot
    bot = InterbotixManipulatorXS("wx250s", "arm", "gripper")

    rospy.Subscriber("test_topic", numpy_msg(Floats), callback=pose_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
    