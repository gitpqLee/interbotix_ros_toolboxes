import rospy
import tf
from geometry_msgs.msg import PointStamped

rospy.init_node('coordinate_transformer')
listener = tf.TransformListener()
# Define the point in the camera frame
point_camera_frame = PointStamped()
point_camera_frame.header.frame_id = "camera_color_optical_frame"  # Replace with your camera frame ID
point_camera_frame.header.stamp = rospy.Time(0)

point_camera_frame.point.x = 0.36  # Replace with your object's x coordinate
point_camera_frame.point.y = -0.213  # Replace with your object's y coordinate
point_camera_frame.point.z = 1.344  # Replace with your object's z coordinate

# Wait for the listener to get the first transform
listener.waitForTransform("wx250s/base_link", "camera_color_optical_frame", rospy.Time(0), rospy.Duration(4.0))
# Transform the point to the robot base link frame
point_robot_base_frame = listener.transformPoint("wx250s/base_link", point_camera_frame)

print("Point in robot base link frame:", point_robot_base_frame.point)