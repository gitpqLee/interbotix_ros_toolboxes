#!/usr/bin/env python
# coding: utf-8

# In[ ]:


# -*- coding: utf-8 -*-
import os
import sys
os.environ['YOLO_VERBOSE'] = str(False)
os.environ['ROS_NAMESPACE'] = "/wx250s" 

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

import rospy
from std_msgs.msg import String

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

global action
global target_object_id

# Check the number of arguments passed
num_args = len(sys.argv) - 1
if num_args == 0:
    print("No action id and target id arguments were passed. Exiting...")
    sys.exit(1)
elif num_args == 1:
    # One additional argument passed
    if int(sys.argv[1]) == 0:
        print(f"Action code is 0. Gipping, but no target id is passed, Exiting...")
        sys.exit(1)
    elif int(sys.argv[1]) == 1:
        print(f"Action code is 1. Ironing")
        action = sys.argv[1]
elif num_args == 2:
    if int(sys.argv[1]) == 0:
        print(f"Action code is 0. Gipping, the target onject id is " + str(sys.argv[2]))
        action = sys.argv[1]
        target_object_id = sys.argv[2]
else:
    print("Too many arguments passed. Exiting...")
    sys.exit(1)

# fill up your yolo .pt file
model = YOLO("/home/xbot/xbot/infer-pipeline/model/yolov8n.pt") 

'''
{0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane', 5: 'bus', 6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light', 
 10: 'fire hydrant', 11: 'stop sign', 12: 'parking meter', 13: 'bench', 14: 'bird', 15: 'cat', 16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow', 
 20: 'elephant', 21: 'bear', 22: 'zebra', 23: 'giraffe', 24: 'backpack', 25: 'umbrella', 26: 'handbag', 27: 'tie', 28: 'suitcase', 29: 'frisbee', 
 30: 'skis', 31: 'snowboard', 32: 'sports ball', 33: 'kite', 34: 'baseball bat', 35: 'baseball glove', 36: 'skateboard', 37: 'surfboard', 
 38: 'tennis racket', 39: 'bottle', 40: 'wine glass', 41: 'cup', 42: 'fork', 43: 'knife', 44: 'spoon', 45: 'bowl', 46: 'banana', 47: 'apple', 
 48: 'sandwich', 49: 'orange', 50: 'broccoli', 51: 'carrot', 52: 'hot dog', 53: 'pizza', 54: 'donut', 55: 'cake', 56: 'chair', 57: 'couch', 
 58: 'potted plant', 59: 'bed', 60: 'dining table', 61: 'toilet', 62: 'tv', 63: 'laptop', 64: 'mouse', 65: 'remote', 66: 'keyboard', 
 67: 'cell phone', 68: 'microwave', 69: 'oven', 70: 'toaster', 71: 'sink', 72: 'refrigerator', 73: 'book', 74: 'clock', 75: 'vase', 
 76: 'scissors', 77: 'teddy bear', 78: 'hair drier', 79: 'toothbrush'}
'''


### RealSense setup
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

# config.enable_stream(rs.stream.depth,  848, 480, rs.format.z16, 90)
# config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)

# config.enable_stream(rs.stream.depth,  1280, 720, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

pipe_profile = pipeline.start(config)
align = rs.align(rs.stream.color)

# ROS node
# rospy.init_node('realsense_detector_publisher', anonymous=True)
# pos_pub = rospy.Publisher('/target_pos', String, queue_size=10)
rospy.init_node('test_publisher_node', anonymous=True)
pos_pub = rospy.Publisher("test_topic", numpy_msg(Floats), queue_size=10)


def get_aligned_images():
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    depth_intri = depth_frame.profile.as_video_stream_profile().intrinsics
    color_intri = color_frame.profile.as_video_stream_profile().intrinsics
    
    # cv2.applyColorMap（
    depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.07), cv2.COLORMAP_JET)
    return depth_intri, depth_frame, color_image
 
if __name__ == '__main__':
    try:
        # while True:
        while not rospy.is_shutdown():
            depth_intri, depth_frame, color_image = get_aligned_images()
            source = [color_image]

            if int(action) == 0:
                results = model.predict(source, save=False, classes=int(target_object_id))

                for result in results:
                    boxes = result.boxes.xywh.tolist()
                    im_array = result.plot()
                    
                    for i in range(len(boxes)):
                        ux, uy = int(boxes[i][0]), int(boxes[i][1])
                        
                        # center of the bounding-box
                        dis = depth_frame.get_distance(ux, uy)
                        camera_xyz = rs.rs2_deproject_pixel_to_point(
                            depth_intri, (ux, uy), dis)

                        camera_xyz = np.round(np.array(camera_xyz), 3)
                        
                        # pos_pub.publish(str(camera_coordinate))
                        print("camera_xyz : ", camera_xyz)

                        array_data = np.array([camera_xyz[0], camera_xyz[1], camera_xyz[2], int(action)], dtype=np.float32)
                        pos_pub.publish(array_data)
                        
                        # from meter to mm
                        camera_xyz = camera_xyz * 1000

                        camera_xyz = list(camera_xyz)

                        # Draw the center point
                        cv2.circle(im_array, (ux, uy), 4, (255, 255, 255), 5)

                        cv2.putText(im_array, str(camera_xyz), (ux + 20, uy + 10), 0, 0.5,
                                    [225, 255, 255], thickness=1, lineType=cv2.LINE_AA)
            elif int(action) == 1:
                # TODO
                # QR code detection, simulate ironing
                pass
            
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', im_array)
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                pipeline.stop()
                break
                
        # ctrl-c
    except KeyboardInterrupt:
        print("Interrupted")
    # any different error
    except RuntimeError as e:
        print(e)
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()





