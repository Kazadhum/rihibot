#!/usr/bin/env python3

import math
import random
from turtle import position

from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion, Vector3
import numpy as np
from rosgraph_msgs.msg import Clock
import rospy
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
from std_msgs.msg import Header

from controller import Robot, Supervisor

robot = Robot()

timestep = int(robot.getBasicTimeStep())

hand_rgb = robot.getDevice("hand_rgb")
hand_rgb.enable(timestep)

# Camera Info
cam_w = hand_rgb.getWidth()
cam_h = hand_rgb.getHeight()
cam_fov = hand_rgb.getFov()

fx = fy = cam_w / (2.0 * np.tan(cam_fov / 2.0))
cx = cam_w / 2.0
cy = cam_h / 2.0

rospy.init_node(name="data_handler", anonymous=True)

cam_info_pub = rospy.Publisher(
    name="hand_rgb/camera_info", data_class=CameraInfo, queue_size=10
)
image_pub = rospy.Publisher(name="hand_rgb/image_raw", data_class=Image, queue_size=10)
bridge = CvBridge()

# Static CameraInfo message
cam_info_msg = CameraInfo()
cam_info_msg.width = cam_w
cam_info_msg.height = cam_h
cam_info_msg.distortion_model = "plumb_bob"
cam_info_msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
cam_info_msg.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
cam_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
cam_info_msg.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

# Clock publisher
clock_pub = rospy.Publisher(name="clock", data_class=Clock, queue_size=1)

while robot.step(timestep) != -1 and not rospy.is_shutdown():

    # Camera messages
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "hand_rgb_optical_frame"
    cam_info_msg.header = header
    cam_info_pub.publish(cam_info_msg)

    # Get image
    cam_data_raw = hand_rgb.getImage()
    img = np.frombuffer(cam_data_raw, dtype=np.uint8).reshape((cam_h, cam_w, 4))[
        :, :, :3
    ]
    img = np.flip(img, axis=0)

    ros_img = bridge.cv2_to_imgmsg(img, encoding="rgb8")
    ros_img.header = header
    image_pub.publish(ros_img)