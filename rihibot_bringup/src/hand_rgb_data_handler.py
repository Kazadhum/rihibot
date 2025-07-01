#!/usr/bin/env python3

from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from webots_ros.srv import get_float, set_int, camera_get_info
import numpy as np


class CameraDataHandler:
    def __init__(self):

        rospy.wait_for_service(service="/robot/get_basic_time_step")
        robot_get_basic_time_step_srv = rospy.ServiceProxy(
            name="/robot/get_basic_time_step", service_class=get_float
        )
        rospy.wait_for_service(service="/hand_rgb_link/enable")
        hand_rgb_enable_srv = rospy.ServiceProxy(
            name="/hand_rgb_link/enable", service_class=set_int
        )
        rospy.wait_for_service(service="/hand_rgb_link/get_info")
        hand_rgb_get_info_srv = rospy.ServiceProxy(
            name="/hand_rgb_link/get_info", service_class=camera_get_info
        )

        timestep = int(robot_get_basic_time_step_srv().value)

        hand_rgb_enable_srv(value=timestep)

        # Camera Info
        info_resp = hand_rgb_get_info_srv()
        cam_w = info_resp.width
        cam_h = info_resp.height
        cam_fov = info_resp.Fov

        fx = fy = cam_w / (2.0 * np.tan(cam_fov / 2.0))
        cx = cam_w / 2.0
        cy = cam_h / 2.0

        rospy.init_node(name="hand_rgb_data_handler", anonymous=True)

        img_sub = rospy.Subscriber(
            name="/hand_rgb_link/image", data_class=Image, callback=self.img_recv
        )
        self.image_pub = rospy.Publisher(
            name="hand_rgb/image_raw", data_class=Image, queue_size=10
        )
        self.cam_info_pub = rospy.Publisher(
            name="hand_rgb/camera_info", data_class=CameraInfo, queue_size=10
        )
        self.bridge = CvBridge()

        self.current_img_msg = None

        # Static CameraInfo message
        self.cam_info_msg = CameraInfo()
        self.cam_info_msg.header = Header()
        self.cam_info_msg.header.stamp = rospy.Time.now()
        self.cam_info_msg.header.frame_id = "hand_rgb_optical_frame"
        self.cam_info_msg.width = cam_w
        self.cam_info_msg.height = cam_h
        self.cam_info_msg.distortion_model = "plumb_bob"
        self.cam_info_msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.cam_info_msg.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.cam_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.cam_info_msg.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

    def img_recv(self, msg):

        new_msg = msg
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = "hand_rgb_optical_frame"
        new_msg.header = header

        self.current_img_msg = new_msg



if __name__ == "__main__":
    image_handler = CameraDataHandler()

    rate = rospy.Rate(hz=10)

    while not rospy.is_shutdown():

        if image_handler.current_img_msg is not None:
            image_handler.image_pub.publish(image_handler.current_img_msg)
            image_handler.cam_info_pub.publish(image_handler.cam_info_msg)
        
        rospy.Rate.sleep(rate)
