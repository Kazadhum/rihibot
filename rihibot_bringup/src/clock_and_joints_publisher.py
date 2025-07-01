#!/usr/bin/env python3

""" A simple controller for the UR5e"""

from pprint import pprint
import random
import time

import numpy as np
from rosgraph_msgs.msg import Clock
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from webots_ros.msg import Float64Stamped
from webots_ros.srv import get_float, set_float, set_int
from robotController import RobotController


def main():
    """Main control loop. Goes to a series of random positions at random, moving the joints with constant velocity."""
    controller = RobotController()

    # The position sensors are already enabled by ur5e_control.py; we just need to wait for the message to be available
    for joint in controller.joint_names:
        rospy.wait_for_message(topic=f"/{joint}_sensor/value", topic_type=Float64Stamped)
        print("got message!")

    joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    # clock_pub = rospy.Publisher(name="clock", data_class=Clock, queue_size=1)

    while not rospy.is_shutdown():

        # Publish joint states
        joint_msg = JointState()
        joint_msg.header = Header()
        
        sim_time = controller.robot_get_time()
        secs = int(sim_time)
        nsecs = int((sim_time - secs) * 1e9)
        
        joint_msg.header.stamp.secs = secs
        joint_msg.header.stamp.nsecs = nsecs
        joint_msg.name = controller.joint_names
        joint_msg.position = [controller.joint_position_dict[joint] for joint in controller.joint_names]
        
        joint_pub.publish(joint_msg)
        

if __name__ == "__main__":
    main()
