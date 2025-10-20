#!/usr/bin/env python3

""" A simple controller for the UR5e"""

from pprint import pprint
import random
import time

import message_filters
import numpy as np
from rosgraph_msgs.msg import Clock
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from webots_ros.msg import Float64Stamped
from webots_ros.srv import get_float, set_float, set_int
from robotController import RobotController
from message_filters import ApproximateTimeSynchronizer



class JointRepublisher:
    def __init__(self) -> None:

        joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        subscribers_arr = [message_filters.Subscriber(f"/{joint}_sensor/value", Float64Stamped) for joint in joint_names]

        self.joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=50)
    
        ats = ApproximateTimeSynchronizer(
            [subscriber for subscriber in subscribers_arr],
            queue_size=50,
            slop=0.010
        )

        ats.registerCallback(self.callback)

    def callback(self, msg1, msg2, msg3, msg4, msg5, msg6):
    
        stamp = msg1.header.stamp
    
        joint_msg = JointState()
        joint_msg.header = Header()
        joint_msg.header.stamp = stamp
        joint_msg.position = [
            msg1.data,
            msg2.data,
            msg3.data,
            msg4.data,
            msg5.data,
            msg6.data,
        ]
    
        self.joint_pub.publish()

def main():
    rospy.init_node(name="joint_republisher", anonymous=True)
    republisher = JointRepublisher()
    rospy.spin()
        

if __name__ == "__main__":
    main()
