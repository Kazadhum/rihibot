#!/usr/bin/env python3

""" A simple controller for the UR5e"""

from pprint import pprint
import random
import time

import numpy as np
import rospy
from webots_ros.msg import Float64Stamped
from webots_ros.srv import get_float, set_float, set_int
from robotController import RobotController

def main():
    """Main control loop. Goes to a series of random positions at random, moving the joints with constant velocity."""
    controller = RobotController()
    controller.enable_joint_position_sensors()

    # Set low velocities for joints
    vel_arr = [0.1] * len(controller.joint_names)
    controller.set_joint_velocities(vel_arr)

    # Create position sequence for movement
    position_sequence = []
    # Initial position
    position_sequence.append([0.0] * len(controller.joint_names))

    for i in range(10):
        random_pos = [random.uniform(-2, 2) for k in range(len(controller.joint_names))]
        position_sequence.append(random_pos)

    # Return to start position
    position_sequence.append([0.0] * len(controller.joint_names))

    for pos_arr in position_sequence:
        controller.set_joint_positions(pos_arr)

        # Check if the joint positions are on target
        on_target: bool = False

        while not on_target:
            joint_sensor_arr = [
                controller.joint_position_dict[joint]
                for joint in controller.joint_names
            ]

            if np.linalg.norm(np.array(joint_sensor_arr) - np.array(pos_arr)) < 0.005:
                print("Target reached!")
                on_target = True

            if pos_arr == position_sequence[-1] and on_target:
                break
            
        time.sleep(3)

    rospy.spin()


if __name__ == "__main__":
    main()
