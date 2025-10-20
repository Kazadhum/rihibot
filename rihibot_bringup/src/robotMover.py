#!/usr/bin/env python3

import rospy
from webots_ros.srv import set_float

def getJointsServices(joints: list) -> dict:
    service_dict = {}

    # first wait for the services to be available
    for joint in joints:
        rospy.wait_for_service(f"/{joint}/set_velocity")
        rospy.wait_for_service(f"/{joint}/set_position")
    
    # now create the service dictionary
    for joint in joints:
        service_dict[f"{joint}_set_velocity"] = rospy.ServiceProxy(name=f"/{joint}/set_velocity", service_class=set_float)
        service_dict[f"{joint}_set_position"] = rospy.ServiceProxy(name=f"/{joint}/set_position", service_class=set_float)

    return service_dict

def main():
    joints: list = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]

    service_dict = getJointsServices(joints)

    # set velocities of all joints to 0.1
    for joint in joints:
        service_dict[f"{joint}_set_velocity"](value=0.1)

    rospy.sleep(duration=5)

    service_dict["shoulder_lift_joint_set_position"](value=-0.5)
    service_dict["shoulder_pan_joint_set_position"](value=-1.57)

    rospy.sleep(duration=10)


if __name__ == "__main__":
    main()
