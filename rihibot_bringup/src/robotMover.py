#!/usr/bin/env python3

import rospy
from webots_ros.srv import set_float
import math

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
        service_dict[f"{joint}_set_velocity"](value=0.3)

    rospy.sleep(duration=5)

    service_dict["shoulder_pan_joint_set_position"](value=-0.13316862192716733)
    service_dict["shoulder_lift_joint_set_position"](value=-1.4320426512613473)
    service_dict["elbow_joint_set_position"](value=2.0900317792632093)
    service_dict["wrist_1_joint_set_position"](value=-3.7271506176338907)
    service_dict["wrist_2_joint_set_position"](value=-1.4063863112570307)
    service_dict["wrist_3_joint_set_position"](value=math.pi)

    rospy.sleep(duration=50)

    service_dict["shoulder_pan_joint_set_position"](value=math.pi)

    rospy.sleep(duration=20)

    service_dict["shoulder_pan_joint_set_position"](value=-0.13316862192716733)

    rospy.sleep(duration=20)

    service_dict["shoulder_pan_joint_set_position"](value=0.027401669256310976)
    service_dict["shoulder_lift_joint_set_position"](value=-0.8206538142877338)
    service_dict["elbow_joint_set_position"](value=1.140049067402696)
    service_dict["wrist_1_joint_set_position"](value=-3.264987431705792)
    service_dict["wrist_2_joint_set_position"](value=-1.8495254083383912)
    service_dict["wrist_3_joint_set_position"](value=math.pi)

    rospy.sleep(duration=50)

    service_dict["shoulder_pan_joint_set_position"](value=-0.3410373358396919)
    service_dict["shoulder_lift_joint_set_position"](value=-0.7578219612159379)
    service_dict["elbow_joint_set_position"](value=1.124341104134747)
    service_dict["wrist_1_joint_set_position"](value=-3.470587217590724)
    service_dict["wrist_2_joint_set_position"](value=-1.1116002005951884)
    service_dict["wrist_3_joint_set_position"](value=math.pi)

    rospy.sleep(duration=50)

    service_dict["shoulder_pan_joint_set_position"](value=0.03490658503988659)
    service_dict["shoulder_lift_joint_set_position"](value=0.14573499254152653)
    service_dict["elbow_joint_set_position"](value=-0.8190830179609389)
    service_dict["wrist_1_joint_set_position"](value=-2.4822072621863356)
    service_dict["wrist_2_joint_set_position"](value=-1.820902008605684)
    service_dict["wrist_3_joint_set_position"](value=math.pi)






if __name__ == "__main__":
    main()
