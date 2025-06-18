#!/usr/bin/env python3

import rospy
from controller import Robot, Supervisor
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# Create ROS publisher
rospy.init_node(name="joint_state_publisher", anonymous=True)
joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

sensors = []
for joint in joint_names:
    motor = robot.getDevice(joint)
    sensor = motor.getPositionSensor()
    sensor.enable(timestep)
    sensors.append(sensor)

rate = rospy.Rate(1000/timestep)

while robot.step(timestep) != -1 and not rospy.is_shutdown():
    joint_msg = JointState()
    joint_msg.header = Header()
    joint_msg.header.stamp = rospy.Time.now()
    joint_msg.name = joint_names
    joint_msg.position = [sensor.getValue() for sensor in sensors]

    joint_pub.publish(joint_msg)
    rate.sleep()