"""supervisor_ur10e_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import random
import time
import rospy
from webots_ros.msg import Float64Stamped
from controller import Supervisor   # pyright: ignore[reportMissingImports]
from robotController import RobotController
import numpy as np

# create the Robot instance.
supervisor = Supervisor()  # pyright: ignore[reportUndefinedVariable]

rospy.init_node(name="ur10e_control_node", anonymous=True)

# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())

controller = RobotController()

time.sleep(5)

# Create position sequence for movement
position_sequence = []
# Initial position
position_sequence.append([0.0] * len(controller.joint_names))

 # Set low velocities for joints
vel_arr = [0.05] * len(controller.joint_names)
controller.set_joint_velocities(vel_arr)

random.seed(a=10)
for i in range(10):
    random_pos = [random.uniform(-2, 2) for k in range(len(controller.joint_names))]
    position_sequence.append(random_pos)

print("waiting...")
for joint_name in controller.joint_names:
    rospy.wait_for_message(f"/{joint_name}_sensor/value", topic_type=Float64Stamped)
print("They're here!")

# Initialize on_target flag and target position idx
on_target = False
target_position_idx = 0

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while supervisor.step(timestep) != -1 and not rospy.is_shutdown():
    print(target_position_idx)
    if target_position_idx >= len(position_sequence):
        exit(code=0)

    pos_arr = position_sequence[target_position_idx]
    controller.set_joint_positions(pos_arr)

    # Check if on target
    joint_sensor_arr = [
        controller.joint_position_dict[joint]
        for joint in controller.joint_names
    ]

    if np.linalg.norm(np.array(joint_sensor_arr) - np.array(pos_arr)) < 0.005:
        print("Target reached!")
        on_target = True
        target_position_idx += 1

    if not on_target:
        print("Not there yet!")

# Enter here exit cleanup code.
