"""supervisor_joints_pub controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from webots_ros.msg import Float64Stamped
from webots_ros.srv import get_float, get_floatRequest
from controller import Supervisor
from robotController import RobotController

print("Hi")

# create the Robot instance.
supervisor = Supervisor()

controller = RobotController()

rospy.init_node(name="joints_pub", anonymous=True)

print("Hey")

# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())

joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

controller.enable_joint_position_sensors()

print("Waiting....")
for joint in joint_names:
    rospy.wait_for_message(topic=f"/{joint}_sensor/value", topic_type=Float64Stamped)

print("Yay!")

joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)

# Set low velocities for joints
vel_arr = [0.1] * len(controller.joint_names)
controller.set_joint_velocities(vel_arr)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while supervisor.step(timestep) != -1 and not rospy.is_shutdown():
    
    # Publish joint states
    joint_msg = JointState()
    joint_msg.header = Header()
    
    sim_time = supervisor.getTime()
    secs = int(sim_time)
    nsecs = int((sim_time - secs) * 1e9)
    
    joint_msg.header.stamp.secs = secs
    joint_msg.header.stamp.nsecs = nsecs
    joint_msg.name = joint_names
    joint_msg.position = [controller.joint_position_dict[joint] for joint in joint_names]
    
    joint_pub.publish(joint_msg)

# Enter here exit cleanup code.