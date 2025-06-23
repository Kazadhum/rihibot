"""imu controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from geometry_msgs.msg import Quaternion, Vector3
from controller import Robot
import rospy
from sensor_msgs.msg import Imu
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Enable sensors
accelerometer = robot.getDevice("accelerometer")
accelerometer.enable(timestep)
gyroscope = robot.getDevice("gyro")
gyroscope.enable(timestep)


rospy.init_node(name="data_handler", anonymous=True)
imu_pub = rospy.Publisher(name="imu", data_class=Imu, queue_size=10)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    
    # Read the accel and gyro sensors:
    gyro_values = gyroscope.getValues()
    accel_values = accelerometer.getValues()
    
    imu_msg = Imu()

    # Header
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = "accelerometer"
    # Orientation (ignore)
    imu_msg.orientation = Quaternion(0, 0, 0, 1)
    imu_msg.orientation_covariance = [0] * 9
    imu_msg.orientation_covariance[0] = -1  # Ignore orientation
    # Linear Acceleration
    imu_msg.linear_acceleration = Vector3(*accel_values)
    imu_msg.linear_acceleration_covariance = [0] * 9
    # Angular Velocity
    imu_msg.angular_velocity = Vector3(*gyro_values)
    imu_msg.angular_velocity_covariance = [0] * 9

    imu_pub.publish(imu_msg)

# Enter here exit cleanup code.
