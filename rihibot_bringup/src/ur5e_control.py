#!/usr/bin/env python3

""" A simple controller for the UR5e"""

from pprint import pprint
from webots_ros.srv import set_float, get_float, set_int
from webots_ros.msg import Float64Stamped
import rospy


class RobotController:
    def __init__(self) -> None:

        rospy.init_node(name="ur5e_controller")

        # Wait for services to be available
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        service_types = ["get_velocity", "set_velocity", "set_position"]

        services_to_wait_for = []
        for joint in self.joint_names:
            for service_type in service_types:
                services_to_wait_for.append(f"/{joint}/{service_type}")
            # Also wait for joint position sensor enabling services
            services_to_wait_for.append(f"/{joint}_sensor/enable")

        # Also wait for getBasicTimeStep service
        services_to_wait_for.append("/robot/get_basic_time_step")
        robot_get_basic_time_step_srv = rospy.ServiceProxy(
            name="/robot/get_basic_time_step", service_class=get_float
        )

        self.basic_time_step = int(robot_get_basic_time_step_srv().value)

        for srv in services_to_wait_for:
            rospy.wait_for_service(service=srv)

        self.srv_proxy_dict = {}
        for joint in self.joint_names:
            self.srv_proxy_dict[f"{joint}_get_velocity"] = rospy.ServiceProxy(
                name=f"/{joint}/get_velocity", service_class=get_float
            )
            self.srv_proxy_dict[f"{joint}_set_velocity"] = rospy.ServiceProxy(
                name=f"/{joint}/set_velocity", service_class=set_float
            )
            self.srv_proxy_dict[f"{joint}_set_position"] = rospy.ServiceProxy(
                name=f"/{joint}/set_position", service_class=set_float
            )
            self.srv_proxy_dict[f"{joint}_sensor_enable"] = rospy.ServiceProxy(
                name=f"/{joint}_sensor/enable", service_class=set_int
            )

        # Configure subscribers to the joint position sensors
        self.joint_position_dict = {}
        self.joint_sensor_subscriber_dict = {}
        for joint in self.joint_names:
            self.joint_sensor_subscriber_dict[joint] = rospy.Subscriber(
                name=f"/{joint}_sensor/value",
                data_class=Float64Stamped,
                callback=self.pos_recv_callback,
                callback_args=joint,
            )

        init_joint_pos_arr = [0.0] * 6
        init_joint_vel_arr = [0.0] * 6

        self.set_joint_positions(joint_pos_array=init_joint_pos_arr)
        self.set_joint_velocities(joint_vel_array=init_joint_vel_arr)

    def enable_joint_position_sensors(self):
        """Turns on every joint position sensor so the data is published via ROS message."""
        for joint in self.joint_names:
            self.srv_proxy_dict[f"{joint}_sensor_enable"](value=self.basic_time_step)

    def pos_recv_callback(self, msg, joint):
        """Simple callback function to record the last joint position data from the ros message"""
        self.joint_position_dict[joint] = msg.data

    def set_joint_positions(self, joint_pos_array):
        """Receives an array of joint positions and calls the appropriate services to set them to the specified values."""
        for i in range(len(self.joint_names)):
            joint = self.joint_names[i]
            self.srv_proxy_dict[f"{joint}_set_position"](value=joint_pos_array[i])

    def set_joint_velocities(self, joint_vel_array):
        """Receives an array of joint velocities and calls the appropriate services to set them to the specified values."""
        for i in range(len(self.joint_names)):
            joint = self.joint_names[i]
            self.srv_proxy_dict[f"{joint}_set_velocity"](value=joint_vel_array[i])


def main():
    controller = RobotController()
    controller.enable_joint_position_sensors()

    rospy.spin()


if __name__ == "__main__":
    main()
