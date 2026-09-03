
#import the main file which does the mathematical calculations 
import Kinematics.py
from Kinematics import (
    DiffDriveKinematics,
    MecanumKinematics,
    ThreeWheelOmniKinematics,
    FourWheelOmniKinematics,
)

# Include ROS 2 libraries
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist  # /cmd_vel subscriber
from std_msgs.msg import Float64MultiArray  # /wheel_setpoints publisher


class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')

        # Declare the parameters of the wheels
        self.declare_parameter("drive_type")
        self.declare_parameter("track_width")
        self.declare_parameter("wheelbase")
        self.declare_parameter("wheel_radius")

        # get the values from yamll file 
        drive_type = self.get_parameter("drive_type").get_parameter_value().string_value
        L = self.get_parameter("track_width").get_parameter_value().double_value
        W = self.get_parameter("wheelbase").get_parameter_value().double_value
        R = self.get_parameter("wheel_radius").get_parameter_value().double_value

        # create instance for each subclass 
        if drive_type == "diff_drive":
            self.kinematics = DiffDriveKinematics(L, W, R)
        elif drive_type == "mecanum":
            self.kinematics = MecanumKinematics(L, W, R)
        elif drive_type == "three_wheel_omni":
            self.kinematics = ThreeWheelOmniKinematics(L, W, R)
        else:
            self.kinematics = FourWheelOmniKinematics(L, W, R)
        