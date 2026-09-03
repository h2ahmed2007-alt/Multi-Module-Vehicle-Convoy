
#import the main file which does the mathematical calculations 
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

        # Get the values from yaml  file
        drive_type = self.get_parameter("drive_type").get_parameter_value().string_value
        L = self.get_parameter("track_width").get_parameter_value().double_value
        W = self.get_parameter("wheelbase").get_parameter_value().double_value
        R = self.get_parameter("wheel_radius").get_parameter_value().double_value

        # Create instance for each subclass
        if drive_type == "diff_drive":
            self.kinematics = DiffDriveKinematics(L, W, R)
        elif drive_type == "mecanum":
            self.kinematics = MecanumKinematics(L, W, R)
        elif drive_type == "three_wheel_omni":
            self.kinematics = ThreeWheelOmniKinematics(L, W, R)
        else:
            self.kinematics = FourWheelOmniKinematics(L, W, R)

        # Create wheel setpoint topic to publish wheel speeds on
        self.wheelsetpnt_pub = self.create_publisher(
            Float64MultiArray,  # message type
            '/wheel_setpoints',
            10
        )

        # Subscribe to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmdvelcallback,
            10
        )

    # Callback function 
    def cmdvelcallback(self, msg: Twist):
        # Receives velocities
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # Pass these values to inverse() to calc motor speeds
        wheel_speeds = self.kinematics.inverse(vx, vy, wz)

        # Convert inverse() return to Python list
        setpoint_msg = Float64MultiArray()
        setpoint_msg.data = wheel_speeds.tolist()

        # Publish the setpoint message
        self.wheelsetpnt_pub.publish(setpoint_msg)


# Main 
def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()