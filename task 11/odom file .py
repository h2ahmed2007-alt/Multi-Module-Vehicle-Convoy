import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

# Import approved kinematics library
from robot_kinematics import (
    DiffDriveKinematics,
    MecanumKinematics,
    ThreeWheelOmniKinematics,
    FourWheelOmniKinematics
)


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')

        # 1. Dynamic Configuration (Parameters)
        self.declare_parameter('drive_type', 'mecanum')
        self.declare_parameter('track_width', 0.5)
        self.declare_parameter('wheelbase', 0.5)
        self.declare_parameter('wheel_radius', 0.1)

        drive_type = self.get_parameter('drive_type').value
        L = self.get_parameter('track_width').value
        W = self.get_parameter('wheelbase').value
        R = self.get_parameter('wheel_radius').value

        # Select drive kinematics class based on parameter
        if drive_type == 'diff_drive':
            self.kinematics = DiffDriveKinematics(L, W, R)
        elif drive_type == 'mecanum':
            self.kinematics = MecanumKinematics(L, W, R)
        elif drive_type == 'three_omni':
            self.kinematics = ThreeWheelOmniKinematics(L, W, R)
        elif drive_type == 'four_omni':
            self.kinematics = FourWheelOmniKinematics(L, W, R)
        else:
            self.get_logger().error(f"Unsupported drive type: {drive_type}")
            raise ValueError(f"Unknown drive type: {drive_type}")

        # Initial pose coordinates (X, Y, theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Record initial time for dt calculation
        self.last_time = self.get_clock().now()

        # 2. Feedback Subscription (/encoder_speeds)
        self.create_subscription(
            Float64MultiArray,
            '/encoder_speeds',
            self.encoder_callback,
            10
        )

        # 3. Telemetry Publisher (/odom)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # 4. TF Broadcaster (odom -> base_link)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Wheel Odometry Node active.")

    def encoder_callback(self, msg: Float64MultiArray):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0:
            return

        # Measured wheel speeds from encoders
        wheel_speeds = msg.data

        # --- [Forward Kinematics] ---
        # Calculate chassis net velocities (vx, vy, wz)
        vx, vy, wz = self.kinematics.forward(wheel_speeds)

        # --- [Pose Integration] ---
        # Calculate change in position and angle over dt
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_theta = wz * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Compute orientation Quaternion
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # --- [Telemetry Publication (/odom)] ---
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Velocity
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = wz

        self.odom_pub.publish(odom_msg)

        # --- [TF Broadcast (odom -> base_link)] ---
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()