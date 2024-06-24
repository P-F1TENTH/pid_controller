import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from simple_pid import PID
from vesc_msgs.msg import VescSetCommand, VescStateStamped


class VESCPIDController(Node):
    """Node for controlling the VESC using PID based on the pose from Optitrack."""

    def __init__(self):
        super().__init__("vesc_pid_controller")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("kp", 1.0),
                ("ki", 0.0),
                ("kd", 0.0),
                ("target_x", 0.0),
                ("target_y", 0.0),
            ],
        )
        self.pid_x = PID(
            self.get_parameter("kp").value,
            self.get_parameter("ki").value,
            self.get_parameter("kd").value,
            setpoint=self.get_parameter("target_x").value,
        )
        self.pid_y = PID(
            self.get_parameter("kp").value,
            self.get_parameter("ki").value,
            self.get_parameter("kd").value,
            setpoint=self.get_parameter("target_y").value,
        )

        self.vesc_command_publisher = self.create_publisher(
            VescSetCommand, "commands/motor/speed", 10
        )
        self.pose_subscription = self.create_subscription(
            PoseStamped, "optitrack/rigid_body_0", self.pose_callback, 10
        )
        self.vesc_state_subscription = self.create_subscription(
            VescStateStamped, "vesc/core", self.vesc_state_callback, 10
        )

    def pose_callback(self, msg):
        """Handle new pose data."""
        current_x = msg.pose.position.x
        current_y = msg.pose.position.y
        speed_command = self.pid_x(
            current_x
        )  # Assume control on x-axis only for simplicity
        self.send_motor_command(speed_command)
        self.get_logger().info(
            f"Pose updated. Current X: {current_x}, Command Speed: {speed_command}"
        )

    def vesc_state_callback(self, msg):
        """Handle new VESC state data."""
        self.get_logger().info(f"VESC state updated. Current speed: {msg.state.speed}")

    def send_motor_command(self, speed):
        """Send motor speed command to the VESC."""
        command = VescSetCommand(command=speed)
        self.vesc_command_publisher.publish(command)
        self.get_logger().info(f"Sent motor command: {speed}")


def main(args=None):
    rclpy.init(args=args)
    vesc_controller = VESCPIDController()
    rclpy.spin(vesc_controller)
    vesc_controller.destroy_node()
    rclpy.shutdown()


if _name_ == "_main_":
    main()
