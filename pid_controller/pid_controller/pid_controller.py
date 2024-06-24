import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from vesc_msgs.msg import VescSetCommand
from simple_pid import PID
from control_interfaces.msg import Control

class VESCPIDController(Node):
    def __init__(self):
        super().__init__("vesc_pid_controller")
        self.declare_parameters(namespace="", parameters=[
            ("kp", 1.0), ("ki", 0.0), ("kd", 0.0),
        ])
        self.pid = PID(self.get_parameter("kp").value, self.get_parameter("ki").value, self.get_parameter("kd").value)
        self.last_position_x = None
        self.last_timestamp = None
        self.target_speed = 0.0  # This will be set based on the control command
        self.steering_angle = 0.0  # This too
        
        self.vesc_command_publisher = self.create_publisher(Control, "commands/ctrl", 10)
        self.pose_subscription = self.create_subscription(PoseStamped, "optitrack/rigid_body_0", self.pose_callback, 10)
        self.control_command_subscription = self.create_subscription(Control, "mpc_control", self.control_command_callback, 1)

    def pose_callback(self, msg):
        current_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if self.last_position_x is not None and self.last_timestamp is not None:
            time_delta = current_timestamp - self.last_timestamp
            current_speed = (msg.pose.position.x - self.last_position_x) / time_delta
            self.pid.setpoint = self.target_speed
            speed_error = self.target_speed - current_speed
            control_output = self.pid(speed_error)
            self.send_motor_command(control_output)
            self.get_logger().info(f"Speed estimated: {current_speed}, Target Speed: {self.target_speed}, Control Output: {control_output}")
        
        self.last_position_x = msg.pose.position.x
        self.last_timestamp = current_timestamp

    def control_command_callback(self, msg):
        self.target_speed = msg.set_speed
        self.steering_angle = msg.steering_angle
        self.get_logger().info(f"New target speed received: {msg.set_speed}")

    def send_motor_command(self, current):
        control = Control(
                set_current=current,
                steering_angle=self.steering_angle,
                control_mode=Control.CURRENT_MODE,
            )
        self.vesc_command_publisher.publish(control)
        self.get_logger().info(f"Sent motor current command: {current}")

def main(args=None):
    rclpy.init(args=args)
    vesc_controller = VESCPIDController()
    rclpy.spin(vesc_controller)
    vesc_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
