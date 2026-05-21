import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MyRobotController(Node):
    def __init__(self):
        super().__init__('my_robot_controller')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/wheel_vel_cmd')

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )
        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.get_logger().info(f'Controller subscribed to: {cmd_vel_topic}')
        self.get_logger().info(f'Controller publishing to: {output_topic}')

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.get_logger().debug(
            f'Received cmd_vel: linear={msg.linear.x:.3f}, angular={msg.angular.z:.3f}'
        )
        # Forward the Twist command to the output topic or insert control logic here.
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MyRobotController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
