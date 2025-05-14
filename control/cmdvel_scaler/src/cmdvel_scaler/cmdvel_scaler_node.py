import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .cmdvel_scaler import CmdVelScaler

class CmdVelScalerNode(Node):
    """
    ROS2 node that modifies cmd_vel commands using motor scaling factors.
    Maintains Twist format while accounting for motor performance characteristics.
    """
    def __init__(self):
        super().__init__('cmdvel_scaler_node')
        
        # Declare parameters with defaults
        self.declare_parameters(namespace='',
            parameters=[
                ('left_rate', 1.0),
                ('right_rate', 1.0),
                ('wheel_separation', 0.5),
                ('verbose', False)
            ])
        
        # Get parameters atomically
        params = self.get_parameters(['left_rate', 'right_rate', 'wheel_separation', 'verbose'])
        self.left_rate = params[0].value
        self.right_rate = params[1].value
        self.wheel_separation = params[2].value
        self.verbose = params[3].value
        
        # Initialize scaler
        self.scaler = CmdVelScaler(
            left_rate=self.left_rate,
            right_rate=self.right_rate,
            wheel_separation=self.wheel_separation
        )

        # Setup subscribers/publishers
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel_scaled', 10)
        
        self.get_logger().info(f"Node initialized with parameters: "
                              f"left_rate={self.left_rate}, "
                              f"right_rate={self.right_rate}, "
                              f"wheel_separation={self.wheel_separation}m, "
                              f"verbose={self.verbose}")

    def cmd_vel_callback(self, msg):
        """Process incoming cmd_vel and publish scaled version"""
        scaled_x, scaled_z = self.scaler.scale_cmd_vel(msg.linear.x, msg.angular.z)
        
        scaled_msg = Twist()
        scaled_msg.linear.x = scaled_x
        scaled_msg.angular.z = scaled_z
        self.publisher.publish(scaled_msg)
        
        if self.verbose:
            self.get_logger().info(
                f"Original: v={msg.linear.x:.2f}m/s ω={msg.angular.z:.2f}rad/s | "
                f"Scaled: v={scaled_x:.2f}m/s ω={scaled_z:.2f}rad/s"
            )

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelScalerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
