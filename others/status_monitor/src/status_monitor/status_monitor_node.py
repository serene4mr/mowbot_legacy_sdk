import rclpy
from rclpy.node import Node

import diagnostic_updater
import diagnostic_msgs.msg

class StatusMonitorNode(Node):
    def __init__(self):
        super().__init__('status_monitor_node')
        self.get_logger().info("Status Monitor Node has been started.")
        
        # Create a DiagnosticUpdater
        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID("none")
        
    

    
    
def main(args=None):
    rclpy.init(args=args)
    node = StatusMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    