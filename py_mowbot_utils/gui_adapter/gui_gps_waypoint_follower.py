import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import yaml
from py_mowbot_utils.gps_utils import latLonYaw2Geopose
from geometry_msgs.msg import PoseStamped
from robot_localization.srv import FromLL


class YamlWaypointParser:
    """
    Parse a set of gps waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        try:
            with open(wps_file_path, 'r') as wps_file:
                self.wps_dict = yaml.safe_load(wps_file)
        except FileNotFoundError:
            raise FileNotFoundError(f"File {wps_file_path} not found.")

    def get_wps(self):
        """
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml file
        """
        gepose_wps = []
        for wp in self.wps_dict["waypoints"]:
            latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp["yaw"]
            gepose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
        return gepose_wps


class WaypointFollower(Node):
    
    def __init__(self):
        super().__init__('waypoint_follower')
        
        self.declare_parameter('waypoints_file', 'gps_waypoints.yaml')
        self.waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        
        self.get_logger().info(f'waypoints_file={self.waypoints_file}')
        
        self.navigator = BasicNavigator("basic_navigator")
        self.wp_parser = None
        self.localizer = self.create_client(FromLL, '/fromLL')
        
        # Wait for service
        while not self.localizer.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('FromLL service not available, waiting again...')

        # State check timer - check navigation status
        self.state_check_timer = self.create_timer(1.0, self.check_nav_state)
        self.state_check_timer.cancel()  # Start it only when navigation begins
        
        # Flag to track if navigation is active
        self.navigation_active = False
        
        self.start_wpf()

    def start_wpf(self):
        """Convert GPS waypoints to map frame and start navigation"""
        
        self.wp_parser = YamlWaypointParser(self.waypoints_file)
        wps = self.wp_parser.get_wps()
        
        self.get_logger().info(f'Loaded {len(wps)} GPS waypoints')

        # Wait for Nav2 to be active
        self.navigator.waitUntilNav2Active(localizer='controller_server')
        self.get_logger().info('Nav2 is active')
        
        # Convert GPS waypoints to map frame poses
        wpl = []
        for idx, wp in enumerate(wps):
            req = FromLL.Request()
            req.ll_point.longitude = wp.position.longitude
            req.ll_point.latitude = wp.position.latitude
            req.ll_point.altitude = wp.position.altitude

            self.get_logger().info(
                f'Waypoint {idx+1}/{len(wps)}: '
                f'lon={req.ll_point.longitude:.6f}, '
                f'lat={req.ll_point.latitude:.6f}, '
                f'alt={req.ll_point.altitude:.2f}'
            )

            # Call service to convert lat/lon to map coordinates
            future = self.localizer.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is None:
                self.get_logger().error(f'Failed to convert waypoint {idx+1}')
                return

            # Create PoseStamped with unique object for each waypoint
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position = future.result().map_point
            pose_stamped.pose.orientation = wp.orientation

            self.get_logger().info(
                f'Map coordinates: '
                f'x={pose_stamped.pose.position.x:.2f}, '
                f'y={pose_stamped.pose.position.y:.2f}, '
                f'z={pose_stamped.pose.position.z:.2f}'
            )
            
            wpl.append(pose_stamped)

        if len(wpl) == 0:
            self.get_logger().error('No valid waypoints to navigate')
            return

        self.get_logger().info(
            f'Starting continuous navigation through {len(wpl)} poses'
        )

        # Use goThroughPoses for continuous navigation without stopping
        self.navigator.goThroughPoses(wpl)
        self.navigation_active = True

        # Start monitoring navigation state
        if self.state_check_timer.is_canceled():
            self.state_check_timer.reset()

    def stop_wpf(self):
        """Cancel navigation and stop monitoring"""
        if self.navigation_active:
            self.navigator.cancelTask()
            self.navigation_active = False

        # Stop the state check timer    
        if not self.state_check_timer.is_canceled():
            self.state_check_timer.cancel()

    def check_nav_state(self):
        """Periodically check navigation status"""
        if not self.navigation_active:
            return

        # Get feedback while navigating
        feedback = self.navigator.getFeedback()
        if feedback:
            # NavigateThroughPoses feedback includes navigation_time
            nav_time = getattr(feedback, 'navigation_time', None)
            if nav_time:
                self.get_logger().info(
                    f'Navigation in progress - '
                    f'Time elapsed: {nav_time.sec}s',
                    throttle_duration_sec=5.0  # Log every 5 seconds
                )

        # Check if task is complete
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(
                    '✓ Successfully navigated through all poses!'
                )
            elif result == TaskResult.CANCELED:
                self.get_logger().warn(
                    '⚠ Navigation was canceled'
                )
            elif result == TaskResult.FAILED:
                self.get_logger().error(
                    '✗ Navigation failed'
                )
            
            self.stop_wpf()


def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    
    try:
        rclpy.spin(waypoint_follower)
    except KeyboardInterrupt:
        waypoint_follower.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        waypoint_follower.stop_wpf()
        waypoint_follower.destroy_node()
        rclpy.shutdown()

    
if __name__ == '__main__':
    main()
