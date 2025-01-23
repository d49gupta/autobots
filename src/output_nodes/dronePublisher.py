import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

class DroneVisualizer(Node):
    def __init__(self):
        super().__init__('drone_visualizer')
        self.path_sub = self.create_subscription(
            Path,
            'test_topic',  # Change to your path topic name
            self.path_callback,
            10
        )
        # Publisher for the drone marker
        self.marker_pub = self.create_publisher(Marker, 'drone_marker', 10)
        self.get_logger().info('Drone Visualizer Node has started.')

    def path_callback(self, msg):
        # If the path is empty, do nothing
        if not msg.poses:
            self.get_logger().warn('Received empty path.')
            return

        # Get the last pose from the path
        latest_pose = msg.poses[-1].pose

        # Create a marker to represent the drone
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'drone'
        marker.id = 0
        marker.type = Marker.ARROW  # You can change to SPHERE or ARROW
        marker.action = Marker.ADD

        # Set position and orientation
        marker.pose = latest_pose

        # Set the scale of the marker (size of the drone)
        marker.scale.x = 1.0  # Length
        marker.scale.y = 0.5  # Width
        marker.scale.z = 0.2  # Height

        # Set the color of the marker
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 1.0  # Fully visible

        # Publish the marker
        self.marker_pub.publish(marker)
        self.get_logger().info(f'Published drone marker at orientation: ({latest_pose.orientation.x}, '
                               f'{latest_pose.orientation.y}, {latest_pose.orientation.z}, {latest_pose.orientation.w})')

def main(args=None):
    rclpy.init(args=args)
    node = DroneVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
