import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from geometry_msgs.msg import Pose
from RosBagInterpreter import sensorData
from nav_msgs.msg import Path

class PosePublisher(Node):
    def __init__(self, sensorData, nodeName, topicName):
        super().__init__(nodeName)
        self.publisher = self.create_publisher(Path, topicName, 10)
        self.timer = self.create_timer(0.05, self.publish_pose)
        self.sensorData = sensorData
        self.lastSequence = 0
        self.path = Path()
        self.frame_id = "map"

    def publish_pose(self):
        data = self.sensorData.getSensorData(self.sensorData.position_topic)
        if data != False and data.header.seq != self.lastSequence:
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.stamp.sec = data.header.stamp.sec
            pose_stamped_msg.header.stamp.nanosec = data.header.stamp.nanosec
            pose_stamped_msg.header.frame_id = self.frame_id

            pose_stamped_msg.pose.position.x = data.point.x
            pose_stamped_msg.pose.position.y = data.point.y
            pose_stamped_msg.pose.position.z = data.point.z
            pose_stamped_msg.pose.orientation.w = 1.0

            self.path.header = pose_stamped_msg.header
            self.path.poses.append(pose_stamped_msg)

            self.publisher.publish(self.path)
            self.get_logger().info(f"Published ground truth data with header sequence: {data.header.seq}")
            self.lastSequence = data.header.seq

def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()
    sensor_publisher = sensorData()
    sensor_publisher.start_bag_reader()
    
    pose_publisher = PosePublisher(sensor_publisher, "pose_publisher", "pose_topic_test")
    executor.add_node(pose_publisher)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.stop_bag_reader()
        pose_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
