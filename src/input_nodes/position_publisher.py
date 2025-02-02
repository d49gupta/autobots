import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from RosBagInterpreter import sensorData

class PositionPublisher(Node):
    def __init__(self, sensorData, nodeName, topicName):
        super().__init__(nodeName)
        self.publisher = self.create_publisher(PointStamped, topicName, 10)
        self.timer = self.create_timer(0.05, self.publish_position)
        self.sensorData = sensorData
        self.lastSequence = 0

    def publish_position(self):
        position_msg = PointStamped()
        data = self.sensorData.getSensorData(self.sensorData.position_topic)

        if data != False and data.header.seq != self.lastSequence:
            position_msg.header.stamp.sec = data.header.stamp.sec
            position_msg.header.stamp.nanosec = data.header.stamp.nanosec
            position_msg.header.frame_id = data.header.frame_id

            point_msg = Point()
            point_msg.x = data.point.x
            point_msg.y = data.point.y
            point_msg.z = data.point.z
            position_msg.point = point_msg

            self.publisher.publish(position_msg)
            self.get_logger().info(f"Published position data with header sequence: {data.header.seq}")
            # self.get_logger().info(f"Published position data with header sequence: {data.header.stamp}")
            self.lastSequence = data.header.seq

def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()
    sensor_publisher = sensorData()
    sensor_publisher.start_bag_reader()
    
    position_publisher = PositionPublisher(sensor_publisher, "position_publisher", "position_topic")
    executor.add_node(position_publisher)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.stop_bag_reader()
        position_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()