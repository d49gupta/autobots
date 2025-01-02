import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from RosBagInterpreter import sensorData
import numpy as np

class ImagePublisher(Node):
    def __init__(self, sensorData):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, 'image_topic', 10)
        self.timer = self.create_timer(1, self.publish_image)
        self.sensorData = sensorData

    def publish_image(self):
        image_msg = Image()
        data = self.sensorData.getSensorData(self.sensorData.camera0_topic)

        if data != False:
            image_msg.header.stamp.sec = data.header.stamp.sec
            image_msg.header.stamp.nanosec = data.header.stamp.nanosec
            image_msg.header.frame_id = data.header.frame_id
            image_msg.height = data.height
            image_msg.width = data.width
            image_msg.encoding = data.encoding
            image_msg.is_bigendian = data.is_bigendian
            image_msg.step = data.step
            image_msg.data = data.data.flatten().tolist()

            self.publisher.publish(image_msg)
            self.get_logger().info(f"Published image0 data with header sequence: {data.header.seq}")

def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()
    sensor_publisher = sensorData()
    sensor_publisher.start_bag_reader()
    
    image_publisher = ImagePublisher(sensor_publisher)
    executor.add_node(image_publisher)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.stop_bag_reader()
        image_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
