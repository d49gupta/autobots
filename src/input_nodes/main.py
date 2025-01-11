import rclpy
from rclpy.node import Node
import signal

from position_publisher import PositionPublisher
from imu_publisher import ImuPublisher
from image_publisher import ImagePublisher
from RosBagInterpreter import sensorData

sensor_publisher = None
position_publisher = None
imu_publisher = None
image_publisher = None

def handle_sigint(signum, frame):
    print("Shutdown signal received")
    if sensor_publisher:
        sensor_publisher.stop_bag_reader()
    if position_publisher:
        position_publisher.destroy_node()
    if imu_publisher:
        imu_publisher.destroy_node()
    if image_publisher:
        imu_publisher.destroy_node()
        
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()
    sensor_publisher = sensorData()
    sensor_publisher.start_bag_reader()
    
    position_publisher = PositionPublisher(sensor_publisher, "position_publisher", "position_topic")
    imu_publisher = ImuPublisher(sensor_publisher, "imu_publisher", "imu_topic")
    image_publisher0 = ImagePublisher(sensor_publisher, "image_publisher0", "image_topic0", 0)
    image_publisher1 = ImagePublisher(sensor_publisher, "image_publisher1", "image_topic1", 1)

    executor.add_node(position_publisher)
    executor.add_node(imu_publisher)
    executor.add_node(image_publisher0)
    executor.add_node(image_publisher1)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.stop_bag_reader()
        position_publisher.destroy_node()
        imu_publisher.destroy_node()
        image_publisher0.destroy_node()
        image_publisher1.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()