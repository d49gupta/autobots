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
    image_publisher = ImagePublisher(sensor_publisher, "image_publisher", "image_topic")

    executor.add_node(position_publisher)
    executor.add_node(imu_publisher)
    executor.add_node(image_publisher)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.stop_bag_reader()
        position_publisher.destroy_node()
        imu_publisher.destroy_node()
        image_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()