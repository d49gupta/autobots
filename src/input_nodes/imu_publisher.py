import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from RosBagInterpreter import sensorData

class ImuPublisher(Node):
    def __init__(self, sensorData):
        super().__init__('imu_publisher')
        self.publisher = self.create_publisher(Imu, 'imu_topic', 10)
        self.timer = self.create_timer(0.01, self.publish_imu)
        self.topicName = '/imu0'
        self.sensorData = sensorData

    def publish_imu(self):
        imu_msg = Imu()
        data = self.sensorData.getSensorData(self.sensorData.imu_topic)
        if data != False:
            imu_msg.header.stamp.sec = data.header.stamp.sec
            imu_msg.header.stamp.nanosec = data.header.stamp.nanosec
            imu_msg.header.frame_id = data.header.frame_id

            orientation = Quaternion()
            orientation.x = data.orientation.x
            orientation.y = data.orientation.y
            orientation.z = data.orientation.z
            orientation.w = data.orientation.w
            imu_msg.orientation = orientation
            imu_msg.orientation_covariance = data.orientation_covariance

            angular_velocity = Vector3()
            angular_velocity.x = data.angular_velocity.x
            angular_velocity.y = data.angular_velocity.y
            angular_velocity.z = data.angular_velocity.z
            imu_msg.angular_velocity = angular_velocity
            imu_msg.angular_velocity_covariance = data.angular_velocity_covariance

            linear_acceleration = Vector3()
            linear_acceleration.x = data.linear_acceleration.x
            linear_acceleration.y = data.linear_acceleration.y
            linear_acceleration.z = data.linear_acceleration.z
            imu_msg.linear_acceleration = linear_acceleration
            imu_msg.linear_acceleration_covariance = data.linear_acceleration_covariance

            self.publisher.publish(imu_msg)
            self.get_logger().info(f"Published IMU data with header sequence: {data.header.seq}")

def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()
    sensor_publisher = sensorData()
    sensor_publisher.start_bag_reader()
    
    imu_publisher = ImuPublisher(sensor_publisher)
    executor.add_node(imu_publisher)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.stop_bag_reader()
        imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
