import rclpy
from rosbags.rosbag1 import Reader
from rosbags.typesys import get_typestore, Stores
from rclpy.node import Node
import threading
import signal

sensor_publisher = None

def handle_sigint(signum, frame):
    print("Shutdown signal received")
    if sensor_publisher:
        sensor_publisher.destroy_node()
    rclpy.shutdown()

class sensorData(Node):
    def __init__(self):
        super().__init__('sensor_data_node')
        self.bag_path = '../../rosbags/MH_01_easy.bag'
        self.typestore = get_typestore(Stores.ROS1_NOETIC)
        self.sensorDict = {}

        self.imu_topic = '/imu0'
        self.camera0_topic = '/cam0/image_raw'
        self.camera1_topic = '/cam1/image_raw'
        self.position_topic = '/leica/position'

        self.currentIndex = {self.imu_topic: -1, self.camera0_topic: -1, self.camera1_topic: -1, self.position_topic: -1}
        self.timer = self.create_timer(0.1, self.getData)
        self.lock = threading.Lock()
        self.reader = Reader(self.bag_path)
        self.reader.open()

        self.stop_reading = False
        self.reader_thread = None

    def getData(self):
        counter = 0
        for connection, timestamp, rawdata in self.reader.messages():
            # if counter >= 100:
            #     self.get_logger().info("Processed 100 iterations, stopping early for testing.")
            #     break
            # counter += 1
            try:
                with self.lock:
                    msg = self.typestore.deserialize_ros1(rawdata, connection.msgtype)
                    if connection.topic not in self.sensorDict:
                        self.sensorDict[connection.topic] = []
                    self.sensorDict[connection.topic].append({'timestamp': timestamp, 'message': msg})
                    self.currentIndex[connection.topic] += 1
                    # self.get_logger().info(f"Processed data for topic: {connection.topic}")
            except Exception as e:
                self.get_logger().error(f"Error deserializing message: {e}")
    
    def getSensorData(self, topic):
        with self.lock:
            if topic in self.sensorDict:
                index = self.currentIndex[topic]
                return self.sensorDict[topic][index]['message']
            else:
                return False
            
    def start_bag_reader(self):
        self.reader_thread = threading.Thread(target=self.getData)
        self.reader_thread.start()

    def stop_bag_reader(self):
        self.stop_reading = True
        if self.reader_thread:
            self.reader_thread.join()
        
def main(args=None):
    rclpy.init(args=args)
    # signal.signal(signal.SIGINT, handle_sigint)
    sensor_publisher = sensorData()
    rclpy.spin(sensor_publisher)
    sensor_publisher.destroy_node()
    rclpy.shutdown()
    print(sensor_publisher.currentIndex)
if __name__ == '__main__':
    main()