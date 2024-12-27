#include "subscriber.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto first_subscriber = std::make_shared<counterSubscriber>("counter_subscriber1", 25, "counter_topic1");
  auto second_subscriber = std::make_shared<counterSubscriber>("counter_subscriber2", 25, "counter_topic2");
  // auto second_subscriber = std::make_shared<ImuSubscriber>("imu_subscriber", 25, "imu_topic");

  auto sensor_fusion = std::make_shared<sensorFusion>(first_subscriber, second_subscriber);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(first_subscriber);
  executor.add_node(second_subscriber);
  executor.add_node(sensor_fusion);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}