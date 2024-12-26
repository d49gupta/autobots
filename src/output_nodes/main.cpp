#include "subscriber.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto first_subscriber = std::make_shared<counterSubscriber>("counter_subscriber", 25, "counter_topic");
  // auto second_subscriber = std::make_shared<ImuSubscriber>("imu_subscriber", 25, "imu_topic");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(first_subscriber);
  // executor.add_node(second_subscriber);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}