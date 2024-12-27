#include "counter_publisher.hpp"
#include "imu_publisher.hpp"
#include <memory>
#include <string>
#include <thread>

void run_counter_publisher()
{
  auto counter_publisher = std::make_shared<counterPublisher>("counter_publisher", "counter_topic");
  rclcpp::spin(counter_publisher);
  rclcpp::shutdown();
}

// void run_imu_publisher()
// {
//   auto imu_publisher = std::make_shared<ImuPublisher>("imu_publisher", "imu_topic");
//   rclcpp::spin(imu_publisher);
//   rclcpp::shutdown();
// }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto counter_publisher1 = std::make_shared<counterPublisher>("counter_publisher1", "counter_topic1");
  auto counter_publisher2 = std::make_shared<counterPublisher>("counter_publisher2", "counter_topic2");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(counter_publisher1);
  executor.add_node(counter_publisher2);

  executor.spin();
  rclcpp::shutdown();

  return 0;
}