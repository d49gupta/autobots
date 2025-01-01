#include "subscriber.hpp"
#include "alphaBetaFilter.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto first_subscriber = std::make_shared<counterSubscriber>("counter_subscriber", 25, "counter_topic");
  auto second_subscriber = std::make_shared<ImuSubscriber>("imu_publisher", 25, "imu_topic");

  auto sensor_fusion = std::make_shared<AlphaBetaFilter>(first_subscriber, second_subscriber, 0.1, 0.2, 0.005);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(first_subscriber);
  executor.add_node(second_subscriber);
  executor.add_node(sensor_fusion);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}