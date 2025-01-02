#include "subscriber.hpp"
#include "alphaBetaFilter.hpp"
#include <csignal>
#include <unistd.h>

rclcpp::executors::MultiThreadedExecutor* global_executor = nullptr;

void signalHandler(int signal) {
  std::cout<<"Stop signal detected"<<std::endl;
  if (global_executor)
    global_executor->cancel();
}

int main(int argc, char * argv[])
{
  signal(SIGINT, signalHandler);

  rclcpp::init(argc, argv);
  auto first_subscriber = std::make_shared<counterSubscriber>("counter_subscriber", 25, "counter_topic");
  auto second_subscriber = std::make_shared<ImuSubscriber>("imu_subscriber", 25, "imu_topic");
  auto third_subscriber = std::make_shared<ImageSubscriber>("image_subsciber", 25, "image_topic");

  // auto sensor_fusion = std::make_shared<AlphaBetaFilter>(first_subscriber, second_subscriber, 0.1, 0.2, 0.005);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(first_subscriber);
  executor.add_node(second_subscriber);
  executor.add_node(third_subscriber);
  // executor.add_node(sensor_fusion);

  global_executor = &executor;
  executor.spin();
  rclcpp::shutdown();
  return 0;
}