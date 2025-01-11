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
  // auto counter_subscriber = std::make_shared<counterSubscriber>("counter_subscriber", 25, "counter_topic");
  // auto position_subscriber = std::make_shared<PositionSubscriber>("position_subscriber", 25, "position_topic");
  // auto sensor_fusion_subscriber = std::make_shared<AlphaBetaFilter>(first_subscriber, second_subscriber, 0.1, 0.2, 0.005);'
  
  auto imu_subscriber = std::make_shared<ImuSubscriber>("imu_subscriber", 25, "imu_topic");
  auto camera0_subscriber = std::make_shared<ImageSubscriber>("image_subsciber1", 25, "image_topic0");
  auto camera1_subscriber = std::make_shared<ImageSubscriber>("image_subsciber0", 25, "image_topic1");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(imu_subscriber);
  executor.add_node(camera0_subscriber);
  executor.add_node(camera1_subscriber);

  global_executor = &executor;
  executor.spin();
  rclcpp::shutdown();
  return 0;
}