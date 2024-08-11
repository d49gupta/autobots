#include "subscriber_lamda_function.hpp"
#include "publisher_lamda_function.hpp"
#include "servo_control.hpp"
#include "astar_search_path.hpp"
#include <thread>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto publisher_node = std::make_shared<MinimalPublisher>();
  auto subscriber_node = std::make_shared<MinimalSubscriber>();

  std::thread publisher_thread([publisher_node]() {
    rclcpp::spin(publisher_node);
  });

  std::thread subscriber_thread([subscriber_node]() {
    rclcpp::spin(subscriber_node);
  });

  publisher_thread.join();
  subscriber_thread.join();

  rclcpp::shutdown();
  return 0;
}