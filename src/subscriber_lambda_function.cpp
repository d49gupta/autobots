#include "subscriber_lamda_function.hpp"

MinimalSubscriber::MinimalSubscriber() : Node("minimal_subscriber") {
  auto topic_callback =
  [this](std_msgs::msg::String::UniquePtr msg) -> void {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  };
  subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, topic_callback);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
