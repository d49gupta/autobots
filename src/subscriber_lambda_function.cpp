#include "subscriber_lamda_function.hpp"

MinimalSubscriber::MinimalSubscriber() : Node("minimal_subscriber") {
  auto topic_callback =
  [this](std_msgs::msg::String::UniquePtr msg) -> void {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  };
  subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, topic_callback);
}

ImuSubscriber::ImuSubscriber() : Node("imu_subscriber") {
  auto topic_callback =
  [this](std_msgs::msg::String::UniquePtr msg) -> void {
    RCLCPP_INFO(this->get_logger(), "I heard the second topic: '%s'", msg->data.c_str());
  };
  subscription_ = this->create_subscription<std_msgs::msg::String>("imu", 10, topic_callback);
}

