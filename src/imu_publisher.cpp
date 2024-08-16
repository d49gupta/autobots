#include "imu_publisher.hpp"

ImuPublisher:: ImuPublisher() : Node("imu_publisher"), count_(0) {
  publisher_ = this->create_publisher<std_msgs::msg::String>("imu", 10);
  auto timer_callback =
  [this]() -> void {
    auto message = std_msgs::msg::String();
    message.data = "To the second topic with " + std::to_string(this->count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing '%s'", message.data.c_str());
    this->publisher_->publish(message);
  };
  timer_ = this->create_wall_timer(500ms, timer_callback);
}