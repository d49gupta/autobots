#include "counter_publisher.hpp"

MinimalPublisher::MinimalPublisher() : Node("minimal_publisher"), count_(0) {
  publisher_ = this->create_publisher<std_msgs::msg::Int32>("topic", 10);
  auto timer_callback =
    [this]() -> void {
      auto message = std_msgs::msg::Int32();
      message.data = this->count_++;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
      this->publisher_->publish(message);
    };
  timer_ = this->create_wall_timer(500ms, timer_callback);
}