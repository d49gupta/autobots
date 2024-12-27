#include "counter_publisher.hpp"

counterPublisher::counterPublisher(std::string nodeName, std::string topicName) : Node(nodeName), count_(0) {
  publisher_ = this->create_publisher<std_msgs::msg::Int32>(topicName, 10);
  auto timer_callback =
    [this, topic = topicName]() -> void {
      auto message = std_msgs::msg::Int32();
      message.data = this->count_++;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%d' on topic %s", message.data, topic.c_str());
      this->publisher_->publish(message);
    };
  timer_ = this->create_wall_timer(500ms, timer_callback);
}