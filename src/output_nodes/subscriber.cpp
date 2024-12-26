#include "subscriber.hpp"

MinimalSubscriber::MinimalSubscriber() : Node("minimal_subscriber") {
  auto topic_callback =
  [this](std_msgs::msg::Int32::UniquePtr msg) -> void {
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
    this->MinimalSubscriberBuffer.enqueue(msg -> data);
    RCLCPP_INFO(this->get_logger(), "data pushed to buffer");
  };
  subscription_ = this->create_subscription<std_msgs::msg::Int32>("topic", 10, topic_callback);
}

ImuSubscriber::ImuSubscriber() : Node("imu_subscriber") {
  auto topic_callback =
  [this](std_msgs::msg::Float32MultiArray::UniquePtr msg) -> void {
    std::vector<float> data = msg->data;
    RCLCPP_INFO(this->get_logger(), "Accelerometer Readings: X: %f, Y: %f, Z: %f", data[0], data[1], data[2]);
    RCLCPP_INFO(this->get_logger(), "Gyroscope Readings: X: %f, Y: %f, Z: %f", data[3], data[4], data[5]);
  };
  subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("imu", 10, topic_callback);
}

