#include "imu_publisher.hpp"

ImuPublisher::ImuPublisher(std::string nodeName, std::string topicName) : Node(nodeName), count_(0), imu_device(0x68) { //TODO: chagne topic name and node name to variable passed in constructor
  sleep(1);
  publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(topicName, 10);
  timer_data = this->create_wall_timer(500ms, std::bind(&ImuPublisher::getIMUData, this));
}

void ImuPublisher:: getIMUData() {
  float ax, ay, az, gr, gp, gy; 

  //Read the current yaw angle
  imu_device.calc_yaw = true;
  imu_device.getAccel(&ax, &ay, &az); //accelerometer values
  imu_device.getGyro(&gr, &gp, &gy); //gyroscope values
  imu_data.data = {ax, ay, az, gr, gp, gy};
  RCLCPP_INFO(this->get_logger(), "Publishing to the IMU topic '%d'", this->count_++);
  this->publisher_->publish(imu_data);
}