#include "imu_publisher.hpp"

ImuPublisher::ImuPublisher(std::string nodeName, std::string topicName) : count_(0), imuReader("../rosbags/data/imu0.csv"), Node(nodeName) {
  this->imuReader.readCSV();
  publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(topicName, 10);
  timer_data = this->create_wall_timer(500ms, std::bind(&ImuPublisher::getIMUData, this));
}

void ImuPublisher::getIMUData() {
  map<string, string> current_imu_data = this->imuReader.data[this->count_];
  // imu_data.header.seq = static_cast<uint32_t>(std::stoul(current_imu_data["header.seq"]));
  int64_t seconds = std::stoll(current_imu_data["header.stamp.secs"]);
  int64_t nanoseconds = std::stoll(current_imu_data["header.stamp.nsecs"]);
  rclcpp::Time time(seconds, nanoseconds);
  imu_data.header.stamp = time;
  imu_data.header.frame_id = current_imu_data["header.frame_id"];

  imu_data.orientation.x = std::stod(current_imu_data["orientation.x"]);
  imu_data.orientation.y = std::stod(current_imu_data["orientation.y"]);
  imu_data.orientation.z = std::stod(current_imu_data["orientation.z"]);
  imu_data.orientation.w = std::stod(current_imu_data["orientation.w"]);
  imu_data.orientation_covariance = {99999.9, 0, 0, 0, 99999.9, 0, 0, 0, 99999.9};

  imu_data.angular_velocity.x = std::stod(current_imu_data["angular_velocity.x"]);
  imu_data.angular_velocity.y = std::stod(current_imu_data["angular_velocity.y"]);
  imu_data.angular_velocity.z = std::stod(current_imu_data["angular_velocity.z"]);
  imu_data.angular_velocity_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

  imu_data.linear_acceleration.x = std::stod(current_imu_data["linear_acceleration.x"]);
  imu_data.linear_acceleration.y = std::stod(current_imu_data["linear_acceleration.y"]);
  imu_data.linear_acceleration.z = std::stod(current_imu_data["linear_acceleration.z"]);
  imu_data.angular_velocity_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};  

  RCLCPP_INFO(this->get_logger(), "Publishing to the IMU topic '%d'", this->count_++);
  this->publisher_->publish(imu_data);

}