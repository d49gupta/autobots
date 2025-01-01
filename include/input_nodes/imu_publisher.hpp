#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

class ImuPublisher : public rclcpp::Node
{
public:
  ImuPublisher(std::string nodeName, std::string topicName);
  void getIMUData();

private:
  rclcpp::TimerBase::SharedPtr timer_data;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  int count_;
  std_msgs::msg::Float32MultiArray imu_data = std_msgs::msg::Float32MultiArray();
};