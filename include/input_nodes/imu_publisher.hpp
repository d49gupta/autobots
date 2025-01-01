#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "csv_reader.hpp"

using namespace std::chrono_literals;

class ImuPublisher : public rclcpp::Node
{
public:
  ImuPublisher(std::string nodeName, std::string topicName);
  void getIMUData();

private:
  int count_;
  csvReader imuReader;
  sensor_msgs::msg::Imu imu_data = sensor_msgs::msg::Imu();
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_data;
};