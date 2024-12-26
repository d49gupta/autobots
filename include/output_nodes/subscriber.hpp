#include <memory>
#include "dataCache.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

class counterSubscriber : public rclcpp::Node
{
public:
  counterSubscriber(std::string nodeName, int size, std::string topicName);
  
private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
  dataCache<int> counterCache;
};

class ImuSubscriber : public rclcpp::Node
{
public:
  ImuSubscriber(std::string nodeName, int size, std::string topicName);

private:
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
  dataCache<int> imuCache;
};