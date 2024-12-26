#include <memory>
#include "dataCache.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber();

private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
  dataCache MinimalSubscriberBuffer = dataCache(25);
  
};

class ImuSubscriber : public rclcpp::Node
{
public:
    ImuSubscriber();

private:
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
  
};