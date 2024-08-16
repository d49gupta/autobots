#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber();

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  
};

class ImuSubscriber : public rclcpp::Node
{
public:
    ImuSubscriber();

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  
};