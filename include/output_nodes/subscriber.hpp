#include <memory>
#include <iostream>
#include "dataCache.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

class counterSubscriber : public rclcpp::Node
{
  public:
    counterSubscriber(std::string nodeName, int size, std::string topicName);
    dataCache<int> counterCache;

  private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

class ImuSubscriber : public rclcpp::Node
{
  public:
    ImuSubscriber(std::string nodeName, int size, std::string topicName);

  private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    dataCache<int> imuCache;
};

class sensorFusion : public rclcpp::Node {
public:
    sensorFusion(const std::shared_ptr<counterSubscriber> counterSensor1, 
                 const std::shared_ptr<counterSubscriber> counterSensor2);

    void fuseData();
private:
    const std::shared_ptr<counterSubscriber> counterSensor1; 
    const std::shared_ptr<counterSubscriber> counterSensor2;
    rclcpp::TimerBase::SharedPtr timer_;
 
};