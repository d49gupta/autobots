#include <memory>
#include <iostream>
#include "dataCache.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

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
    struct IMUdata {
      geometry_msgs::msg::Vector3 linear_acceleration;
      geometry_msgs::msg::Vector3 angular_velocity;
      geometry_msgs::msg::Quaternion orientation;
    };
    
    dataCache<IMUdata> imuCache;
  private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

class sensorFusion : public rclcpp::Node {
public:
    sensorFusion(const std::shared_ptr<counterSubscriber> counterSensor1, 
                 const std::shared_ptr<ImuSubscriber> counterSensor2);

    void fuseData();
private:
    const std::shared_ptr<counterSubscriber> counterSensor1; 
    const std::shared_ptr<ImuSubscriber> counterSensor2;
    rclcpp::TimerBase::SharedPtr timer_;
 
};