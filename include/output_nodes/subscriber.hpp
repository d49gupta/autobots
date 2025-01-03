#ifndef SUBSCRIBER_HPP
#define SUBSCRIBER_HPP

#include <memory>
#include <iostream>
#include "dataCache.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"

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

class ImageSubscriber : public rclcpp::Node 
{
  public:
    ImageSubscriber(std::string nodeName, int size, std::string topicName);
    dataCache<sensor_msgs::msg::Image> imageCache;
    void image_callback();

  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    sensor_msgs::msg::Image::UniquePtr last_image;
    cv_bridge::CvImagePtr cv_ptr;
};

class PositionSubscriber : public rclcpp::Node
{
  public:
    PositionSubscriber(std::string nodeName, int size, std::string topicName);
    dataCache<geometry_msgs::msg::Point> positionCache;

  private:
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
};

#endif
