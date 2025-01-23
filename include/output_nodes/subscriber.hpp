#ifndef SUBSCRIBER_HPP
#define SUBSCRIBER_HPP

#include <memory>
#include <iostream>
#include <mutex>
#include "dataCache.hpp"
#include "hashMap.hpp"

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
#include "builtin_interfaces/msg/time.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/header.hpp"

struct IMUdata {
  geometry_msgs::msg::Vector3 linear_acceleration;
  geometry_msgs::msg::Vector3 angular_velocity;
  geometry_msgs::msg::Quaternion orientation;
  std_msgs::msg::Header header;
};

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
    ImuSubscriber(std::string nodeName, int size, std::string topicName, int resolution);
    HashMap<builtin_interfaces::msg::Time, IMUdata> imuHashMap;
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
    cv_bridge::CvImagePtr cv_ptr;
    std::mutex image_mutex;
    std::string topicName;
};

class PositionSubscriber : public rclcpp::Node
{
  public:
    PositionSubscriber(std::string nodeName, int size, std::string subscriber_topicName, std::string publisher_topicName, HashMap<builtin_interfaces::msg::Time, IMUdata>& imuHashMap);
    dataCache<geometry_msgs::msg::Point> positionCache;
    void pathCallback(const geometry_msgs::msg::PointStamped &msg);
    std::string publisher_topicName;
    float totalMessagesSubscribed = 0;
    float totalMessagesPublished = 0;
    nav_msgs::msg::Path path;
    std::string frame_id = "map";

  private:
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    HashMap<builtin_interfaces::msg::Time, IMUdata>& imuHashMap;
};

#endif
