#include "subscriber.hpp"

counterSubscriber::counterSubscriber(std::string nodeName, int size, std::string topicName) : Node(nodeName), counterCache(size) {
  auto topic_callback =
  [this, topic = topicName](std_msgs::msg::Int32::UniquePtr msg) -> void {
    RCLCPP_INFO(this->get_logger(), "I heard: '%d' on topic: %s", msg->data, topic.c_str());
    this->counterCache.enqueue(msg -> data);
  };
  subscription_ = this->create_subscription<std_msgs::msg::Int32>(topicName, 10, topic_callback);
}

ImuSubscriber::ImuSubscriber(std::string nodeName, int size, std::string topicName) : Node(nodeName), imuCache(size) { //same data is being pushed to cache, need a way to prevent that
  auto topic_callback =
  [this, topic = topicName](sensor_msgs::msg::Imu::UniquePtr msg) -> void {
    IMUdata imu_data;
    imu_data.linear_acceleration = msg->linear_acceleration;
    imu_data.angular_velocity = msg->angular_velocity;
    imu_data.orientation = msg->orientation;
    this->imuCache.enqueue(imu_data);

    RCLCPP_INFO(this->get_logger(), "IMU data received on topic %s", topic.c_str());
  };
  subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(topicName, 10, topic_callback);
}

ImageSubscriber::ImageSubscriber(std::string nodeName, int size, std::string topicName) : Node(nodeName), imageCache(size) {
  auto topic_callback =
  [this, topic = topicName](sensor_msgs::msg::Image::UniquePtr msg) -> void {
    this->cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::MONO8);  
    image_callback();
    this->imageCache.enqueue(*msg); //dereference msg pointer to get msg
    RCLCPP_INFO(this->get_logger(), "Image data received on topic %s", topic.c_str());
  };
  subscription_ = this->create_subscription<sensor_msgs::msg::Image>(topicName, 10, topic_callback);
}

PositionSubscriber::PositionSubscriber(std::string nodeName, int size, std::string topicName) : Node(nodeName), positionCache(size) {
  auto topic_callback =
  [this, topic = topicName](geometry_msgs::msg::PointStamped::UniquePtr msg) -> void {
    this->positionCache.enqueue(msg->point);
    RCLCPP_INFO(this->get_logger(), "Position data received on topic %s", topic.c_str());
  };
  subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(topicName, 10, topic_callback);
}

void ImageSubscriber::image_callback() {
    try
    {      
        cv::imshow("Mono8 Image", this->cv_ptr->image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("image_listener"), "Could not convert image");
    }
}