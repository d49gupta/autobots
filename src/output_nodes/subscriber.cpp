#include "subscriber.hpp"

counterSubscriber::counterSubscriber(std::string nodeName, int size, std::string topicName) : Node(nodeName), counterCache(size) {
  auto topic_callback =
  [this, topic = topicName](std_msgs::msg::Int32::UniquePtr msg) -> void {
    RCLCPP_INFO(this->get_logger(), "I heard: '%d' on topic: %s", msg->data, topic.c_str());
    this->counterCache.enqueue(msg -> data);
  };
  subscription_ = this->create_subscription<std_msgs::msg::Int32>(topicName, 10, topic_callback);
}

ImuSubscriber::ImuSubscriber(std::string nodeName, int size, std::string topicName, int resolution) : Node(nodeName), imuHashMap(size, resolution){ //same data is being pushed to cache, need a way to prevent that
  auto topic_callback =
  [this, topic = topicName](sensor_msgs::msg::Imu::UniquePtr msg) -> void {
    IMUdata imu_data;
    imu_data.linear_acceleration = msg->linear_acceleration;
    imu_data.angular_velocity = msg->angular_velocity;
    imu_data.orientation = msg->orientation;
    imu_data.header = msg->header;
    imuHashMap.add(msg->header.stamp, imu_data);

    // RCLCPP_INFO(this->get_logger(), "IMU data received on topic %s", topic.c_str());
  };
  subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(topicName, 10, topic_callback);
}

ImageSubscriber::ImageSubscriber(std::string nodeName, int size, std::string topicName) : Node(nodeName), imageCache(size), topicName(topicName) {
  auto topic_callback =
  [this, topic = topicName](sensor_msgs::msg::Image::UniquePtr msg) -> void {
    if (msg){
      std::lock_guard<std::mutex> lock(this->image_mutex);
      this->cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::MONO8);
    }
    image_callback();
    // this->imageCache.enqueue(*msg); //dereference msg pointer to get msg
    RCLCPP_INFO(this->get_logger(), "Image width: %d, height: %d, encoding: %s", 
            msg->width, msg->height, msg->encoding.c_str());
  };
  subscription_ = this->create_subscription<sensor_msgs::msg::Image>(topicName, 10, topic_callback);
}

PositionSubscriber::PositionSubscriber(std::string nodeName, int size, std::string subscriber_topicName, std::string publisher_topicName, HashMap<builtin_interfaces::msg::Time, IMUdata>& imuHashMap) : 
                                      Node(nodeName), positionCache(size), publisher_topicName(publisher_topicName), imuHashMap(imuHashMap) {
  publisher_ = this->create_publisher<nav_msgs::msg::Path>(publisher_topicName, 10);
  
  auto topic_callback =
  [this, subscriber_topicName = subscriber_topicName](geometry_msgs::msg::PointStamped::UniquePtr msg) -> void {
    RCLCPP_INFO(this->get_logger(), "Position data received on topic %s", subscriber_topicName.c_str());
    this->totalMessagesSubscribed += 1;
    pathCallback(*msg);
  };
  subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(subscriber_topicName, 10, topic_callback);
}

void PositionSubscriber::pathCallback(const geometry_msgs::msg::PointStamped &msg) {
    IMUdata imu_data = this->imuHashMap.getNewest(msg.header.stamp);
    if (imuHashMap.foundHash != false)
    {
      geometry_msgs::msg::PoseStamped curr_pose;
      curr_pose.header = msg.header;
      curr_pose.header.frame_id = this->frame_id;
      curr_pose.pose.orientation = imu_data.orientation;
      curr_pose.pose.position = msg.point;
      this->path.header = curr_pose.header;
      this->path.header.frame_id = this->frame_id;
      this->path.poses.push_back(curr_pose);

      this->publisher_->publish(this->path);
      this->totalMessagesPublished += 1;
      RCLCPP_INFO(this->get_logger(), "Path data published on topic %s with %f accuracy", this->publisher_topicName.c_str(), (totalMessagesPublished / totalMessagesSubscribed));
    }
}

void ImageSubscriber::image_callback() {
    try
    {   
        std::lock_guard<std::mutex> lock(this->image_mutex);
        cv::imshow(this->topicName, this->cv_ptr->image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("image_listener"), "Could not convert image");
    }
}