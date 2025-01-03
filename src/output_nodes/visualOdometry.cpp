#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>

void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        // Convert ROS2 image message with mono8 encoding to OpenCV image
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        
        // Now you can use cv_ptr->image as a normal OpenCV image
        cv::imshow("Mono8 Image", cv_ptr->image);
        cv::waitKey(1);  // Display the image
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("image_listener"), "Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_listener");
    auto subscription = node->create_subscription<sensor_msgs::msg::Image>(
        "camera/image", 10, image_callback);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
