#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

class DronePublisher : public rclcpp::Node {
public:
    DronePublisher() : Node("drone_publisher") {
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path_topic", 10, std::bind(&DronePublisher::pathCallback, this, std::placeholders::_1));
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/drone_marker", 10);
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr path_msg) {
        if (path_msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty path!");
            return;
        }

        // Get the latest pose from the path
        auto latest_pose = path_msg->poses.back().pose;

        // Create a marker to represent the drone
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = path_msg->header.frame_id;
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "drone";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;  // Represents the drone
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set marker position and orientation to match the latest pose
        marker.pose = latest_pose;

        // Set marker scale (size of the drone)
        marker.scale.x = 1.0;  // Length
        marker.scale.y = 0.5;  // Width
        marker.scale.z = 0.2;  // Height

        // Set marker color
        marker.color.r = 0.0;
        marker.color.g = 0.5;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        // Publish the marker
        marker_pub_->publish(marker);
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DronePublisher>());
    rclcpp::shutdown();
    return 0;
}
