#include "counter_publisher.hpp"
#include "imu_publisher.hpp"
#include <memory>
#include <string>
#include <thread>

void run_counter_publisher()
{
  auto counter_publisher = std::make_shared<counterPublisher>("counter_publisher", "counter_topic");
  rclcpp::spin(counter_publisher);
  rclcpp::shutdown();
}

// void run_imu_publisher()
// {
//   auto imu_publisher = std::make_shared<ImuPublisher>("imu_publisher", "imu_topic");
//   rclcpp::spin(imu_publisher);
//   rclcpp::shutdown();
// }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::thread counter_publisher_thread(run_counter_publisher);
  // std::thread imu_publisher_thread(run_imu_publisher);

  counter_publisher_thread.join();
  // imu_publisher_thread.join();

  return 0;
}