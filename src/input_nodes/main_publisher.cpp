#include "publisher_lamda_function.hpp"
#include "imu_publisher.hpp"
#include <memory>
#include <string>
#include <thread>

void run_minimial_publisher()
{
  auto minimal_publisher = std::make_shared<MinimalPublisher>();
  rclcpp::spin(minimal_publisher);
  rclcpp::shutdown();
}

// void run_imu_publisher()
// {
//   auto imu_publisher = std::make_shared<ImuPublisher>();
//   rclcpp::spin(imu_publisher);
//   rclcpp::shutdown();
// }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::thread minimal_publisher_thread(run_minimial_publisher);
  std::thread minimal_publisher_thread_copy(run_minimial_publisher);
  // std::thread imu_publisher_thread(run_imu_publisher);

  minimal_publisher_thread.join();
  minimal_publisher_thread_copy.join();
  // imu_publisher_thread.join();

  return 0;
}