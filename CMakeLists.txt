cmake_minimum_required(VERSION 3.8)
project(autonomous_car_project)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(car_publisher 
  src/publisher_lambda_function.cpp
  src/main_publisher.cpp
)

add_executable(car_subscriber
  src/subscriber_lambda_function.cpp
  src/main_subscriber.cpp
)

ament_target_dependencies(car_publisher rclcpp std_msgs)
ament_target_dependencies(car_subscriber rclcpp std_msgs)

install(TARGETS
  car_publisher
  car_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()