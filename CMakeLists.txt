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

add_executable(autonomous_car_project 
    src/publisher_lambda_function.cpp
    src/subscriber_lambda_function.cpp
    src/main.cpp
)
ament_target_dependencies(autonomous_car_project rclcpp std_msgs)

install(TARGETS
  autonomous_car_project
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()