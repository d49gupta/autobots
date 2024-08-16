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
    ${OpenCV_INCLUDE_DIRS}
)

set(OpenCV_DIR $ENV{OpenCV_DIR})
find_package(OpenCV REQUIRED core imgproc imgcodecs highgui opencv_dnn)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(car_publisher 
  src/publisher_lambda_function.cpp
  src/imu_publisher.cpp
  src/main_publisher.cpp
)

add_executable(car_subscriber
  src/subscriber_lambda_function.cpp
  src/main_subscriber.cpp
)

add_executable(image_detection
  src/image_detection.cpp
)

target_link_libraries(image_detection 
    ${OpenCV_LIBS}
)

ament_target_dependencies(car_publisher rclcpp std_msgs)
ament_target_dependencies(car_subscriber rclcpp std_msgs)

install(TARGETS
  car_publisher
  car_subscriber
  image_detection
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()