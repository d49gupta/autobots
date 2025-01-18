cmake_minimum_required(VERSION 3.8)
project(autobots)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-Wno-reorder)
add_compile_options(-Wno-unused-but-set-variable)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/input_nodes
    ${CMAKE_CURRENT_SOURCE_DIR}/include/output_nodes/extra
    ${CMAKE_CURRENT_SOURCE_DIR}/include/output_nodes
    ${OpenCV_INCLUDE_DIRS}
    /usr/include
    ${cv_bridge_INCLUDE_DIRS}
)

set(OpenCV_DIR $ENV{OpenCV_DIR})
find_package(OpenCV REQUIRED core imgproc imgcodecs highgui opencv_dnn)
find_package(cv_bridge REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_library(GPIO_LIB NAMES gpiod PATHS /usr/lib/aarch64-linux-gnu)
find_library(i2c_LIB NAMES i2c PATHS /usr/lib/aarch64-linux-gnu)

# add_executable(car_publisher 
#   src/input_nodes/counter_publisher.cpp
#   src/input_nodes/imu_publisher.cpp
#   src/input_nodes/main_publisher.cpp
#   src/input_nodes/csv_reader.cpp
# )

add_executable(car_subscriber
  src/output_nodes/subscriber.cpp
  src/output_nodes/main.cpp
  src/output_nodes/alphaBetaFilter.cpp
)

add_executable(test_hashmap
  src/output_nodes/hashMap.cpp
)

# add_executable(gpio_control
#   src/output_nodes/extra/servo_control.cpp
#   src/output_nodes/extra/main_gpio.cpp
# )

# target_link_libraries(car_publisher
#     ${i2c_LIB}
# )

# target_link_libraries(gpio_control 
#     ${OpenCV_LIBS}
#     ${GPIO_LIB}
# )

target_link_libraries(car_subscriber
    ${OpenCV_LIBS}
)

# ament_target_dependencies(car_publisher rclcpp std_msgs sensor_msgs)
ament_target_dependencies(car_subscriber rclcpp std_msgs sensor_msgs cv_bridge)

install(TARGETS
  # car_publisher
  car_subscriber
  test_hashmap
  # gpio_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()