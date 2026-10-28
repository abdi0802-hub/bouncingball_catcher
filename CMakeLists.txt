cmake_minimum_required(VERSION 3.8)
project(bouncingball_catcher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node rclcpp sensor_msgs cv_bridge)
target_link_libraries(camera_node ${OpenCV_LIBS})

add_executable(game_node src/game_node.cpp)
ament_target_dependencies(game_node rclcpp sensor_msgs std_msgs cv_bridge OpenCV)
target_link_libraries(game_node ${OpenCV_LIBS})

install(TARGETS
  camera_node
  game_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
