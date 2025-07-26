cmake_minimum_required(VERSION 3.5)
project(artag)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  artag/camera_node.py
  artag/detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()