cmake_minimum_required(VERSION 3.8)
project(ur_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config srdf urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
