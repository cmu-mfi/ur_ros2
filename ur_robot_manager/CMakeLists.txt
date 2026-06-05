cmake_minimum_required(VERSION 3.8)
project(ur_robot_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Dependencies
set(DEPENDENCIES
  ament_cmake
  moveit_core
  moveit_ros_planning_interface
  moveit_servo
  rclcpp
  rclcpp_action
  tf2_ros
  robot_manager_interfaces
  std_srvs
  geometry_msgs
  ur_msgs
)


# Find dependencies
foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

# Define the executable
add_executable(
  ur_robot_manager 
  src/ur_robot_manager.cpp 
  src/joint_goal.cpp
  src/pose_goal.cpp
  src/home.cpp
  src/set_payload.cpp
  src/wrench_publisher.cpp
)

# Add include directories for the executable
target_include_directories(ur_robot_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(ee_state_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link the libraries
ament_target_dependencies(ur_robot_manager ${DEPENDENCIES})
ament_target_dependencies(ee_state_publisher ${DEPENDENCIES})
ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})

install(TARGETS ur_robot_manager
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS ee_state_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
