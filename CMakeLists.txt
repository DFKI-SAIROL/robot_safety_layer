cmake_minimum_required(VERSION 3.8)
project(robot_safety_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(franka_custom_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
)

# Create a library for sharing safety logic
add_library(safety_lib SHARED
  src/safety_layer.cpp
  src/vis.cpp
  src/model_utils.cpp
)

ament_target_dependencies(safety_lib
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  franka_custom_msgs
  pinocchio
)

# Create the node
add_executable(franka_controller_node src/franka_controller_node.cpp)
target_link_libraries(franka_controller_node safety_lib)
ament_target_dependencies(franka_controller_node
  rclcpp
)

# Export for other packages
ament_export_include_directories(include)
ament_export_libraries(safety_lib)
ament_export_dependencies(rclcpp geometry_msgs std_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs franka_custom_msgs pinocchio)

install(TARGETS safety_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS franka_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
