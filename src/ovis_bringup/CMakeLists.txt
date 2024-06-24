cmake_minimum_required(VERSION 3.8)
project(ovis_bringup)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
# Install Python scripts
install(PROGRAMS
  scripts/publish_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
