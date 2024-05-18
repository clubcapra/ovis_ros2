add_library(
  ovis_ros2_control
  SHARED
  src/ovis_hw_interface.cpp
)
target_include_directories(
  ovis_ros2_control
  PUBLIC
  include
)
ament_target_dependencies(
  ovis_ros2_control
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)
# prevent pluginlib from using boost
target_compile_definitions(ovis_ros2_control PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface ovis_ros2_control.xml)

install(
  TARGETS
  ovis_ros2_control
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)


  ament_add_gmock(test_ovis_hw_interface test/test_ovis_hw_interface.cpp)
  target_include_directories(test_ovis_hw_interface PRIVATE include)
  ament_target_dependencies(
    test_ovis_hw_interface
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )

ament_export_include_directories(
  include
)
ament_export_libraries(
  ovis_ros2_control
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
