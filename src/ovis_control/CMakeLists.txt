add_library(
  ovis_control
  SHARED
  src/ovis_control.cpp
)
target_include_directories(
  ovis_control
  PUBLIC
  include
)
ament_target_dependencies(
  ovis_control
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)
# prevent pluginlib from using boost
target_compile_definitions(ovis_control PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface ovis_control.xml)

install(
  TARGETS
  ovis_control
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)


  ament_add_gmock(test_ovis_control test/test_ovis_control.cpp)
  target_include_directories(test_ovis_control PRIVATE include)
  ament_target_dependencies(
    test_ovis_control
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )

ament_export_include_directories(
  include
)
ament_export_libraries(
  ovis_control
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
