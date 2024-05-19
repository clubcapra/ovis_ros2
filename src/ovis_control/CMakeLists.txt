cmake_minimum_required(VERSION 3.16)
project(ovis_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  angles
  tf2
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  ovis_control
  SHARED
  src/ovis_control.cpp
  src/kinova_api.cpp
  src/kinova_comm.cpp
  src/kinova_ros_types.cpp
)

target_compile_features(ovis_control PUBLIC cxx_std_17)
target_include_directories(ovis_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/ovis_control>
)

ament_target_dependencies(
  ovis_control
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
  
  target_include_directories(
  ovis_control
  PUBLIC
  include
)


# prevent pluginlib from using boost
target_compile_definitions(ovis_control PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface ovis_control.xml)

install(
  DIRECTORY include/
  DESTINATION include
)


install(
  TARGETS
  ovis_control
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)


# if(BUILD_TESTING)
#   ament_add_gmock(test_ovis_control test/test_ovis_control.cpp)
#   target_include_directories(test_ovis_control PRIVATE include)
#   ament_target_dependencies(
#     test_ovis_control
#     hardware_interface
#     pluginlib
#     ros2_control_test_assets
#   )
# endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ovis_control
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()