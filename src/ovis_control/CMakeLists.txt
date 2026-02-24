cmake_minimum_required(VERSION 3.16)
project(ovis_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  angles
  tf2
  tf2_ros
  ros2_control_test_assets
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
link_directories(${PROJECT_SOURCE_DIR}/lib/${CMAKE_LIBRARY_ARCHITECTURE})

include_directories(
  include
)

add_library(
  ovis_control
  SHARED
  src/ovis_control.cpp
  src/kinova_api.cpp
  src/kinova_comm.cpp
  src/kinova_ros_types.cpp
  include/ovis_control/ovis_control.hpp
  include/kinova_driver/kinova_api.h
  include/kinova_driver/kinova_comm.h
  include/kinova_driver/kinova_ros_types.h
  include/kinova/Kinova.API.USBCommLayerUbuntu.h
  include/kinova/Kinova.API.USBCommandLayerUbuntu.h
  include/kinova/KinovaTypes.h
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

# Install

# prevent pluginlib from using boost
target_compile_definitions(ovis_control PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface ovis_control.xml)

# Copy over Kinova .so binary blobs to /usr/lib
install(FILES
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/USBCommLayerUbuntu.so
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/USBCommandLayerUbuntu.so
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/EthCommLayerUbuntu.so
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/EthCommandLayerUbuntu.so
  DESTINATION lib
)

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



if(BUILD_TESTING)
  ament_add_gmock(test_ovis_control test/test_ovis_control.cpp)
  target_include_directories(test_ovis_control PRIVATE include)
  ament_target_dependencies(
    test_ovis_control
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )
endif()

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