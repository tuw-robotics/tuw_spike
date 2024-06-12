cmake_minimum_required(VERSION 3.8)
project(tuw_camera_laserscan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include(FetchContent)

# Add stack traces on crashes
FetchContent_Declare(backward
  GIT_REPOSITORY https://github.com/bombela/backward-cpp
  GIT_TAG master
)
FetchContent_MakeAvailable(backward)
set_property(TARGET backward_object PROPERTY POSITION_INDEPENDENT_CODE ON)

# Common flags and dependencies
add_library(tuw_camera_laserscan_common INTERFACE)
target_include_directories(tuw_camera_laserscan_common INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tuw_camera_laserscan_common INTERFACE c_std_99 cxx_std_20)  # Require C99 and C++20

ament_target_dependencies(tuw_camera_laserscan_common INTERFACE
  rclcpp_components
  sensor_msgs
  image_transport
  OpenCV
  cv_bridge)

# Include node source
add_subdirectory(src)

# Register nodes
rclcpp_components_register_node(tuw_camera_laserscan_ray_localizer_node
  PLUGIN "tuw_camera_laserscan::RayLocalizerNode"
  EXECUTABLE tuw_camera_laserscan_ray_localizer)

# Export libraries
ament_export_targets(export_tuw_camera_laserscan)
install(
  TARGETS
  tuw_camera_laserscan_common
  tuw_camera_laserscan_ray_localizer_node
  tuw_camera_laserscan_ray_localizer_parameters
  EXPORT export_tuw_camera_laserscan
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch calibration config maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
