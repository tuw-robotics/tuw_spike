cmake_minimum_required(VERSION 3.8)
project(tuw_libcamera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LibCamera REQUIRED IMPORTED_TARGET libcamera)

add_library(tuw_libcamera_capture_node SHARED
  src/capture_node.cpp
  src/convert.cpp
  src/stream_handler.cpp
  src/buffer_context.cpp
  src/camera_controls_handler.cpp)

target_include_directories(tuw_libcamera_capture_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tuw_libcamera_capture_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

generate_parameter_library(tuw_libcamera_capture_node_parameters
  src/capture_node_parameters.yaml)

ament_target_dependencies(tuw_libcamera_capture_node
  rclcpp_components
  sensor_msgs
  OpenCV
  image_transport)

target_link_libraries(tuw_libcamera_capture_node
  PkgConfig::LibCamera
  tuw_libcamera_capture_node_parameters)

rclcpp_components_register_node(tuw_libcamera_capture_node
  PLUGIN "tuw_libcamera::CaptureNode"
  EXECUTABLE tuw_libcamera_capture)

ament_export_targets(export_tuw_libcamera)
install(
  TARGETS
  tuw_libcamera_capture_node
  tuw_libcamera_capture_node_parameters
  EXPORT export_tuw_libcamera
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
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
