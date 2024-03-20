cmake_minimum_required(VERSION 3.8)
project(tuw_libcamera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LibCamera REQUIRED IMPORTED_TARGET libcamera)

add_library(tuw_libcamera_component SHARED
  src/tuw_libcamera_node.cpp)

target_include_directories(tuw_libcamera_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tuw_libcamera_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(tuw_libcamera_component
  rclcpp_components)
target_link_libraries(tuw_libcamera_component
  PkgConfig::LibCamera)

rclcpp_components_register_node(
  tuw_libcamera_component
  PLUGIN "tuw_libcamera::LibcameraNode"
  EXECUTABLE tuw_libcamera
)

ament_export_targets(export_tuw_libcamera_component)
install(
  TARGETS tuw_libcamera_component
  EXPORT export_tuw_libcamera_component
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
