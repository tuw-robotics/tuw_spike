cmake_minimum_required(VERSION 3.10)
project(tuw_spike_control)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Boost REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Add library and library alias
add_library(tuw_spike_control_interface SHARED
        src/tuw_spike_control_interface.cpp
)
# add_executable(tuw_spike_control_if src/tuw_spike_control_interface.cpp)
target_include_directories(tuw_spike_control_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tuw_spike_control_interface PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17
target_link_libraries(tuw_spike_control_interface PUBLIC
  Boost::headers
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ament_index_cpp::ament_index_cpp)

################################
## ros2_control plugin export ##

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(tuw_spike_control_interface PRIVATE "TUW_SPIKE_CONTROL_BUILDING_LIBRARY")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface hardware_interface.xml)

# Add library aliases
add_library(tuw_spike_control::hardware_interface ALIAS tuw_spike_control_interface)

############
## demo   ##

# Simple hello-world executable that exercises the tuw_spike_control_interface
# library by calling its on_configure() lifecycle callback.
add_executable(demo src/demo.cpp)
target_compile_features(demo PUBLIC c_std_99 cxx_std_20)
target_link_libraries(demo PUBLIC
  tuw_spike_control_interface
  hardware_interface::hardware_interface
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle)

install(
        TARGETS demo
        DESTINATION lib/${PROJECT_NAME}
)

################
## diffdrive  ##

# Differential-drive node: subscribes to a stamped twist on cmd_vel and
# converts it to left/right wheel angular velocities.
add_executable(diffdrive src/diffdrive_node.cpp)
target_include_directories(diffdrive PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(diffdrive PUBLIC c_std_99 cxx_std_20)
target_link_libraries(diffdrive PUBLIC
  tuw_spike_control_interface
  hardware_interface::hardware_interface
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${geometry_msgs_TARGETS})

install(
        TARGETS diffdrive
        DESTINATION lib/${PROJECT_NAME}
)

############
## others ##

# Setup install targets
install(
        DIRECTORY include/
        DESTINATION include
)
install(
        TARGETS
        tuw_spike_control_interface
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

## Launch and config files ##
install(
        DIRECTORY launch config firmware
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

## Ament package exports ##
ament_export_include_directories(
        include
)
ament_export_libraries(
        tuw_spike_control::hardware_interface
)
ament_export_targets(
        export_${PROJECT_NAME}
)

ament_package()
