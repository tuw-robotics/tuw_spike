cmake_minimum_required(VERSION 3.8)
project(tuw_spike_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(xacro REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(controller_manager REQUIRED)

# Add library and library alias
add_library(tuw_spike_control_interface SHARED
        src/tuw_spike_control_interface.cpp
)
# add_executable(tuw_spike_control_if src/tuw_spike_control_interface.cpp)
target_include_directories(tuw_spike_control_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tuw_spike_control_interface PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17
ament_target_dependencies(tuw_spike_control_interface Boost hardware_interface pluginlib
rclcpp
rclcpp_lifecycle
realtime_tools)
# target_link_libraries(tuw_spike_control_interface Boost::system)

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
        DIRECTORY launch config
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
