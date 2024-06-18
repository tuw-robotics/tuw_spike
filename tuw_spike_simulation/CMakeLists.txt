cmake_minimum_required(VERSION 3.8)
project(tuw_spike_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(xacro REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED)

find_package(ignition-transport8 QUIET REQUIRED OPTIONAL_COMPONENTS log)

add_executable(spawn src/spawn_robot.cpp)
ament_target_dependencies(spawn PUBLIC rclcpp std_msgs)

target_link_libraries(spawn PUBLIC Boost::boost ignition-transport8::core)

install(TARGETS
  spawn
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY 
        world launch
        DESTINATION share/${PROJECT_NAME}
)


# Install Python executables
install(PROGRAMS
  scripts/spawn_robot.py
  scripts/circle_trajectory.py
  scripts/line_trajectory.py
  scripts/curve_trajectory.py
  scripts/combined_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
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
