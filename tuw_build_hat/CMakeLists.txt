cmake_minimum_required(VERSION 3.20)
project(tuw_build_hat)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

###############
## build_hat ##

# Library implementing the BuildHatDriver.
add_library(build_hat SHARED src/tuw_build_hat/build_hat.cpp src/tuw_build_hat/sensor.cpp src/tuw_build_hat/motor.cpp)
target_include_directories(build_hat PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(build_hat PUBLIC c_std_99 cxx_std_20)
target_link_libraries(build_hat PUBLIC
  Boost::headers
)

####################
## demo ##

# Minimal executable that exercises BuildHatDriver by calling its init() method.
add_executable(demo src/demo.cpp)
target_compile_features(demo PUBLIC c_std_99 cxx_std_20)
target_link_libraries(demo PUBLIC
  build_hat
  Boost::program_options
)

install(
        TARGETS demo
        DESTINATION lib/${PROJECT_NAME}
)

install(
        TARGETS build_hat
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        DIRECTORY include/
        DESTINATION include
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
        build_hat
)
ament_export_targets(
        export_${PROJECT_NAME}
)

ament_package()
