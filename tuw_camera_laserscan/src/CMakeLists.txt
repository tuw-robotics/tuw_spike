add_library(tuw_camera_laserscan_ray_localizer_node SHARED
  ray_localizer_node.cpp
  ray_localizer.cpp
  geometry.cpp)

generate_parameter_library(tuw_camera_laserscan_ray_localizer_parameters
  ray_localizer_parameters.yaml)

target_link_libraries(tuw_camera_laserscan_ray_localizer_node PUBLIC
  tuw_camera_laserscan_common
  tuw_camera_laserscan_ray_localizer_parameters)

ament_target_dependencies(tuw_camera_laserscan_ray_localizer_node PUBLIC
  tf2
  tf2_ros
  tf2_geometry_msgs)

# Add stack traces on crashes
target_link_libraries(tuw_camera_laserscan_ray_localizer_node PRIVATE Backward::Object)
