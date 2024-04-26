add_library(tuw_spike_camera_ray_localizer_node SHARED
  ray_localizer_node.cpp
  ray_localizer.cpp
  geometry.cpp)

generate_parameter_library(tuw_spike_camera_ray_localizer_parameters
  ray_localizer_parameters.yaml)

target_link_libraries(tuw_spike_camera_ray_localizer_node
  tuw_spike_camera_common
  tuw_spike_camera_ray_localizer_parameters)

ament_target_dependencies(tuw_spike_camera_ray_localizer_node
  tf2
  tf2_ros
  tf2_geometry_msgs)
