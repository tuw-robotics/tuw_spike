add_library(tuw_libcamera_transport_node SHARED
  transport_node.cpp)

ament_target_dependencies(tuw_libcamera_transport_node
  pluginlib
  image_transport)

target_link_libraries(tuw_libcamera_transport_node
  tuw_libcamera_common)
