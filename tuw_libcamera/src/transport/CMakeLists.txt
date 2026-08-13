add_library(tuw_libcamera_transport_node SHARED
  transport_node.cpp)

target_link_libraries(tuw_libcamera_transport_node
  ${pluginlib_TARGETS}
  ${image_transport_TARGETS}
  tuw_libcamera_common)
