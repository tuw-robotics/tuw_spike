add_library(tuw_libcamera_capture_node SHARED
  capture_node.cpp
  convert.cpp
  stream_handler.cpp
  buffer_context.cpp
  request_handler.cpp
  camera_controls_handler.cpp)

generate_parameter_library(tuw_libcamera_capture_node_parameters
  capture_node_parameters.yaml)

ament_target_dependencies(tuw_libcamera_capture_node
  camera_info_manager
  OpenCV)

target_link_libraries(tuw_libcamera_capture_node
  tuw_libcamera_common
  tuw_libcamera_capture_node_parameters
  PkgConfig::LibCamera)
