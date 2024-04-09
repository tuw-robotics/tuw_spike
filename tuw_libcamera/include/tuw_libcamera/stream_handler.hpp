#ifndef TUW_LIBCAMERA__STREAM_HANDLER_HPP_
#define TUW_LIBCAMERA__STREAM_HANDLER_HPP_

#include <libcamera/stream.h>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/header.hpp>

#include "tuw_libcamera/buffer_context.hpp"

#include "tuw_libcamera_capture_node_parameters.hpp"

namespace tuw_libcamera {

class StreamHandler {
  public:
    using Ptr = std::unique_ptr<StreamHandler>;
    virtual void publish_buffer(const BufferContext &ctx,
                                const std_msgs::msg::Header &header) = 0;
};

StreamHandler::Ptr
create_stream_handler(rclcpp::Node *node,
                      const libcamera::StreamConfiguration &stream_cfg,
                      const Params::Streams::MapStreamRoles &params);

} // namespace tuw_libcamera

#endif // TUW_LIBCAMERA__STREAM_HANDLER_HPP_