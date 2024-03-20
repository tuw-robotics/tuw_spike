#include <cstdio>
#include <libcamera/libcamera.h>
#include <rclcpp/rclcpp.hpp>

namespace tuw_libcamera {

class LibcameraNode : public rclcpp::Node {
  public:
    LibcameraNode(const rclcpp::NodeOptions &options)
        : Node("libcamera_node", options) {

        std::unique_ptr<libcamera::CameraManager> cm =
            std::make_unique<libcamera::CameraManager>();
        cm->start();

        RCLCPP_INFO(get_logger(), "Cameras:");
        for (auto const &camera : cm->cameras()) {
            RCLCPP_INFO(get_logger(), " -> %s", camera->id().c_str());
        }
    }
};

}; // namespace tuw_libcamera

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tuw_libcamera::LibcameraNode)