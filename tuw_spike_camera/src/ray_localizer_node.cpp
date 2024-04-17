#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace tuw_spike_camera {

using namespace sensor_msgs::msg;

class RayLocalizerNode : public rclcpp::Node {
  public:
    RayLocalizerNode(const rclcpp::NodeOptions &options)
        : Node("ray_localizer", options) {}

  private:
};

}; // namespace tuw_spike_camera

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tuw_spike_camera::RayLocalizerNode)