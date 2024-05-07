#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tuw_spike_camera/ray_localizer.hpp>

#include "tuw_spike_camera_ray_localizer_parameters.hpp"

namespace tuw_spike_camera {

using namespace sensor_msgs::msg;

class RayLocalizerNode : public rclcpp::Node {
  public:
    explicit RayLocalizerNode(const rclcpp::NodeOptions &options)
        : Node("ray_localizer", options) {

        auto param_listener =
            std::make_unique<ParamListener>(get_node_parameters_interface());

        // Disable intra_process_comm for TF2 because of incompatible QoS
        auto tf_sub_opts =
            tf2_ros::detail::get_default_transform_listener_sub_options();
        auto tf_static_sub_opts = tf2_ros::detail::
            get_default_transform_listener_static_sub_options();
        tf_sub_opts.use_intra_process_comm =
            rclcpp::IntraProcessSetting::Disable;
        tf_static_sub_opts.use_intra_process_comm =
            rclcpp::IntraProcessSetting::Disable;

        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(
            *tf_buffer, this, true, tf2_ros::DynamicListenerQoS(),
            tf2_ros::StaticListenerQoS(), tf_sub_opts, tf_static_sub_opts);

        debug_pub = image_transport::create_publisher(
            this, "~/debug", rmw_qos_profile_sensor_data);

        auto ray_localizer = std::make_shared<RayLocalizer>(
            get_logger(), tf_buffer, std::move(param_listener), debug_pub);

        auto laser_scan_pub = create_publisher<sensor_msgs::msg::LaserScan>(
            "laser_scan", rclcpp::SensorDataQoS());

        camera_sub = image_transport::create_camera_subscription(
            this, "image_rect",
            [laser_scan_pub, ray_localizer](auto &img, auto &info) {
                auto scan = ray_localizer->process_frame(img, info);
                if (scan) {
                    laser_scan_pub->publish(std::move(scan));
                }
            },
            "raw", rmw_qos_profile_sensor_data);
    }

  private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    image_transport::Publisher debug_pub;
    image_transport::CameraSubscriber camera_sub;
};

} // namespace tuw_spike_camera

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tuw_spike_camera::RayLocalizerNode)