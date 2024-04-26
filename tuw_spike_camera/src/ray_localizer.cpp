#include <utility>

#include "rclcpp/logging.hpp"
#include "tuw_spike_camera/ray_localizer.hpp"

namespace tuw_spike_camera {

RayLocalizer::RayLocalizer(const rclcpp::Logger &logger,
                           std::shared_ptr<tf2_ros::Buffer> tfBuffer,
                           std::unique_ptr<ParamListener> paramListener)
    : logger(logger), tf_buffer(std::move(tfBuffer)),
      param_listener(std::move(paramListener)) {
    params = this->param_listener->get_params();
}

void RayLocalizer::process_frame(
    const sensor_msgs::msg::Image::ConstSharedPtr &image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info) {

    if (param_listener->is_old(params)) {
        params = param_listener->get_params();
    }

    /*cv::Matx34d cam_extrinsic;
    try {
        cam_extrinsic = get_camera_extrinsic(info->header.stamp, info->header.frame_id);
    } catch (const tf2::TransformException &e) {
        RCLCPP_ERROR(logger, "Failed to get camera transform: %s", e.what());
        return;
    }*/

        (void) image;
    RCLCPP_INFO(logger, "Received frame with stamp: %d",
                info->header.stamp.sec);
}

cv::Matx34d
RayLocalizer::get_camera_extrinsic(const rclcpp::Time &time,
                                   const std::string &optical_frame) {
    auto transform =
        tf_buffer->lookupTransform(optical_frame, params.ray_frame, time);

    tf2::Transform tmp;
    tf2::fromMsg(transform.transform, tmp);
    const auto &rot = tmp.getBasis();
    const auto &trans = tmp.getOrigin();
    // clang-format off
    return {rot[0][0], rot[0][1], rot[0][2], trans[0],
            rot[1][0], rot[1][1], rot[1][2], trans[1],
            rot[2][0], rot[2][1], rot[2][2], trans[2]};
    // clang-format on
}

} // namespace tuw_spike_camera