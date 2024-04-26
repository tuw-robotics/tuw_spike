#include <utility>

#include "rclcpp/logging.hpp"
#include "tuw_spike_camera/ray_localizer.hpp"

#include <cv_bridge/cv_bridge.h>

// Need for tf2 conversion function to link
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace tuw_spike_camera {

RayLocalizer::RayLocalizer(const rclcpp::Logger &logger,
                           std::shared_ptr<tf2_ros::Buffer> tfBuffer,
                           std::unique_ptr<ParamListener> paramListener,
                           image_transport::Publisher debug_pub)
    : logger(logger), tf_buffer(std::move(tfBuffer)),
      param_listener(std::move(paramListener)),
      debug_pub(std::move(debug_pub)) {
    params = this->param_listener->get_params();
}

void RayLocalizer::process_frame(
    const sensor_msgs::msg::Image::ConstSharedPtr &image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info) {

    auto cv_img = cv_bridge::toCvShare(image, "bgr8");

    if (param_listener->is_old(params)) {
        params = param_listener->get_params();
    }

    cv::Matx33d cam_intrinsic{info->k.data()};
    cv::Matx34d cam_extrinsic;
    try {
        cam_extrinsic =
            get_camera_extrinsic(info->header.stamp, info->header.frame_id);
    } catch (const tf2::TransformException &e) {
        RCLCPP_ERROR(logger, "Failed to get camera transform: %s", e.what());
        return;
    }

    // Get projection matrix from the ray frame to the camera frame
    auto proj = cam_intrinsic * cam_extrinsic;

    // Extract columns 0, 1, 3 to get the homography matrix from
    // The ray XY plane to the camera plane

    // clang-format off
    cv::Matx33d homography{proj(0, 0), proj(0, 1), proj(0, 3),
                           proj(1, 0), proj(1, 1), proj(1, 3),
                           proj(2, 0), proj(2, 1), proj(2, 3)};
    // clang-format on
    auto inv_homography = homography.inv();

    // Create an affine transform to map the ray XY plane to the debug
    // image dimensions
    constexpr int debug_img_size = 400; // Size of the debug image
    constexpr double debug_real_size =
        0.2; // Size of the debug image viewport in meters
    constexpr double scale = debug_img_size / debug_real_size;
    constexpr double offset = debug_img_size / 2;

    // clang-format off
    cv::Matx33d debug_affine{scale, 0,      0,
                             0, scale, offset,
                             0,     0,      1};
    // clang-format on

    std_msgs::msg::Header debug_header;
    debug_header.frame_id = params.ray_frame;
    debug_header.stamp = image->header.stamp;
    cv_bridge::CvImage debug_image{debug_header, "bgr8"};

    cv::warpPerspective(cv_img->image, debug_image.image,
                        debug_affine * inv_homography,
                        cv::Size(debug_img_size, debug_img_size),
                        cv::InterpolationFlags::INTER_NEAREST,
                        cv::BorderTypes::BORDER_CONSTANT);

    debug_pub.publish(debug_image.toImageMsg());
}

cv::Matx34d
RayLocalizer::get_camera_extrinsic(const rclcpp::Time &time,
                                   const std::string &optical_frame) {
    auto transform =
        tf_buffer->lookupTransform(optical_frame, params.ray_frame, time, 10ms);

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