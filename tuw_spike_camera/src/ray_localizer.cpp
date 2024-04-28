#include <array>
#include <utility>

#include "rclcpp/logging.hpp"
#include "tuw_spike_camera/ray_localizer.hpp"

#include <cv_bridge/cv_bridge.h>

// Need for tf2 conversion function to link
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "tuw_spike_camera/geometry.hpp"

using namespace std::chrono_literals;

namespace tuw_spike_camera {

template <typename T, size_t N1, size_t N2>
static constexpr std::array<T, N1 + N2 - 1>
convolve(const std::array<T, N1> &a, const std::array<T, N2> &b) {
    static_assert(N1 > 0);
    static_assert(N2 > 0);
    static_assert((N2 % 2) == 1);

    // 01234
    // 012

    //--01234--
    // 012
    // 0123456

    std::array<T, N1 + N2 - 1> result{};
    for (size_t i = 0; i < result.size(); i++) {
        for (size_t j = 0; j < N2; j++) {
            size_t k = i - j;
            result[i] += ((k < N1) ? a[k] : 0) * b[j];
        }
    }
    return result;
}

static constexpr auto KERNEL_DIFF = std::to_array<int16_t>({-1, 0, 1});

static constexpr auto KERNEL_ONES =
    std::to_array<int16_t>({0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0});

static constexpr auto KERNEL = convolve(KERNEL_DIFF, KERNEL_ONES);
// static constexpr auto KERNEL = KERNEL_DIFF;

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

    auto cv_img = cv_bridge::toCvShare(image, "mono8");

    if (param_listener->is_old(params)) {
        params = param_listener->get_params();
    }

    int width = cv_img->image.cols;
    int height = cv_img->image.rows;

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
    if (cv::determinant(homography) < 0) {
        // If the determinant is zero, make it positive by negating the matrix
        // to ensure that directions are preserved. Since homographies are only
        // defined up to a scalar multiple this represents the same
        // transformation.
        homography *= -1;
    }
    auto inv_homography = homography.inv();

    ProjPoint2d start_pt =
        inv_homography * ProjPoint2d(0.0, (double)(height - 1));
    ProjPoint2d end_pt =
        inv_homography * ProjPoint2d((double)(width - 1), (double)(height - 1));

    double start_angle = atan2(start_pt.y(), start_pt.x());
    double end_angle = atan2(end_pt.y(), end_pt.x());

    ProjPoint2d ray_center_img = homography * ProjPoint2d(0, 0);

    cv::Rect2d viewport{0.5, 0.5, (double)(width - 1), (double)(height - 1)};

    for (int64_t i = 0; i < params.num_rays; i++) {
        double angle = std::lerp(start_angle, end_angle,
                                 i / (double)(params.num_rays - 1));
        // Homography H transforms points from ray plane -> image plane
        // H^(-T) transforms lines from ray plane -> image plane
        ProjLine2d line_img = inv_homography.t() * ProjLine2d({0, 0}, angle);

        auto intersection =
            ray_intersect(static_cast<cv::Point2d>(ray_center_img),
                          line_img.direction(), viewport);

        if (intersection) {
            auto [start, end] = *intersection;
            auto start_pt = static_cast<cv::Point>(start);
            auto end_pt = static_cast<cv::Point>(end);
            cv::LineIterator it{cv_img->image, start_pt, end_pt};

            // Convolve kernel along line
            std::array<std::pair<cv::Point, uint8_t>, KERNEL.size()> history{};
            std::size_t history_base = 0;
            // Fill history with values of ray start
            history.fill({it.pos(), **it});
            for (size_t i = 0; i <= (history.size() / 2 + it.count); i++) {
                uint8_t &value = **it;

                // Save historic values for kernel application
                history[history_base] = {it.pos(), value};
                history_base = (history_base + 1) % history.size();

                // Mark ray for debugging
                // value = 0;

                // Advance iterator
                if (i < (size_t)it.count) {
                    ++it;
                }

                // Skip calculation for ray points out of bounds
                if (i <= history.size() / 2) {
                    continue;
                }

                // Apply kernel convolution
                int16_t convolve_result = 0;
                for (size_t j = 0; j < history.size(); j++) {
                    size_t k = (history_base + j) % history.size();
                    convolve_result +=
                        history[k].second * KERNEL[KERNEL.size() - j - 1];
                }

                // Get position for which kernel was calculated
                size_t center_idx =
                    (history_base + history.size() / 2) % history.size();
                cv::Point kernel_pos = history[center_idx].first;

                // Mark edge for debugging
                if (abs(convolve_result) > params.edge_threshold) {
                    cv::circle(cv_img->image, kernel_pos, 5, 128, cv::FILLED);
                }
            }
        }
    }

    // Create an affine transform to map the ray XY plane to the debug
    // image dimensions
    double scale = params.debug_img_size / params.debug_real_size;
    double offset = params.debug_img_size / 2;

    // clang-format off
    cv::Matx33d debug_affine{scale, 0,      0,
                             0, scale, offset,
                             0,     0,      1};
    // clang-format on

    std_msgs::msg::Header debug_header;
    debug_header.frame_id = params.ray_frame;
    debug_header.stamp = image->header.stamp;
    cv_bridge::CvImage debug_image{debug_header, "mono8"};

    cv::warpPerspective(cv_img->image, debug_image.image,
                        debug_affine * inv_homography,
                        cv::Size(params.debug_img_size, params.debug_img_size),
                        cv::InterpolationFlags::INTER_NEAREST,
                        cv::BorderTypes::BORDER_CONSTANT);

    debug_pub.publish(cv_img->toImageMsg());
    // debug_pub.publish(debug_image.toImageMsg());
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