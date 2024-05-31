#include <array>
#include <cmath>
#include <utility>

#include "rclcpp/logging.hpp"
#include "tuw_spike_camera/ray_localizer.hpp"

#include <cv_bridge/cv_bridge.h>

// Need for tf2 conversion function to link
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "tuw_spike_camera/filter_kernel.hpp"
#include "tuw_spike_camera/geometry.hpp"

using namespace std::chrono_literals;

namespace tuw_spike_camera {

static constexpr auto KERNEL_DIFF = std::to_array<int16_t>({1, 0, -1});
static constexpr auto KERNEL_GAUSS = gaussian<int16_t, 5>(100, 0.8);

// static constexpr auto KERNEL = convolve(KERNEL_DIFF, KERNEL_GAUSS);
static constexpr auto KERNEL = KERNEL_DIFF;

struct RayLocalizer::ProcessingState {
    cv::Mat image;
    cv::Mat debug_image;
    cv::Matx33d homography;
    cv::Matx33d inv_homography;
    cv::Matx33d debug_transform;
    cv::Rect2d viewport;
    ProjPoint2d ray_center;
    std::optional<std::pair<ProjPoint2d, ProjPoint2d>> prev_ray;
};

RayLocalizer::RayLocalizer(const rclcpp::Logger &logger,
                           std::shared_ptr<tf2_ros::Buffer> tfBuffer,
                           std::unique_ptr<ParamListener> paramListener,
                           image_transport::Publisher debug_pub)
    : logger(logger), tf_buffer(std::move(tfBuffer)),
      param_listener(std::move(paramListener)),
      debug_pub(std::move(debug_pub)) {
    params = this->param_listener->get_params();
}

sensor_msgs::msg::LaserScan::UniquePtr RayLocalizer::process_frame(
    const sensor_msgs::msg::Image::ConstSharedPtr &image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info) {
    ProcessingState state{};

    // Update parameters
    if (param_listener->is_old(params)) {
        params = param_listener->get_params();
    }

    // Convert input image and setup debug image
    auto img = cv_bridge::toCvShare(image, "mono8");
    state.image = img->image;

    // Get image viewport
    int width = img->image.cols;
    int height = img->image.rows;
    state.viewport = {0.0, 0.0, (double)(width - 1), (double)(height - 1)};

    // Retrieve ray plane -> image plane homography
    auto homography = get_homography(info);
    if (!homography) {
        return nullptr;
    }
    state.homography = *homography;
    state.inv_homography = homography->inv();

    // Initialize debug image from input image
    setup_debug_image(state);

    // Initialize laser scan parameters
    state.ray_center = state.homography * ProjPoint2d(0, 0);
    ProjPoint2d start_pt =
        state.inv_homography *
        ProjPoint2d(state.viewport.tl().x, state.viewport.br().y);
    ProjPoint2d end_pt =
        state.inv_homography *
        ProjPoint2d(state.viewport.br().x, state.viewport.br().y);

    double start_angle = atan2(start_pt.y(), start_pt.x());
    double end_angle = atan2(end_pt.y(), end_pt.x());
    if (start_angle > end_angle) {
        std::swap(start_angle, end_angle);
    }
    
    double angle_increment =
        (end_angle - start_angle) / (double)(params.num_rays - 1);

    // Setup laser scan message
    auto laser_scan = std::make_unique<sensor_msgs::msg::LaserScan>();
    laser_scan->header.frame_id = params.ray_frame;
    laser_scan->header.stamp = image->header.stamp;
    laser_scan->angle_increment = (float)angle_increment;
    laser_scan->angle_min = (float)start_angle;
    laser_scan->angle_max = (float)end_angle;
    laser_scan->time_increment = 0;
    laser_scan->scan_time = 0;
    laser_scan->range_min = 0;
    laser_scan->range_max = 100.0;
    laser_scan->ranges.reserve(params.num_rays);

    // Populate laser scan ranges
    for (int64_t i = 0; i < params.num_rays; i++) {
        double angle = start_angle + (double)i * angle_increment;
        laser_scan->ranges.push_back(ray_cast(state, angle));
    }

    // Publish debug image
    std_msgs::msg::Header debug_header;
    debug_header.frame_id = params.ray_frame;
    debug_header.stamp = image->header.stamp;
    cv_bridge::CvImage dbg_img{debug_header, "bgr8", state.debug_image};
    debug_pub.publish(dbg_img.toImageMsg());

    // Return laser scan
    return laser_scan;
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

std::optional<cv::Matx33d> RayLocalizer::get_homography(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info) {
    cv::Matx33d cam_intrinsic{info->k.data()};
    cv::Matx34d cam_extrinsic;
    try {
        cam_extrinsic =
            get_camera_extrinsic(info->header.stamp, info->header.frame_id);
    } catch (const tf2::TransformException &e) {
        RCLCPP_ERROR(logger, "Failed to get camera transform: %s", e.what());
        return std::nullopt;
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

    return homography;
}

float RayLocalizer::ray_cast(ProcessingState &state, double angle) const {
    // Homography H transforms points from ray plane -> image plane
    // H^(-T) transforms lines from ray plane -> image plane
    ProjLine2d line = ProjLine2d({0, 0}, angle);
    ProjLine2d line_img = state.inv_homography.t() * line;

    // Clip ray to image bounds
    auto intersection = ray_clip(static_cast<cv::Point2d>(state.ray_center),
                                 line_img.direction(), state.viewport);

    if (intersection) {
        auto [start, end] = *intersection;
        auto ray_start = static_cast<cv::Point>(start);
        auto ray_end = static_cast<cv::Point>(end);

        debug_line(state, {255, 0, 0}, ray_start, ray_end);

        auto edge = detect_edge(state, ray_start, ray_end);
        if (edge) {
            // Get distance to origin
            double distance = cv::norm(static_cast<cv::Point2d>(*edge));
            return (float)distance;
        }
    }

    return std::numeric_limits<float>::infinity();
}

std::optional<ProjPoint2d>
RayLocalizer::detect_edge(ProcessingState &state, cv::Point start,
                          cv::Point end) const {
    std::optional<ProjPoint2d> detected;

    // Convolve filter kernel along ray
    FilterLineIterator<uint8_t, int16_t> line{state.image, std::move(start),
                                              std::move(end), KERNEL};

    // Line entry point
    std::optional<cv::Point> entry;

    while (++line) {
        auto [pos, filter_value] = *line;

        if (!entry && -filter_value > params.edge_enter_threshold) {
            // Record entry point
            entry = pos;
        } else if (entry && filter_value > params.edge_exit_threshold) {
            debug_point(state, {0, 255, 0}, *entry);
            debug_point(state, {0, 0, 255}, pos);

            ProjPoint2d pos_entry =
                state.inv_homography * ProjPoint2d(*entry);
            ProjPoint2d pos_exit =
                state.inv_homography * ProjPoint2d(pos);
            if (state.prev_ray) {
                auto [last_entry, last_exit] = *state.prev_ray;
                ProjLine2d entry_line = pos_entry.cross(last_entry);
                //ProjLine2d exit_line = pos_exit.cross(last_exit);
                double w1 = entry_line.distance(pos_exit);
                double w2 = entry_line.distance(last_exit);
                double edge_width = (w1 + w2) * 0.5;


                ProjLine2d dbg_line = state.inv_homography.t() * entry_line;
                debug_vector(state, {0, 255, 0}, *entry, dbg_line.direction() * 20);

                if (params.edge_min_width <= edge_width &&
                    (params.edge_max_width < 0 ||
                     edge_width < params.edge_max_width)) {
                    RCLCPP_DEBUG(
                        logger,
                        "Detected edge with w1 = %.1fmm w2 = %.1fmm",
                        w1*1e3, w2*1e3
                    );
                    debug_line(state, {255, 255, 0}, *entry, pos);
                    detected = pos_entry;
                }
            }

            state.prev_ray.emplace(pos_entry, pos_exit);
            break;
        }
    }

    return detected;
}

cv::Vec2d RayLocalizer::get_gradient(const cv::Mat &image,
                                     const cv::Point &point) const {

    int kernel_size = (int)params.edge_sobel_filter_size;
    if (params.edge_gradient_filter == "scharr") {
        kernel_size = cv::FILTER_SCHARR;
    }

    double dx, dy;
    cv::Mat roi(image, cv::Rect(point, cv::Size(1, 1)));
    cv::Sobel(roi, {&dx, 1}, CV_64F, 1, 0, kernel_size);
    cv::Sobel(roi, {&dy, 1}, CV_64F, 0, 1, kernel_size);
    return {dx, dy};
}

void RayLocalizer::setup_debug_image(ProcessingState &state) const {
    cv::Matx33d debug_transform;

    if (params.debug_warped) {
        // Create an affine transform to map the ray XY plane to the debug
        // image dimensions
        auto scale = (double)params.debug_img_size / params.debug_real_size;
        auto offset = (double)params.debug_img_size / 2.0;

        // clang-format off
        cv::Matx33d debug_affine{scale,  0,      0,
                                 0, -scale, offset,
                                 0,      0,      1};
        // clang-format on
        debug_transform = debug_affine * state.inv_homography;

        // Warp source image into debug image
        cv::warpPerspective(
            state.image, state.debug_image, debug_transform,
            cv::Size((int)params.debug_img_size, (int)params.debug_img_size),
            cv::InterpolationFlags::INTER_NEAREST,
            cv::BorderTypes::BORDER_CONSTANT);
    } else {
        // Resize source image into debug image
        double scale = (double)params.debug_img_size / (double)state.image.cols;
        cv::resize(state.image, state.debug_image, cv::Size(), scale, scale,
                   cv::INTER_AREA);

        // clang-format off
        debug_transform = {
            scale, 0, 0,
            0, scale, 0,
            0,     0, 1
        };
        // clang-format on
    }

    // Convert debug image to color
    cv::cvtColor(state.debug_image, state.debug_image, cv::COLOR_GRAY2BGR);

    state.debug_transform = debug_transform;
}

void RayLocalizer::debug_point(const ProcessingState &state,
                               const cv::Scalar &color,
                               const cv::Point2d &point) {
    ProjPoint2d pt = state.debug_transform * ProjPoint2d(point.x, point.y);
    if (!pt.at_infinity()) {
        cv::circle(state.debug_image, static_cast<cv::Point>(pt), 2, color,
                   cv::FILLED);
    }
}

void RayLocalizer::debug_line(const ProcessingState &state,
                              const cv::Scalar &color, const cv::Point2d &start,
                              const cv::Point2d &end) {
    ProjPoint2d start_dbg =
        state.debug_transform * ProjPoint2d(start.x, start.y);
    ProjPoint2d end_dbg = state.debug_transform * ProjPoint2d(end.x, end.y);
    if (!start_dbg.at_infinity() && !end_dbg.at_infinity()) {
        cv::line(state.debug_image, static_cast<cv::Point>(start_dbg),
                 static_cast<cv::Point>(end_dbg), color);
    }
}

void RayLocalizer::debug_vector(const ProcessingState &state,
                                const cv::Scalar &color,
                                const cv::Point2d &origin,
                                const cv::Vec2d &vector) {
    debug_line(state, color, origin, cv::Vec2d(origin) + vector);
}

} // namespace tuw_spike_camera