#ifndef TUW_SPIKE_CAMERA__RAY_LOCALIZER_HPP_
#define TUW_SPIKE_CAMERA__RAY_LOCALIZER_HPP_

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/core/matx.hpp>
#include <rclcpp/logger.hpp>
#include <tf2_ros/buffer.h>

#include "tuw_spike_camera_ray_localizer_parameters.hpp"

namespace tuw_spike_camera {

class RayLocalizer {
  public:
    explicit RayLocalizer(const rclcpp::Logger &logger,
                          std::shared_ptr<tf2_ros::Buffer> tfBuffer,
                          std::unique_ptr<ParamListener> paramListener);
    void
    process_frame(const sensor_msgs::msg::Image::ConstSharedPtr &image,
                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info);

  private:
    rclcpp::Logger logger;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::unique_ptr<ParamListener> param_listener;
    Params params;

    cv::Matx34d get_camera_extrinsic(const rclcpp::Time &time,
                                     const std::string &optical_frame);
};

} // namespace tuw_spike_camera

#endif // TUW_SPIKE_CAMERA__RAY_LOCALIZER_HPP_
