#ifndef TUW_SPIKE_CAMERA__GEOMETRY_HPP_
#define TUW_SPIKE_CAMERA__GEOMETRY_HPP_

#include <opencv2/core.hpp>

namespace tuw_spike_camera {

class Line2d : public cv::Vec3d {
  public:
    // Make type interchangeable with base class
    using cv::Vec3d::Vec3d;
    Line2d(const cv::Vec3d &vec) // NOLINT(google-explicit-constructor)
        : cv::Vec3d(vec) {}
    Line2d(const cv::Vec3d &&vec) // NOLINT(google-explicit-constructor)
        : cv::Vec3d(vec) {}


};

void test() {
    cv::Vec3d vec{0.0, 0.0, 0.0};

    Line2d line = vec;
    cv::Vec3d test = line;
}

} // namespace tuw_spike_camera

#endif // TUW_SPIKE_CAMERA__GEOMETRY_HPP_
