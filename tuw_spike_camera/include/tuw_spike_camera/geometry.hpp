#ifndef TUW_SPIKE_CAMERA__GEOMETRY_HPP_
#define TUW_SPIKE_CAMERA__GEOMETRY_HPP_

#include <cmath>
#include <opencv2/core.hpp>

namespace tuw_spike_camera {

/**
 * @brief Represents a point in P² as a homogenous vector of R³
 * @details
 * The point (x,y) of R² is represented as k*(x, y, 1) for any real, nonzero k.
 * The vector (a, b, 0) represents a point on the line at infinity, in
 * the "direction" (b, -a).
*/
class ProjPoint2d : public cv::Vec3d {
  public:
    // Make type interchangeable with base class
    using cv::Vec3d::Vec3d;
    ProjPoint2d(const cv::Vec3d &vec) // NOLINT(google-explicit-constructor)
        : cv::Vec3d(vec) {}
    ProjPoint2d(const cv::Vec3d &&vec) // NOLINT(google-explicit-constructor)
        : cv::Vec3d(vec) {}
    ProjPoint2d(double x, double y)
        : cv::Vec3d(x, y, 1.0) {}
    ProjPoint2d(const cv::Vec2d point)
        : ProjPoint2d(point(0), point(1)) {}


    /**
     * @return True, if the point is on the line at infinity
    */
    bool at_infinity() {
        return abs((*this)(2)) <= DBL_EPSILON;
    }

    /**
     * @return The x coordinate of the represented point
     * @note Only use this conversion if the point is not at infinity.
     * @see ProjPoint2d::at_infinity
    */
    double x() const {
        return (*this)(0) / (*this)(2);
    }

    /**
     * @return The y coordinate of the represented point
     * @note Only use this conversion if the point is not at infinity.
     * @see ProjPoint2d::at_infinity
    */
    double y() const {
        return (*this)(1) / (*this)(2);
    }

    /**
     * @brief Convert to a point in K²
     * @tparam K the data type of the point coordinates
     * @note Only use this conversion if the point is not at infinity.
     * @see ProjPoint2d::at_infinity
    */
    template<typename K>
    explicit operator cv::Point_<K>() const {
        return {
            static_cast<K>(x()),
            static_cast<K>(y())
        };
    }
};

/**
 * @brief Represents a line in P² as a homogenous vector of R³
 * @details
 * The vector (a, b, c) represents the line ax + by + c = 0.
*/
class ProjLine2d : public cv::Vec3d {
  public:
    // Make type interchangeable with base class
    using cv::Vec3d::Vec3d;
    ProjLine2d(const cv::Vec3d &vec) // NOLINT(google-explicit-constructor)
        : cv::Vec3d(vec) {}
    ProjLine2d(const cv::Vec3d &&vec) // NOLINT(google-explicit-constructor)
        : cv::Vec3d(vec) {}

    /**
     * @brief Create a line using a point on the line and a direction vector.
     * @param point A point on the line
     * @param dir A vector tangent to the line direction
    */
    ProjLine2d(const cv::Vec2d point, const cv::Vec2d dir)
        : cv::Vec3d(-dir(1), dir(0), -point.dot(dir)) {}

    /**
     * @brief Create a line using a point on the line and the line angle.
     * @param point A point on the line
     * @param angle The line angle in radians, counterclockwise from the positive x axis.
    */
    ProjLine2d(const cv::Vec2d point, double angle)
        : ProjLine2d(point, {cos(angle), sin(angle)}) {}

    ProjPoint2d intersect(const ProjLine2d &other) const {
        return cross(other);
    }
};

} // namespace tuw_spike_camera

#endif // TUW_SPIKE_CAMERA__GEOMETRY_HPP_
