#ifndef TUW_SPIKE_CAMERA__GEOMETRY_HPP_
#define TUW_SPIKE_CAMERA__GEOMETRY_HPP_

#include <cmath>
#include <opencv2/core.hpp>
#include <optional>
#include <utility>

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
    ProjPoint2d(double x, double y) : cv::Vec3d(x, y, 1.0) {}
    ProjPoint2d(const cv::Vec2d &point) // NOLINT(google-explicit-constructor)
        : ProjPoint2d(point(0), point(1)) {}
    ProjPoint2d(const cv::Point2d &point) // NOLINT(google-explicit-constructor)
        : ProjPoint2d(point.x, point.y) {}

    /**
     * @return True, if the point is on the line at infinity
     */
    [[nodiscard]] bool at_infinity() const {
        return abs((*this)(2)) <= DBL_EPSILON;
    }

    /**
     * @return The x coordinate of the represented point
     * @note Only use this conversion if the point is not at infinity.
     * @see ProjPoint2d::at_infinity
     */
    [[nodiscard]] double x() const { return (*this)(0) / (*this)(2); }

    /**
     * @return The y coordinate of the represented point
     * @note Only use this conversion if the point is not at infinity.
     * @see ProjPoint2d::at_infinity
     */
    [[nodiscard]] double y() const { return (*this)(1) / (*this)(2); }

    /**
     * @brief Convert to a point in K²
     * @tparam K the data type of the point coordinates
     * @note Only use this conversion if the point is not at infinity.
     * @see ProjPoint2d::at_infinity
     */
    template <typename K> explicit operator cv::Point_<K>() const {
        return {static_cast<K>(x()), static_cast<K>(y())};
    }

    /**
     * @brief Calculate distance to another point
     * @param other the point to calculate the distance to
     * @return +infinity, if one of the points is at infinity,
     * their Euclidean distance otherwise.
     */
    [[nodiscard]] double distance(const ProjPoint2d &other) const {
        if (at_infinity() || other.at_infinity()) {
            return std::numeric_limits<double>::infinity();
        } else {
            double dx = x() - other.x();
            double dy = y() - other.y();
            return sqrt(dx * dx + dy * dy);
        }
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
    ProjLine2d(const cv::Point2d &point, const cv::Vec2d &dir)
        : cv::Vec3d(-dir(1), dir(0), dir(1) * point.x - dir(0) * point.y) {}

    /**
     * @brief Create a line using a point on the line and the line angle.
     * @param point A point on the line
     * @param angle The line angle in radians, counterclockwise from the
     * positive x axis.
     */
    ProjLine2d(const cv::Point2d &point, double angle)
        : ProjLine2d(point, {cos(angle), sin(angle)}) {}

    [[nodiscard]] ProjPoint2d intersect(const ProjLine2d &other) const {
        return cross(other);
    }

    /**
     * @return The line direction as a vector in R²
     */
    [[nodiscard]] cv::Vec2d direction() const {
        return cv::normalize(cv::Vec2d((*this)(1), -(*this)(0)));
    }

    double distance(const ProjPoint2d &point) {
        double a = (*this)(0), b = (*this)(1);
        return abs(dot(point) / point(2)) / sqrt(a*a + b*b);
    }
};

std::optional<std::pair<cv::Vec2d, cv::Vec2d>> inline ray_clip(
    const cv::Vec2d &ray_start, const cv::Vec2d &direction,
    const cv::Rect2d &bounds) {
    cv::Vec4d p = {-direction(0), direction(0), -direction(1), direction(1)};
    cv::Vec4d q = {
        ray_start(0) - bounds.x,
        bounds.x + bounds.width - ray_start(0),
        ray_start(1) - bounds.y,
        bounds.y + bounds.height - ray_start(1),
    };
    auto t = q.div(p);

    double t1 = 0, t2 = +std::numeric_limits<double>::infinity();

    for (int i = 0; i < 4; i++) {
        if (p(i) < -DBL_EPSILON) {
            t1 = std::max(t1, t(i));
        } else if (p(i) > +DBL_EPSILON) {
            t2 = std::min(t2, t(i));
        } else if (q(i) < 0) {
            // Line parallel to edge and outside of viewport
            return std::nullopt;
        } // else parallel and inside of viewport
    }

    if (t1 > t2) {
        return std::nullopt;
    }

    return std::make_pair(ray_start + t1 * direction,
                          ray_start + t2 * direction);
}

} // namespace tuw_spike_camera

#endif // TUW_SPIKE_CAMERA__GEOMETRY_HPP_
