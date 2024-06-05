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
    ProjPoint2d(const cv::Vec3d &vec);   // NOLINT(google-explicit-constructor)
    ProjPoint2d(const cv::Vec3d &&vec);  // NOLINT(google-explicit-constructor)
    ProjPoint2d(double x, double y);     // NOLINT(google-explicit-constructor)
    ProjPoint2d(const cv::Vec2d &point); // NOLINT(google-explicit-constructor)
    ProjPoint2d(                         // NOLINT(google-explicit-constructor)
        const cv::Point2d &point);

    /**
     * @return True, if the point is on the line at infinity
     */
    [[nodiscard]] bool at_infinity() const;

    /**
     * @return The x coordinate of the represented point
     * @note Only use this conversion if the point is not at infinity.
     * @see ProjPoint2d::at_infinity
     */
    [[nodiscard]] double x() const;

    /**
     * @return The y coordinate of the represented point
     * @note Only use this conversion if the point is not at infinity.
     * @see ProjPoint2d::at_infinity
     */
    [[nodiscard]] double y() const;

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
    [[nodiscard]] double distance(const ProjPoint2d &other) const;
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
    ProjLine2d(const cv::Vec3d &vec);  // NOLINT(google-explicit-constructor)
    ProjLine2d(const cv::Vec3d &&vec); // NOLINT(google-explicit-constructor)

    /**
     * @brief Create a line using a point on the line and a direction vector.
     * @param point A point on the line
     * @param dir A vector tangent to the line direction
     */
    ProjLine2d(const cv::Point2d &point, const cv::Vec2d &dir);

    /**
     * @brief Create a line using a point on the line and the line angle.
     * @param point A point on the line
     * @param angle The line angle in radians, counterclockwise from the
     * positive x axis.
     */
    ProjLine2d(const cv::Point2d &point, double angle);

    [[nodiscard]] ProjPoint2d intersect(const ProjLine2d &other) const;

    /**
     * @return The line direction as a vector in R²
     */
    [[nodiscard]] cv::Vec2d direction() const;

    double distance(const ProjPoint2d &point);
};

/**
 * Clip the ray given by start and direction vector to the given bounds
 * @param ray_start Ray start in R²
 * @param direction Ray direction as vector in R²
 * @param bounds The axis aligned bounds to clip the ray to
 * @return Clipped ray start and end points if the ray intersects the bounds,
 * std::nullopt otherwise.
 */
std::optional<std::pair<cv::Vec2d, cv::Vec2d>>
ray_clip(const cv::Vec2d &ray_start, const cv::Vec2d &direction,
         const cv::Rect2d &bounds);

} // namespace tuw_spike_camera

#endif // TUW_SPIKE_CAMERA__GEOMETRY_HPP_
