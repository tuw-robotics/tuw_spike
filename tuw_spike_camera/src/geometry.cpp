#include "tuw_spike_camera/geometry.hpp"
#include <cmath>
#include <cstdlib>

namespace tuw_spike_camera {

ProjPoint2d::ProjPoint2d(const cv::Vec3d &vec) : cv::Vec3d(vec) {}

ProjPoint2d::ProjPoint2d(const cv::Vec3d &&vec) : cv::Vec3d(vec) {}

ProjPoint2d::ProjPoint2d(const double x, const double y)
    : cv::Vec3d(x, y, 1.0) {}

ProjPoint2d::ProjPoint2d(const cv::Vec2d &point)
    : ProjPoint2d(point(0), point(1)) {}

ProjPoint2d::ProjPoint2d(const cv::Point2d &point)
    : ProjPoint2d(point.x, point.y) {}

bool ProjPoint2d::at_infinity() const { return abs((*this)(2)) <= DBL_EPSILON; }

double ProjPoint2d::x() const { return (*this)(0) / (*this)(2); }

double ProjPoint2d::y() const { return (*this)(1) / (*this)(2); }

double ProjPoint2d::distance(const ProjPoint2d &other) const {
    if (at_infinity() || other.at_infinity()) {
        return std::numeric_limits<double>::infinity();
    }

    const double dx = x() - other.x();
    const double dy = y() - other.y();
    return sqrt(dx * dx + dy * dy);
}

ProjLine2d::ProjLine2d(const cv::Vec3d &vec) : cv::Vec3d(vec) {}

ProjLine2d::ProjLine2d(const cv::Vec3d &&vec) : cv::Vec3d(vec) {}

ProjLine2d::ProjLine2d(const cv::Point2d &point, const cv::Vec2d &dir)
    : cv::Vec3d(-dir(1), dir(0), dir(1) * point.x - dir(0) * point.y) {}

ProjLine2d::ProjLine2d(const cv::Point2d &point, double angle)
    : ProjLine2d(point, {cos(angle), sin(angle)}) {}

ProjPoint2d ProjLine2d::intersect(const ProjLine2d &other) const {
    return cross(other);
}

cv::Vec2d ProjLine2d::direction() const {
    return cv::normalize(cv::Vec2d((*this)(1), -(*this)(0)));
}

double ProjLine2d::distance(const ProjPoint2d &point) const {
    const double a = (*this)(0);
    const double b = (*this)(1);
    double tmp = dot(point) / point(2);
    // doesn't use abs, which always returns 0 because of a compilation bug
    if (tmp < 0.0)
        tmp = -tmp;
    return tmp / sqrt(a * a + b * b);
}

std::optional<std::pair<cv::Vec2d, cv::Vec2d>>
ray_clip(const cv::Vec2d &ray_start, const cv::Vec2d &direction,
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