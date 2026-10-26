#include <visualization/Camera3D.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aeroforge::visualization
{
    namespace
    {
        math::Vector3 render(const math::Vector3& p)
        {
            return { p.y,-p.z,p.x };
        }
    }
    void Camera3D::orbit(double yaw, double pitch)
    {
        if (!std::isfinite(yaw) || !std::isfinite(pitch))
            throw std::invalid_argument("Non-finite camera motion");
        yaw_ += yaw;
        pitch_ = std::clamp(pitch_ + pitch, -1.45, 1.45);
    }
    void Camera3D::zoom(double amount)
    {
        if (!std::isfinite(amount)) throw std::invalid_argument("Non-finite camera zoom");
        distance_ = std::clamp(distance_ * std::exp(-amount * 0.12), 2.0, 1000.0);
    }
    ScreenPoint Camera3D::project(const math::Vector3& ned, double w, double h) const
    {
        if (!ned.isFinite() || !std::isfinite(w) || !std::isfinite(h) || w <= 0 || h <= 0)
            return {};
        const auto p = render(ned);
        const math::Vector3 eye(
            distance_ * std::cos(pitch_) * std::sin(yaw_),
            distance_ * std::sin(pitch_),
            distance_ * std::cos(pitch_) * std::cos(yaw_));
        const math::Vector3 forward = (-eye).normalize();
        const math::Vector3 right = forward.cross(math::Vector3(0, 1, 0)).normalize();
        const math::Vector3 up = right.cross(forward).normalize();
        const auto relative = p - eye;
        const double depth = relative.dot(forward);
        if (depth <= 0.05) return {};
        constexpr double fovy = 0.8726646259971648; // 50 degrees
        const double scale = 1.0 / std::tan(fovy / 2.0);
        const double sx = relative.dot(right) * scale / (depth * (w / h));
        const double sy = relative.dot(up) * scale / depth;
        return { (sx * 0.5 + 0.5) * w,(0.5 - sy * 0.5) * h,true };
    }
}
