#pragma once
#include <math/Vector3.h>

namespace aeroforge::visualization
{
    struct ScreenPoint
    {
        double x = 0;
        double y = 0;
        bool visible = false;
    };
    // Software perspective projection; OpenGL/ImGui owns final rasterization.
    // World NED -> render East/Up/North = (N.y, -N.z, N.x).
    class Camera3D
    {
    public:
        void orbit(double deltaYawRad, double deltaPitchRad);
        void zoom(double amount);
        ScreenPoint project(const math::Vector3& ned,
            double width, double height) const;
        double distance() const { return distance_; }
    private:
        double yaw_ = 0.65;
        double pitch_ = 0.4;
        double distance_ = 16.0;
    };
}
