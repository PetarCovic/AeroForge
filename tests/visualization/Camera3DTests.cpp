#include <visualization/Camera3D.h>
#include <gtest/gtest.h>
#include <cmath>
using namespace aeroforge::visualization;
TEST(Camera3DTests, ProjectsOriginToCenter)
{
    Camera3D camera;
    const auto p = camera.project({ 0,0,0 }, 800, 600);
    EXPECT_TRUE(p.visible);
    EXPECT_NEAR(p.x, 400, 1e-9);
    EXPECT_NEAR(p.y, 300, 1e-9);
}
TEST(Camera3DTests, RejectsInvalidViewport)
{
    Camera3D camera;
    EXPECT_FALSE(camera.project({ 0,0,0 }, 0, 600).visible);
}
TEST(Camera3DTests, ZoomClampsAndOrbitDoesNotThrow)
{
    Camera3D camera;
    camera.orbit(0.1, 0.2);
    camera.zoom(1000);
    EXPECT_GE(camera.distance(), 2.0);
    camera.zoom(-1000);
    EXPECT_LE(camera.distance(), 1000.0);
}
