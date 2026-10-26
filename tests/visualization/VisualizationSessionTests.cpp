#include <visualization/VisualizationSession.h>
#include <visualization/TelemetryCsvLoader.h>
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <chrono>
using namespace aeroforge::visualization;
namespace
{
    VisualizationSnapshot s(double t)
    {
        using aeroforge::math::Quaternion;
        return { t,{t,0,0},{1,0,0},Quaternion::identity(),{0,0,0},
           {t,0,0},{1,0,0},Quaternion::identity(),{0,0,0},
           {0,0,0},{0,0,0},{0,0,0} };
    }
}
TEST(VisualizationSessionTests, LoadsSeeksAndAdvances)
{
    const auto path = std::filesystem::temp_directory_path() /
        ("aeroforge_gui_test_" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()) + ".csv");
    { std::ofstream out(path);TelemetryCsvLoader::write(out, { s(0),s(10) }); }
    VisualizationSession session;
    EXPECT_NO_THROW(session.loadRecording(path));
    std::filesystem::remove(path);
    EXPECT_EQ(session.mode(), VisualizationSession::Mode::Recorded);
    session.seek(3.5);
    ASSERT_NE(session.current(), nullptr);
    EXPECT_NEAR(session.current()->truthPositionN.x, 3.5, 1e-9);
    session.play();session.update(1.0);
    EXPECT_NEAR(session.currentTimeS(), 4.5, 1e-9);
    session.restart();EXPECT_DOUBLE_EQ(session.currentTimeS(), 0.0);
}
TEST(VisualizationSessionTests, ConsumesExternalPublisher)
{
    auto publisher = std::make_shared<LiveTelemetryPublisher>();
    VisualizationSession session;
    session.attachLive(publisher);
    publisher->publish(s(2));
    session.update(0.02);
    ASSERT_NE(session.current(), nullptr);
    EXPECT_DOUBLE_EQ(session.currentTimeS(), 2.0);
    session.stopLive();EXPECT_EQ(session.current(), nullptr);
}
