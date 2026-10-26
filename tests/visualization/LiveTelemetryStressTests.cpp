#include <visualization/LiveTelemetryPublisher.h>
#include <visualization/LiveVisualizationModel.h>
#include <gtest/gtest.h>
#include <thread>
using namespace aeroforge::visualization;
TEST(LiveTelemetryStressTests, PublisherConsumerCanRunConcurrently)
{
    using aeroforge::math::Quaternion;
    auto publisher = std::make_shared<LiveTelemetryPublisher>();
    std::thread producer([&]
        {
            for (int i = 0;i < 5000;++i)
            {
                const double t = static_cast<double>(i) / 1000.0;
                publisher->publish({ t,{t,0,0},{1,0,0},Quaternion::identity(),{0,0,0},
                    {t,0,0},{1,0,0},Quaternion::identity(),{0,0,0},{0,0,0},{0,0,0},{0,0,0} });
            }
        });
    LiveVisualizationModel model(publisher, 64);
    for (int i = 0;i < 2000;++i)model.update();
    producer.join();model.update();
    ASSERT_TRUE(model.hasSnapshot());
    EXPECT_LE(model.getTrajectoryHistory().size(), 64u);
    EXPECT_TRUE(model.getCurrentSnapshot().truthPositionN.isFinite());
}
