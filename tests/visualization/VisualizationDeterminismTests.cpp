#include <visualization/RecordedTelemetryDataSource.h>
#include <visualization/VisualizationModel.h>
#include <gtest/gtest.h>
using namespace aeroforge::visualization;
TEST(VisualizationDeterminismTests, PlaybackNeverMutatesSourceSnapshots)
{
    using aeroforge::math::Quaternion;
    const VisualizationSnapshot a{ 0,{0,0,0},{1,0,0},Quaternion::identity(),{0,0,0},
        {0,0,0},{1,0,0},Quaternion::identity(),{0,0,0},{0,0,0},{0,0,0},{0,0,0} };
    auto b = a;b.simulationTimeS = 10;b.truthPositionN.x = 10;
    auto source = std::make_shared<RecordedTelemetryDataSource>(std::vector<VisualizationSnapshot>{a, b});
    VisualizationModel first(source), second(source);
    for (double t : {0.0, 0.5, 1.5, 4.5, 9.0, 10.0})
    {
        first.seek(t);second.seek(t);
        EXPECT_DOUBLE_EQ(first.getCurrentSnapshot().truthPositionN.x,
            second.getCurrentSnapshot().truthPositionN.x);
    }
    EXPECT_DOUBLE_EQ(source->sampleAt(10).truthPositionN.x, 10.0);
    EXPECT_DOUBLE_EQ(source->sampleAt(0).truthPositionN.x, 0.0);
}
