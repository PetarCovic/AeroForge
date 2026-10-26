#include <visualization/FrameTimingStats.h>
#include <gtest/gtest.h>
using namespace aeroforge::visualization;
TEST(FrameTimingStatsTests, ComputesWindowQuantiles)
{
    FrameTimingStats stats(4);
    for(const double s:{0.01,0.02,0.03,0.04})stats.addFrameSeconds(s);
    const auto result=stats.summary();
    EXPECT_EQ(result.count,4u);
    EXPECT_NEAR(result.meanMs,25.0,1e-9);
    EXPECT_NEAR(result.p50Ms,25.0,1e-9);
    EXPECT_NEAR(result.p95Ms,38.5,1e-9);
    EXPECT_NEAR(result.p99Ms,39.7,1e-9);
    EXPECT_NEAR(result.estimatedFps,40.0,1e-9);
}
TEST(FrameTimingStatsTests, EnforcesBoundedCapacity)
{
    FrameTimingStats stats(2);
    stats.addFrameSeconds(0.01);stats.addFrameSeconds(0.02);stats.addFrameSeconds(0.03);
    EXPECT_EQ(stats.summary().count,2u);
    EXPECT_NEAR(stats.summary().meanMs,25.0,1e-9);
}
