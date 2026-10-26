#include <visualization/TelemetryCsvLoader.h>
#include <gtest/gtest.h>
#include <sstream>
#include <string>
using namespace aeroforge::visualization;
namespace
{
    VisualizationSnapshot sample(double t)
    {
        using aeroforge::math::Vector3;
        return { t,{t,2,3},{4,5,6},aeroforge::math::Quaternion::identity(),{0,0,1},
            {t + 0.2,2,3},{4,5,6},aeroforge::math::Quaternion::identity(),{0,0,1},
            {0,0,0},{1,0,0},{0.5,0,0} };
    }
}
TEST(TelemetryCsvLoaderTests, CanonicalRoundTrip)
{
    std::stringstream csv;
    TelemetryCsvLoader::write(csv, { sample(0),sample(1.5) });
    auto result = TelemetryCsvLoader::parse(csv);
    ASSERT_EQ(result.size(), 2u);
    EXPECT_DOUBLE_EQ(result[1].simulationTimeS, 1.5);
    EXPECT_NEAR(result[1].estimatedPositionN.x, 1.7, 1e-12);
    EXPECT_DOUBLE_EQ(result[0].actualMomentB.x, 0.5);
}
TEST(TelemetryCsvLoaderTests, RejectsRepeatedTimestamp)
{
    std::stringstream csv;
    TelemetryCsvLoader::write(csv, { sample(0),sample(0) });
    EXPECT_THROW(TelemetryCsvLoader::parse(csv), std::runtime_error);
}
TEST(TelemetryCsvLoaderTests, RejectsMissingTruthColumns)
{
    std::stringstream csv("simulationTimeS,truthPositionN.x\n0,1\n");
    EXPECT_THROW(TelemetryCsvLoader::parse(csv), std::runtime_error);
}
TEST(TelemetryCsvLoaderTests, RejectsNonFiniteValues)
{
    std::stringstream csv;
    TelemetryCsvLoader::write(csv, { sample(0) });
    std::string content = csv.str();
    auto pos = content.find("\n0,");
    ASSERT_NE(pos, std::string::npos);
    content.replace(pos + 1, 1, "nAn");
    std::stringstream bad(content);
    EXPECT_THROW(TelemetryCsvLoader::parse(bad), std::runtime_error);
}
