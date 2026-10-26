#include <visualization/PlotDataBuilder.h>
#include <gtest/gtest.h>
#include <algorithm>
using namespace aeroforge::visualization;
namespace
{
    VisualizationSnapshot s(double t, double x)
    {
        using aeroforge::math::Vector3;using aeroforge::math::Quaternion;
        return { t,{x,0,0},{0,0,0},Quaternion::identity(),{0,0,0},
          {x,0,0},{0,0,0},Quaternion::identity(),{0,0,0},
          {0,0,0},{0,0,0},{0,0,0} };
    }
}
TEST(PlotDataBuilderTests, KeepsEndpointsAndExtreme)
{
    std::vector<VisualizationSnapshot> v;
    for (int i = 0;i < 1000;++i)v.push_back(s(i, i == 497 ? 1000.0 : 0.0));
    auto idx = PlotDataBuilder::decimate(v, 100, 0);
    ASSERT_LE(idx.size(), 100u);
    EXPECT_EQ(idx.front(), 0u);
    EXPECT_EQ(idx.back(), 999u);
    EXPECT_NE(std::find(idx.begin(), idx.end(), 497u), idx.end());
}
TEST(PlotDataBuilderTests, RejectsInvalidCapacity)
{
    EXPECT_THROW(PlotDataBuilder::decimate({ s(0,0) }, 3, 0), std::invalid_argument);
}
