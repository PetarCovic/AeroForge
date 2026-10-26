#pragma once
#include <visualization/VisualizationSnapshot.h>
#include <cstddef>
#include <vector>

namespace aeroforge::visualization
{
    struct PlotSample
    {
        double timeS;
        double truth;
        double estimate;
    };
    class PlotDataBuilder
    {
    public:
        // Fixed-size time-based selection with extrema preserved in each bucket.
        static std::vector<std::size_t> decimate(
            const std::vector<VisualizationSnapshot>& samples,
            std::size_t maxPoints,
            int field);
        static double component(const VisualizationSnapshot& s, int field, bool estimated);
    };
}
