#include <visualization/PlotDataBuilder.h>
#include <algorithm>
#include <stdexcept>
#include <cmath>
#include <array>

namespace aeroforge::visualization
{
    double PlotDataBuilder::component(const VisualizationSnapshot& s, int field, bool estimated)
    {
        switch (field)
        {
        case 0: return estimated ? s.estimatedPositionN.x : s.truthPositionN.x;
        case 1: return estimated ? s.estimatedPositionN.y : s.truthPositionN.y;
        case 2: return estimated ? s.estimatedPositionN.z : s.truthPositionN.z;
        case 3: return estimated ? s.estimatedVelocityN.x : s.truthVelocityN.x;
        case 4: return estimated ? s.estimatedVelocityN.y : s.truthVelocityN.y;
        case 5: return estimated ? s.estimatedVelocityN.z : s.truthVelocityN.z;
        case 6: return s.attitudeErrorB.x;
        case 7: return s.attitudeErrorB.y;
        case 8: return s.attitudeErrorB.z;
        case 9: return estimated ? s.actualMomentB.x : s.commandedMomentB.x;
        case 10:return estimated ? s.actualMomentB.y : s.commandedMomentB.y;
        case 11:return estimated ? s.actualMomentB.z : s.commandedMomentB.z;
        default: throw std::invalid_argument("Unknown plot field");
        }
    }
    std::vector<std::size_t> PlotDataBuilder::decimate(
        const std::vector<VisualizationSnapshot>& samples, std::size_t maxPoints, int field)
    {
        if (field < 0 || field>11) throw std::invalid_argument("Unknown plot field");
        if (maxPoints < 4) throw std::invalid_argument("maxPoints must be >= 4");
        const std::size_t n = samples.size();
        std::vector<std::size_t> selected;
        if (n <= maxPoints)
        {
            selected.reserve(n);
            for (std::size_t i = 0;i < n;++i) selected.push_back(i);
            return selected;
        }
        selected.reserve(maxPoints);
        selected.push_back(0);
        // Each bucket preserves min/max of both truth and estimate, hence 4 possible indices.
        const std::size_t bucketCount = (maxPoints - 2) / 4;
        for (std::size_t b = 0;b < bucketCount;++b)
        {
            const std::size_t begin = 1 + (n - 2) * b / bucketCount;
            const std::size_t end = 1 + (n - 2) * (b + 1) / bucketCount;
            std::size_t tMin = begin, tMax = begin, eMin = begin, eMax = begin;
            for (std::size_t i = begin + 1;i < end;++i)
            {
                const double t = component(samples[i], field, false);
                const double e = component(samples[i], field, true);
                if (t < component(samples[tMin], field, false))tMin = i;
                if (t > component(samples[tMax], field, false))tMax = i;
                if (e < component(samples[eMin], field, true))eMin = i;
                if (e > component(samples[eMax], field, true))eMax = i;
            }
            std::array<std::size_t, 4> indices = { tMin,tMax,eMin,eMax };
            std::sort(indices.begin(), indices.end());
            selected.insert(selected.end(), indices.begin(), std::unique(indices.begin(), indices.end()));
        }
        selected.push_back(n - 1);
        return selected;
    }
}
