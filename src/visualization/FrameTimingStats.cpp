#include <visualization/FrameTimingStats.h>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace aeroforge::visualization
{
    FrameTimingStats::FrameTimingStats(std::size_t capacity) :capacity_(capacity)
    {
        if (!capacity_) throw std::invalid_argument("Frame window capacity must be positive");
    }
    void FrameTimingStats::addFrameSeconds(double seconds)
    {
        if (!std::isfinite(seconds) || seconds <= 0.0)
            throw std::invalid_argument("Frame duration must be finite and positive");
        if (milliseconds_.size() == capacity_)milliseconds_.pop_front();
        milliseconds_.push_back(seconds * 1000.0);
    }
    FrameTimingSummary FrameTimingStats::summary(double budgetMs) const
    {
        if (!std::isfinite(budgetMs) || budgetMs <= 0)
            throw std::invalid_argument("Frame budget must be finite and positive");
        FrameTimingSummary result;
        result.count = milliseconds_.size();
        if (result.count == 0) return result;
        std::vector<double> v(milliseconds_.begin(), milliseconds_.end());
        std::sort(v.begin(), v.end());
        const auto quantile = [&](double p)
            {
                const double f = p * static_cast<double>(v.size() - 1);
                const auto lo = static_cast<std::size_t>(f);
                const auto hi = std::min(lo + 1, v.size() - 1);
                return v[lo] + (v[hi] - v[lo]) * (f - static_cast<double>(lo));
            };
        result.meanMs = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
        result.p50Ms = quantile(0.5);result.p95Ms = quantile(0.95);result.p99Ms = quantile(0.99);
        result.estimatedFps = 1000.0 / result.meanMs;
        result.overBudgetPercent = 100.0 * std::count_if(v.begin(), v.end(),
            [&](double ms) {return ms > budgetMs;}) / v.size();
        return result;
    }
}
