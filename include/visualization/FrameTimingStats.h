#pragma once
#include <cstddef>
#include <deque>

namespace aeroforge::visualization
{
    struct FrameTimingSummary
    {
        std::size_t count = 0;
        double meanMs = 0.0;
        double p50Ms = 0.0;
        double p95Ms = 0.0;
        double p99Ms = 0.0;
        double estimatedFps = 0.0;
        double overBudgetPercent = 0.0;
    };
    class FrameTimingStats
    {
    public:
        explicit FrameTimingStats(std::size_t capacity = 600);
        void addFrameSeconds(double seconds);
        FrameTimingSummary summary(double budgetMs = 16.6666667) const;
    private:
        std::size_t capacity_;
        std::deque<double> milliseconds_;
    };
}
