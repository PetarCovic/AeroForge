#include <hil/HilTelemetryStore.h>

#include <cmath>
#include <stdexcept>

namespace aeroforge::hil
{
    void HilTelemetryStore::publish(
        const HilTelemetryPoint& point,
        const HilHealthSnapshot& health)
    {
        if (!std::isfinite(point.simulationTimeS) ||
            !std::isfinite(point.stepExecutionMs) ||
            !std::isfinite(point.communicationMs) ||
            !std::isfinite(point.commandLatencyMs) ||
            point.simulationTimeS < 0.0 ||
            point.stepExecutionMs < 0.0 ||
            point.communicationMs < 0.0 ||
            point.commandLatencyMs < -1.0)
        {
            throw std::invalid_argument("Invalid HIL telemetry sample");
        }

        std::lock_guard lock(mutex_);
        ring_[next_] = point;
        next_ = (next_ + 1) % ring_.size();
        if (count_ < ring_.size()) ++count_;
        health_ = health;
        ++revision_;
    }

    void HilTelemetryStore::publishHealth(const HilHealthSnapshot& health)
    {
        std::lock_guard lock(mutex_);
        health_ = health;
        ++revision_;
    }

    HilTelemetrySnapshot HilTelemetryStore::snapshot() const
    {
        std::lock_guard lock(mutex_);
        HilTelemetrySnapshot result{};
        result.count = count_;
        result.revision = revision_;
        result.health = health_;

        const std::size_t oldest =
            (next_ + ring_.size() - count_) % ring_.size();
        for (std::size_t i = 0; i < count_; ++i)
            result.points[i] = ring_[(oldest + i) % ring_.size()];
        return result;
    }

    void HilTelemetryStore::reset()
    {
        std::lock_guard lock(mutex_);
        next_ = 0;
        count_ = 0;
        health_ = {};
        ++revision_;
    }
}
