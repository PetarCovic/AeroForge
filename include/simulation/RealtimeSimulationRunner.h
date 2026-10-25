#pragma once

#include <functional>
#include <simulation/RealtimeExecutionConfig.h>
#include <simulation/RealtimeExecutionResult.h>

namespace aeroforge::simulation
{
    using StepFunction =
        std::function<void(double simulationTimeS, double dtS)>;

    class RealtimeSimulationRunner
    {
    public:
        explicit RealtimeSimulationRunner(
            const RealtimeExecutionConfig& config);

        RealtimeExecutionResult run(
            const StepFunction& stepFunction) const;

    private:
        RealtimeExecutionConfig config;
    };
}