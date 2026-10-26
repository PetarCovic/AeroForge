#pragma once

#include <cstddef>
#include <stop_token>

namespace aeroforge::visualization
{
    class LiveTelemetryPublisher;

    struct SixDoFLiveRunResult
    {
        std::size_t publicationAttempts = 0;
        double lastSimulationTimeS = 0.0;
        bool cancelled = false;
    };

    // Runs AeroForge's actual 13-state SixDoFDynamicsModel/RK4 simulator.
    // Applied forces/moments are simplified and constant; this is not a
    // RocketDynamicsModel powered-flight simulation.
    //
    // Publishes at most the latest state through the existing thread-safe
    // publisher. In realtime mode, simulation is paced to approximately dtS;
    // publicationAttempts counts attempts, not guaranteed GUI frames.
    SixDoFLiveRunResult runSixDoFLiveSimulation(
        LiveTelemetryPublisher& publisher,
        std::stop_token stop,
        double dtS = 0.01,
        std::size_t steps = 1000,
        bool realtime = true);
}
