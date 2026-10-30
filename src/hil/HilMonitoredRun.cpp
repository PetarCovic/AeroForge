#include <hil/HilMonitoredRun.h>
#include <hil/HilPrecisePacer.h>

#include <hil/HilSimulationBridge.h>
#include <hil/HilSoftwareController.h>
#include <hil/HilTimingMonitor.h>
#include <hil/HilWireEndpoint.h>

#include <dynamics/ForceAccumulator.h>
#include <dynamics/InertiaProperties.h>
#include <dynamics/MassProperties.h>
#include <dynamics/MomentAccumulator.h>
#include <dynamics/RigidBodyQuaternionNormalization.h>
#include <dynamics/SixDoFDynamicsModel.h>

#include <math/Matrix.h>
#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <sensors/ImuModel.h>

#include <simulation/RK4Integrator.h>
#include <simulation/State.h>

#include <visualization/LiveTelemetryPublisher.h>
#include <visualization/VisualizationSnapshot.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <optional>
#include <span>
#include <stdexcept>
#include <vector>

namespace aeroforge::hil
{
    namespace
    {
        using Clock = std::chrono::steady_clock;

        struct Duplex
        {
            std::deque<std::uint8_t> hostToController;
            std::deque<std::uint8_t> controllerToHost;

            static std::size_t receive(
                std::deque<std::uint8_t>& queue,
                std::span<std::uint8_t> output)
            {
                const std::size_t count =
                    std::min(output.size(), queue.size());

                for (std::size_t i = 0; i < count; ++i)
                {
                    output[i] = queue.front();
                    queue.pop_front();
                }

                return count;
            }

            static std::size_t send(
                std::deque<std::uint8_t>& queue,
                std::span<const std::uint8_t> data)
            {
                for (const auto byte : data)
                {
                    queue.push_back(byte);
                }

                return data.size();
            }

            HilBytePort hostPort()
            {
                return {
                    [this](std::span<std::uint8_t> bytes)
                    {
                        return receive(
                            controllerToHost, bytes);
                    },

                    [this](std::span<const std::uint8_t> bytes)
                    {
                        return send(
                            hostToController, bytes);
                    }
                };
            }

            HilBytePort controllerPort()
            {
                return {
                    [this](std::span<std::uint8_t> bytes)
                    {
                        return receive(
                            hostToController, bytes);
                    },

                    [this](std::span<const std::uint8_t> bytes)
                    {
                        return send(
                            controllerToHost, bytes);
                    }
                };
            }
        };

        double milliseconds(
            Clock::duration duration)
        {
            return std::chrono::duration<
                double, std::milli>(duration).count();
        }

        double newestMs(
            const HilDurationStatistics& before,
            const HilDurationStatistics& after)
        {
            if (after.count <= before.count)
            {
                return -1.0;
            }

            return
                after.averageMs *
                static_cast<double>(after.count) -
                before.averageMs *
                static_cast<double>(before.count);
        }

        HilHealthSnapshot makeHealth(
            const HilSimulationBridge& bridge,
            const HilTimingMonitor& timing,
            double targetPeriodMs,
            bool running)
        {
            HilHealthSnapshot health{};

            const auto& bridgeStats = bridge.stats();
            const auto& timingStats = timing.statistics();

            health.mode =
                HilLinkMode::SOFTWARE_LOOPBACK;

            health.serialState =
                HilSerialWorkerState::STOPPED;

            health.watchdogState =
                bridge.watchdogState();

            health.running = running;

            health.queuedSamples =
                bridgeStats.queuedSamples;

            health.droppedSamples =
                bridgeStats.droppedSamples;

            health.acceptedCommands =
                bridgeStats.acceptedCommands;

            health.rejectedCommands =
                bridgeStats.rejectedCommands;

            health.deadlineMisses =
                timingStats.stepDeadlineMisses;

            health.stepCount =
                timingStats.simulationSteps.count;

            health.commandLatencyCount =
                timingStats.commandLatency.count;

            health.targetPeriodMs =
                targetPeriodMs;

            health.averageStepMs =
                timingStats.simulationSteps.averageMs;

            health.maximumStepMs =
                timingStats.simulationSteps.maximumMs;

            health.averageCommandLatencyMs =
                timingStats.commandLatency.averageMs;

            health.maximumCommandLatencyMs =
                timingStats.commandLatency.maximumMs;

            return health;
        }

        visualization::VisualizationSnapshot visualSample(
            double timeS,
            const simulation::State& state,
            const math::Vector3& commanded)
        {
            const math::Vector3 pos(
                state[0], state[1], state[2]);

            const math::Vector3 vel(
                state[3], state[4], state[5]);

            const math::Quaternion q(
                state[6], state[7], state[8], state[9]);

            const math::Vector3 omega(
                state[10], state[11], state[12]);

            const math::Vector3 zero{};

            return {
                timeS,
                pos,
                vel,
                q,
                omega,
                pos,
                vel,
                q,
                omega,
                zero,
                commanded,
                commanded
            };
        }

        struct CsvRow
        {
            HilTelemetryPoint point{};
            HilHealthSnapshot health{};

            double startLatenessMs = -1.0;
            double completionOverrunMs = -1.0;

            bool completionDeadlineMiss = false;
        };

        double nearestRank(
            std::span<const double> sorted,
            double fraction)
        {
            const auto position =
                static_cast<std::size_t>(
                    std::ceil(
                        fraction *
                        static_cast<double>(sorted.size())));

            return sorted[
                std::max<std::size_t>(
                    1, position) - 1];
        }
    }

    HilSchedulingStatistics summarizeScheduling(
        std::span<const double> startLatenessMs,
        std::uint64_t completionDeadlineMisses,
        double maxCompletionOverrunMs)
    {
        if (!std::isfinite(maxCompletionOverrunMs) ||
            maxCompletionOverrunMs < 0.0 ||
            completionDeadlineMisses >
            startLatenessMs.size())
        {
            throw std::invalid_argument(
                "Invalid scheduling summary");
        }

        for (const double value : startLatenessMs)
        {
            if (!std::isfinite(value) ||
                value < 0.0)
            {
                throw std::invalid_argument(
                    "Invalid scheduling lateness");
            }
        }

        HilSchedulingStatistics stats{};

        stats.samples =
            static_cast<std::uint64_t>(
                startLatenessMs.size());

        stats.completionDeadlineMisses =
            completionDeadlineMisses;

        stats.maxCompletionOverrunMs =
            maxCompletionOverrunMs;

        if (startLatenessMs.empty())
        {
            if (completionDeadlineMisses != 0 ||
                maxCompletionOverrunMs != 0.0)
            {
                throw std::invalid_argument(
                    "No scheduling measurements");
            }

            return stats;
        }

        std::vector<double> sorted(
            startLatenessMs.begin(),
            startLatenessMs.end());

        std::sort(
            sorted.begin(),
            sorted.end());

        const long double sum =
            std::accumulate(
                sorted.begin(),
                sorted.end(),
                0.0L);

        stats.meanStartLatenessMs =
            static_cast<double>(
                sum /
                static_cast<long double>(sorted.size()));

        stats.p50StartLatenessMs =
            nearestRank(sorted, 0.50);

        stats.p95StartLatenessMs =
            nearestRank(sorted, 0.95);

        stats.p99StartLatenessMs =
            nearestRank(sorted, 0.99);

        stats.maxStartLatenessMs =
            sorted.back();

        return stats;
    }

    HilMonitoredRunResult runHilMonitoredLoopback(
        HilTelemetryStore& telemetry,
        visualization::LiveTelemetryPublisher* visualPublisher,
        std::stop_token stop,
        const HilMonitoredRunConfig& config)
    {
        if (!std::isfinite(config.stepS) ||
            config.stepS <= 0.0 ||
            config.stepS > 1.0 ||
            config.steps == 0)
        {
            throw std::invalid_argument(
                "Invalid HIL run configuration");
        }

        const auto period =
            std::chrono::duration_cast<
            Clock::duration>(
                std::chrono::duration<double>(
                    config.stepS));

        if (period <= Clock::duration::zero())
        {
            throw std::invalid_argument(
                "HIL period too small");
        }

        // Create the precise pacer only for
        // real-time runs.
        //
        // Unpaced benchmarks avoid timer setup.
        std::optional<HilPrecisePacer> pacer;

        if (config.realtimePacing)
        {
            pacer.emplace();
        }

        std::ofstream csv;
        std::vector<CsvRow> csvRows;

        if (!config.csvPath.empty())
        {
            csv.open(
                config.csvPath,
                std::ios::out | std::ios::trunc);

            if (!csv)
            {
                throw std::runtime_error(
                    "Cannot open HIL benchmark CSV");
            }

            csvRows.reserve(config.steps);
        }

        std::vector<double> startLateness;

        if (config.realtimePacing)
        {
            startLateness.reserve(config.steps);
        }

        std::uint64_t completionDeadlineMisses = 0;
        double maxCompletionOverrunMs = 0.0;

        telemetry.reset();

        Duplex duplex;

        HilWireEndpoint host(
            duplex.hostPort());

        HilWireEndpoint peer(
            duplex.controllerPort());

        HilBridgeConfig bridgeConfig{};

        bridgeConfig.wallClockCommandTimeout =
            std::chrono::milliseconds(100);

        HilSimulationBridge bridge(
            host,
            bridgeConfig);

        HilTimingMonitor monitor(period);

        bridge.attachTimingMonitor(&monitor);

        HilSoftwareController controller(
            peer,
            HilRateControllerConfig{});

        // ------------------------------------------
        // 6DOF simulation setup
        // ------------------------------------------

        dynamics::ForceAccumulator forces;
        dynamics::MomentAccumulator moments;

        const dynamics::MassProperties mass(10.0);

        const math::Matrix inertiaMatrix(
            3, 3,
            {
                2.0, 0.0, 0.0,
                0.0, 3.0, 0.0,
                0.0, 0.0, 4.0
            });

        const dynamics::InertiaProperties inertia(
            inertiaMatrix);

        const dynamics::SixDoFDynamicsModel model(
            forces,
            moments,
            mass,
            inertia);

        const simulation::RK4Integrator integrator;

        simulation::State state{
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            1.0, 0.0, 0.0, 0.0,
            0.3, -0.2, 0.4
        };

        HilMonitoredRunResult result{};

        const auto wallStart = Clock::now();
        const double periodMs =
            config.stepS * 1000.0;

        telemetry.publishHealth(
            makeHealth(
                bridge,
                monitor,
                periodMs,
                true));

        // ------------------------------------------
        // Main simulation loop
        // ------------------------------------------

        for (std::size_t index = 0;
            index < config.steps;
            ++index)
        {
            if (stop.stop_requested())
            {
                result.cancelled = true;
                break;
            }

            // Absolute scheduling:
            //
            // Each start is calculated from the
            // original wall-clock epoch.
            //
            // This prevents cumulative sleep drift.
            const auto scheduledStart =
                wallStart +
                period *
                static_cast<Clock::duration::rep>(
                    index);

            // Use precise, cancellation-aware pacing.
            if (config.realtimePacing)
            {
                if (!pacer->waitUntil(
                    scheduledStart,
                    stop))
                {
                    result.cancelled = true;
                    break;
                }
            }

            if (stop.stop_requested())
            {
                result.cancelled = true;
                break;
            }

            const auto actualStart = Clock::now();

            const double timeS =
                static_cast<double>(index) *
                config.stepS;

            std::uint64_t timeUs = 0;

            if (!HilSimulationBridge::
                secondsToMicroseconds(
                    timeS,
                    timeUs))
            {
                throw std::overflow_error(
                    "Simulation time exceeds "
                    "HIL timestamp range");
            }

            const auto beforeCommunication =
                monitor.statistics().communication;

            const auto beforeLatency =
                monitor.statistics().commandLatency;

            // --------------------------------------
            // Controller and dynamics execution
            // --------------------------------------

            controller.pump();
            bridge.pump(timeUs);

            const math::Vector3 commanded =
                bridge.currentCommandedMomentB(
                    timeUs);

            moments.clear();
            moments.addMoment(commanded);

            state = integrator.step(
                model,
                timeS,
                state,
                config.stepS);

            dynamics::normalizeRigidBodyQuaternion(
                state);

            if (!state.isFinite())
            {
                throw std::runtime_error(
                    "Nonfinite HIL 6DOF state");
            }

            // --------------------------------------
            // Sensor publication and response
            // --------------------------------------

            const double nextS =
                static_cast<double>(index + 1) *
                config.stepS;

            const sensors::ImuMeasurement idealImu{
                nextS,
                math::Vector3{},
                math::Vector3(
                    state[10],
                    state[11],
                    state[12])
            };

            bridge.publishSensorSample(idealImu);

            std::uint64_t nextUs = 0;

            if (!HilSimulationBridge::
                secondsToMicroseconds(
                    nextS,
                    nextUs))
            {
                throw std::overflow_error(
                    "Simulation time exceeds "
                    "HIL timestamp range");
            }

            bridge.pump(nextUs);
            controller.pump();
            bridge.pump(nextUs);

            // --------------------------------------
            // Computational execution timing
            // --------------------------------------

            const auto computeEnd = Clock::now();

            monitor.recordSimulationStep(
                computeEnd - actualStart);

            const auto& timing =
                monitor.statistics();

            HilTelemetryPoint point{};

            point.simulationTimeS = nextS;

            point.stepExecutionMs =
                milliseconds(
                    computeEnd - actualStart);

            if (timing.communication.count >
                beforeCommunication.count)
            {
                point.communicationMs =
                    std::max(
                        0.0,
                        timing.communication.averageMs *
                        timing.communication.count -
                        beforeCommunication.averageMs *
                        beforeCommunication.count);
            }

            point.commandLatencyMs =
                newestMs(
                    beforeLatency,
                    timing.commandLatency);

            const HilHealthSnapshot health =
                makeHealth(
                    bridge,
                    monitor,
                    periodMs,
                    true);

            telemetry.publish(point, health);

            if (visualPublisher != nullptr)
            {
                visualPublisher->publish(
                    visualSample(
                        nextS,
                        state,
                        commanded));
            }

            // --------------------------------------
            // Scheduling deadline measurements
            // --------------------------------------

            // Completion includes simulation work,
            // communication, and telemetry publishing.
            // CSV output is deferred until after the run.
            const auto actualCompletion =
                Clock::now();

            CsvRow row{};
            row.point = point;
            row.health = health;

            if (config.realtimePacing)
            {
                row.startLatenessMs =
                    std::max(
                        0.0,
                        milliseconds(
                            actualStart -
                            scheduledStart));

                const auto scheduledDeadline =
                    scheduledStart + period;

                row.completionOverrunMs =
                    std::max(
                        0.0,
                        milliseconds(
                            actualCompletion -
                            scheduledDeadline));

                row.completionDeadlineMiss =
                    actualCompletion >
                    scheduledDeadline;

                startLateness.push_back(
                    row.startLatenessMs);

                if (row.completionDeadlineMiss)
                {
                    ++completionDeadlineMisses;
                }

                maxCompletionOverrunMs =
                    std::max(
                        maxCompletionOverrunMs,
                        row.completionOverrunMs);
            }

            // Only collect CSV rows when a file
            // was actually opened.
            if (csv.is_open())
            {
                csvRows.push_back(row);
            }

            result.completedSteps = index + 1;
        }

        // Account for the last scheduled interval
        // in the total paced wall-clock duration.
        //
        // This wait is not part of a step's
        // completion deadline measurement.
        if (config.realtimePacing &&
            !result.cancelled)
        {
            const auto finalDeadline =
                wallStart +
                period *
                static_cast<Clock::duration::rep>(
                    result.completedSteps);

            if (!pacer->waitUntil(
                finalDeadline,
                stop))
            {
                result.cancelled = true;
            }
        }

        const auto wallEnd = Clock::now();

        result.wallElapsedS =
            std::chrono::duration<double>(
                wallEnd - wallStart).count();

        if (config.realtimePacing)
        {
            result.scheduling =
                summarizeScheduling(
                    startLateness,
                    completionDeadlineMisses,
                    maxCompletionOverrunMs);
        }

        // Disarm the controller on shutdown.
        bridge.notifyTransportDisconnected();

        result.finalHealth =
            makeHealth(
                bridge,
                monitor,
                periodMs,
                false);

        telemetry.publishHealth(
            result.finalHealth);

        // ------------------------------------------
        // Deferred CSV reporting
        // ------------------------------------------

        if (csv.is_open())
        {
            csv <<
                "simulation_time_s,"
                "step_execution_ms,"
                "communication_ms,"
                "new_command_latency_ms,"
                "queued_samples,"
                "dropped_samples,"
                "accepted_commands,"
                "rejected_commands,"
                "execution_deadline_misses,"
                "watchdog_state,"
                "start_lateness_ms,"
                "completion_overrun_ms,"
                "completion_deadline_miss\n";

            csv << std::setprecision(12);

            for (const auto& row : csvRows)
            {
                csv
                    << row.point.simulationTimeS << ','
                    << row.point.stepExecutionMs << ','
                    << row.point.communicationMs << ','
                    << row.point.commandLatencyMs << ','
                    << row.health.queuedSamples << ','
                    << row.health.droppedSamples << ','
                    << row.health.acceptedCommands << ','
                    << row.health.rejectedCommands << ','
                    << row.health.deadlineMisses << ','
                    << static_cast<int>(
                        row.health.watchdogState)
                    << ',';

                if (config.realtimePacing)
                {
                    csv
                        << row.startLatenessMs << ','
                        << row.completionOverrunMs << ','
                        << (row.completionDeadlineMiss
                            ? 1 : 0);
                }
                else
                {
                    csv << ",,";
                }

                csv << '\n';
            }

            csv.close();

            if (!csv)
            {
                throw std::runtime_error(
                    "Cannot write HIL benchmark CSV");
            }
        }

        return result;
    }
}