#include <hil/HilHardwareRun.h>

#include <hil/HilPrecisePacer.h>
#include <hil/HilSimulationBridge.h>
#include <hil/HilSerialWorker.h>
#include <hil/HilTimingMonitor.h>
#include <hil/HilWireEndpoint.h>
#include <hil/WindowsSerialTransport.h>

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
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

namespace aeroforge::hil
{
    namespace
    {
        using Clock = std::chrono::steady_clock;

        double ms(Clock::duration duration)
        {
            return std::chrono::duration<double, std::milli>(duration).count();
        }

        // Convert cumulative running averages into the sum of new observations.
        // A pump may accept more than one controller response at once.
        double newlyObservedLatencyMs(
            const HilDurationStatistics& previous,
            const HilDurationStatistics& current)
        {
            if (current.count <= previous.count) return -1.0;
            const auto n = static_cast<double>(current.count - previous.count);
            const double sum = current.averageMs * static_cast<double>(current.count)
                             - previous.averageMs * static_cast<double>(previous.count);
            return std::max(0.0, sum / n);
        }

        visualization::VisualizationSnapshot visualize(
            double timeS, const simulation::State& state,
            const math::Vector3& command)
        {
            const math::Vector3 pos(state[0], state[1], state[2]);
            const math::Vector3 vel(state[3], state[4], state[5]);
            const math::Quaternion q(state[6], state[7], state[8], state[9]);
            const math::Vector3 omega(state[10], state[11], state[12]);
            const math::Vector3 zero{};
            // This demonstration uses truth for estimation; no navigator is run.
            return {timeS, pos, vel, q, omega, pos, vel, q, omega,
                    zero, command, command};
        }

        HilHealthSnapshot makeHealth(
            const HilSimulationBridge& bridge,
            const HilTimingMonitor& timing,
            const HilSerialWorker& worker,
            double targetPeriodMs,
            bool running)
        {
            const auto& b = bridge.stats();
            const auto& t = timing.statistics();
            HilHealthSnapshot health{};
            health.mode = HilLinkMode::SERIAL;
            health.serialState = worker.state();
            health.serialRxDroppedBytes = worker.droppedReceiveBytes();
            health.serialLastError = worker.lastError();
            health.watchdogState = bridge.watchdogState();
            health.running = running;
            health.queuedSamples = b.queuedSamples;
            health.droppedSamples = b.droppedSamples;
            health.acceptedCommands = b.acceptedCommands;
            health.rejectedCommands = b.rejectedCommands;
            health.deadlineMisses = t.stepDeadlineMisses;
            health.stepCount = t.simulationSteps.count;
            health.commandLatencyCount = t.commandLatency.count;
            health.targetPeriodMs = targetPeriodMs;
            health.averageStepMs = t.simulationSteps.averageMs;
            health.maximumStepMs = t.simulationSteps.maximumMs;
            health.averageCommandLatencyMs = t.commandLatency.averageMs;
            health.maximumCommandLatencyMs = t.commandLatency.maximumMs;
            return health;
        }

        struct CsvRow
        {
            HilTelemetryPoint point{};
            HilHealthSnapshot health{};
            double startLatenessMs = 0.0;
            double completionOverrunMs = 0.0;
            bool completionMiss = false;
            bool usedValidCommand = false;
        };
    }

    void validateHilHardwareRunConfig(const HilHardwareRunConfig& config)
    {
        if (config.portName.empty() || config.portName.size() > 64 ||
            config.baudRate < 9600 || config.baudRate > 2000000 ||
            !std::isfinite(config.stepS) ||
            config.stepS <= 0.0 || config.stepS > 1.0 ||
            config.steps == 0 ||
            config.startupDelay.count() < 0 ||
            config.startupDelay > std::chrono::seconds(30) ||
            config.commandTimeout <= std::chrono::milliseconds(0) ||
            config.commandTimeout > std::chrono::seconds(10))
        {
            throw std::invalid_argument("Invalid HIL hardware configuration");
        }
    }

    HilHardwareRunResult runHilHardwareClosedLoop(
        HilTelemetryStore& telemetry,
        visualization::LiveTelemetryPublisher* visualPublisher,
        std::stop_token stop,
        const HilHardwareRunConfig& config)
    {
#ifndef _WIN32
        (void)telemetry;
        (void)visualPublisher;
        (void)stop;
        (void)config;
        throw std::runtime_error("Hardware COM-port runner requires Windows");
#else
        validateHilHardwareRunConfig(config);

        const auto period = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(config.stepS));
        if (period <= Clock::duration::zero())
            throw std::invalid_argument("Simulation step period too small");

        HilHardwareRunResult result{};
        telemetry.reset();
        if (stop.stop_requested())
        {
            result.cancelled = true;
            result.finalHealth.mode = HilLinkMode::SERIAL;
            telemetry.publishHealth(result.finalHealth);
            return result;
        }

        // CSV writes are deliberately deferred until after measured execution.
        std::vector<CsvRow> csvRows;
        if (!config.csvPath.empty())
        {
            // Preflight the destination before starting the physical session.
            std::ofstream preflight(config.csvPath, std::ios::trunc);
            if (!preflight) throw std::runtime_error("Cannot open hardware HIL CSV");
            preflight.close();
            csvRows.reserve(config.steps);
        }

        WindowsSerialConfig serialConfig{};
        serialConfig.portName = config.portName;
        serialConfig.baudRate = config.baudRate;
        serialConfig.writeTimeoutMs = 25;

        auto device = std::make_unique<WindowsSerialTransport>(serialConfig);
        HilSerialWorker worker(std::move(device));
        if (!worker.start())
            throw std::runtime_error("Could not start COM-port serial worker");

        HilWireEndpoint endpoint(worker.makePort());
        HilBridgeConfig bridgeConfig{};
        bridgeConfig.maximumCommandAgeUs = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                config.commandTimeout).count());
        bridgeConfig.wallClockCommandTimeout = config.commandTimeout;
        HilSimulationBridge bridge(endpoint, bridgeConfig);
        HilTimingMonitor timing(period);
        bridge.attachTimingMonitor(&timing);
        HilPrecisePacer pacer;

        // Opening a CH340 COM port may reset the ESP32. This grace period
        // occurs before the simulation epoch, not inside its timed steps.
        if (config.startupDelay.count() > 0 &&
            !pacer.waitUntil(Clock::now() + config.startupDelay, stop))
        {
            result.cancelled = true;
            bridge.notifyTransportDisconnected();
            result.finalHealth = makeHealth(bridge, timing, worker, config.stepS * 1000.0, false);
            worker.stop();
            result.finalHealth.serialState = HilSerialWorkerState::STOPPED;
            telemetry.publishHealth(result.finalHealth);
            return result;
        }

        // Existing validated 6DOF physics configuration.
        dynamics::ForceAccumulator forces;
        dynamics::MomentAccumulator moments;
        const dynamics::MassProperties mass(10.0);
        const math::Matrix inertiaMatrix(3, 3, {
            2.0, 0.0, 0.0,
            0.0, 3.0, 0.0,
            0.0, 0.0, 4.0
        });
        const dynamics::InertiaProperties inertia(inertiaMatrix);
        const dynamics::SixDoFDynamicsModel model(forces, moments, mass, inertia);
        const simulation::RK4Integrator integrator;
        simulation::State state{
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            1.0, 0.0, 0.0, 0.0,
            0.3, -0.2, 0.4
        };

        std::vector<double> startLateness;
        startLateness.reserve(config.steps);
        std::uint64_t completionMisses = 0;
        double maximumOverrunMs = 0.0;
        const auto wallStart = Clock::now();
        const double periodMs = config.stepS * 1000.0;

        telemetry.publishHealth(makeHealth(bridge, timing, worker, periodMs, true));

        for (std::size_t index = 0; index < config.steps; ++index)
        {
            if (stop.stop_requested())
            {
                result.cancelled = true;
                break;
            }

            const auto scheduledStart = wallStart +
                period * static_cast<Clock::duration::rep>(index);
            if (!pacer.waitUntil(scheduledStart, stop))
            {
                result.cancelled = true;
                break;
            }

            const auto actualStart = Clock::now();
            if (worker.state() == HilSerialWorkerState::FAILED ||
                worker.droppedReceiveBytes() != 0)
            {
                result.serialFailed = true;
                bridge.notifyTransportDisconnected();
                break;
            }

            const double timeS = static_cast<double>(index) * config.stepS;
            const double nextS = static_cast<double>(index + 1) * config.stepS;
            std::uint64_t timeUs = 0;
            std::uint64_t nextUs = 0;
            if (!HilSimulationBridge::secondsToMicroseconds(timeS, timeUs) ||
                !HilSimulationBridge::secondsToMicroseconds(nextS, nextUs))
                throw std::overflow_error("HIL simulation timestamp overflow");

            const auto beforeComm = timing.statistics().communication;
            const auto beforeLatency = timing.statistics().commandLatency;

            // Poll asynchronous serial responses from earlier sensor frames.
            // Never block a physics step waiting for USB turnaround.
            bridge.pump(timeUs);
            const auto command = bridge.currentCommandedMomentB(timeUs);
            const bool validCommand = bridge.watchdogState() ==
                HilWatchdogState::HEALTHY;
            const math::Vector3 applied = validCommand ? command : math::Vector3{};
            if (validCommand) ++result.stepsWithCommand;
            else ++result.stepsUsingZeroFallback;

            moments.clear();
            moments.addMoment(applied);
            state = integrator.step(model, timeS, state, config.stepS);
            dynamics::normalizeRigidBodyQuaternion(state);
            if (!state.isFinite())
                throw std::runtime_error("Nonfinite hardware HIL 6DOF state");

            const sensors::ImuMeasurement imu{
                nextS, math::Vector3{},
                math::Vector3(state[10], state[11], state[12])
            };
            bridge.publishSensorSample(imu);
            bridge.pump(nextUs); // nonblocking queue/flush/poll

            const auto computeEnd = Clock::now();
            timing.recordSimulationStep(computeEnd - actualStart);
            const auto& t = timing.statistics();

            HilTelemetryPoint point{};
            point.simulationTimeS = nextS;
            point.stepExecutionMs = ms(computeEnd - actualStart);
            if (t.communication.count > beforeComm.count)
            {
                const double sum =
                    t.communication.averageMs * static_cast<double>(t.communication.count) -
                    beforeComm.averageMs * static_cast<double>(beforeComm.count);
                point.communicationMs = std::max(0.0, sum);
            }
            point.commandLatencyMs =
                newlyObservedLatencyMs(beforeLatency, t.commandLatency);

            const auto health = makeHealth(bridge, timing, worker, periodMs, true);
            telemetry.publish(point, health);
            if (visualPublisher)
                visualPublisher->publish(visualize(nextS, state, applied));

            const auto complete = Clock::now();
            CsvRow row{};
            row.point = point;
            row.health = health;
            row.usedValidCommand = validCommand;
            row.startLatenessMs = std::max(0.0, ms(actualStart - scheduledStart));
            row.completionOverrunMs = std::max(0.0,
                ms(complete - (scheduledStart + period)));
            row.completionMiss = complete > scheduledStart + period;

            startLateness.push_back(row.startLatenessMs);
            if (row.completionMiss) ++completionMisses;
            maximumOverrunMs = std::max(maximumOverrunMs, row.completionOverrunMs);
            if (!config.csvPath.empty()) csvRows.push_back(row);
            ++result.completedSteps;
        }

        if (!result.cancelled && !result.serialFailed &&
            result.completedSteps == config.steps)
        {
            if (!pacer.waitUntil(wallStart + period *
                    static_cast<Clock::duration::rep>(config.steps), stop))
                result.cancelled = true;
        }

        result.wallElapsedS =
            std::chrono::duration<double>(Clock::now() - wallStart).count();
        result.scheduling = summarizeScheduling(
            startLateness, completionMisses, maximumOverrunMs);

        // Force zero command locally before shutting down serial transport.
        bridge.notifyTransportDisconnected();
        result.finalHealth = makeHealth(bridge, timing, worker, periodMs, false);
        worker.stop();
        if (!result.serialFailed)
            result.finalHealth.serialState = HilSerialWorkerState::STOPPED;
        telemetry.publishHealth(result.finalHealth);

        if (!config.csvPath.empty())
        {
            std::ofstream csv(config.csvPath, std::ios::trunc);
            if (!csv) throw std::runtime_error("Cannot write hardware HIL CSV");
            csv << "simulation_time_s,step_execution_ms,communication_ms,"
                   "new_command_latency_ms,accepted_commands,rejected_commands,"
                   "dropped_samples,serial_rx_dropped_bytes,watchdog_state,"
                   "serial_state,used_valid_command,start_lateness_ms,"
                   "completion_overrun_ms,completion_deadline_miss\n";
            csv << std::setprecision(12);
            for (const auto& row : csvRows)
            {
                csv << row.point.simulationTimeS << ','
                    << row.point.stepExecutionMs << ','
                    << row.point.communicationMs << ','
                    << row.point.commandLatencyMs << ','
                    << row.health.acceptedCommands << ','
                    << row.health.rejectedCommands << ','
                    << row.health.droppedSamples << ','
                    << row.health.serialRxDroppedBytes << ','
                    << static_cast<int>(row.health.watchdogState) << ','
                    << static_cast<int>(row.health.serialState) << ','
                    << (row.usedValidCommand ? 1 : 0) << ','
                    << row.startLatenessMs << ','
                    << row.completionOverrunMs << ','
                    << (row.completionMiss ? 1 : 0) << '\n';
            }
            csv.close();
            if (!csv) throw std::runtime_error("Hardware HIL CSV write failed");
        }

        return result;
#endif
    }
}