
#include <hil/HilWireEndpoint.h>
#include <hil/HilSimulationBridge.h>
#include <hil/HilClosedLoopStep.h>
#include <hil/HilSoftwareController.h>

#include <actuators/BodyMomentActuator.h>
#include <dynamics/ForceAccumulator.h>
#include <dynamics/MomentAccumulator.h>
#include <dynamics/MassProperties.h>
#include <dynamics/InertiaProperties.h>
#include <dynamics/SixDoFDynamicsModel.h>
#include <dynamics/RigidBodyState.h>

#include <sensors/AccelerometerModel.h>
#include <sensors/GyroscopeModel.h>
#include <sensors/ImuModel.h>
#include <sensors/SensorSchedule.h>

#include <simulation/RK4Integrator.h>
#include <simulation/State.h>

#include <math/Matrix.h>
#include <math/Vector3.h>
#include <math/Quaternion.h>

#include <algorithm>
#include <cstdint>
#include <deque>
#include <iostream>
#include <span>

int main()
{
    using namespace aeroforge;

    // ----------------------------------------------------------
    // 1. Independent bidirectional byte channels
    // ----------------------------------------------------------

    std::deque<std::uint8_t> pcToController;
    std::deque<std::uint8_t> controllerToPc;

    auto readBytes = [](
        std::deque<std::uint8_t>& queue,
        std::span<std::uint8_t> output) -> std::size_t
        {
            const std::size_t count =
                std::min(queue.size(), output.size());

            for (std::size_t i = 0; i < count; ++i)
            {
                output[i] = queue.front();
                queue.pop_front();
            }

            return count;
        };

    auto writeBytes = [](
        std::deque<std::uint8_t>& queue,
        std::span<const std::uint8_t> input) -> std::size_t
        {
            for (std::uint8_t byte : input)
            {
                queue.push_back(byte);
            }

            return input.size();
        };

    hil::HilBytePort pcPort{
        [&](std::span<std::uint8_t> output)
        {
            return readBytes(controllerToPc, output);
        },
        [&](std::span<const std::uint8_t> input)
        {
            return writeBytes(pcToController, input);
        }
    };

    hil::HilBytePort controllerPort{
        [&](std::span<std::uint8_t> output)
        {
            return readBytes(pcToController, output);
        },
        [&](std::span<const std::uint8_t> input)
        {
            return writeBytes(controllerToPc, input);
        }
    };

    hil::HilWireEndpoint pcEndpoint(pcPort);
    hil::HilWireEndpoint controllerEndpoint(controllerPort);

    // ----------------------------------------------------------
    // 2. Simulation-side HIL bridge
    // ----------------------------------------------------------

    hil::HilBridgeConfig bridgeConfig{};
    bridgeConfig.maximumCommandAgeUs = 100000;
    bridgeConfig.maximumAbsMomentB =
        math::Vector3(20.0, 20.0, 20.0);

    hil::HilSimulationBridge bridge(
        pcEndpoint,
        bridgeConfig);

    // ----------------------------------------------------------
    // 3. Independent software controller
    // ----------------------------------------------------------

    hil::HilRateControllerConfig controllerConfig{};

    controllerConfig.rateGain =
        math::Vector3(5.0, 5.0, 5.0);

    controllerConfig.maximumAbsMomentB =
        math::Vector3(20.0, 20.0, 20.0);

    hil::HilSoftwareController controller(
        controllerEndpoint,
        controllerConfig);

    // ----------------------------------------------------------
    // 4. Six-degree-of-freedom dynamics
    // ----------------------------------------------------------

    dynamics::ForceAccumulator forces;
    dynamics::MomentAccumulator moments;

    const dynamics::MassProperties mass(10.0);

    const dynamics::InertiaProperties inertia(
        math::Matrix(
            3, 3,
            {
                2.0, 0.0, 0.0,
                0.0, 3.0, 0.0,
                0.0, 0.0, 4.0
            }
        )
    );

    const dynamics::SixDoFDynamicsModel model(
        forces,
        moments,
        mass,
        inertia);

    const simulation::RK4Integrator integrator;

    // Initial nonzero body rates.
    simulation::State state =
        dynamics::RigidBodyState(
            math::Vector3(),
            math::Vector3(),
            math::Quaternion(1.0, 0.0, 0.0, 0.0),
            math::Vector3(0.3, -0.2, 0.4)
        ).toStateVector();

    // ----------------------------------------------------------
    // 5. Sensor models
    // ----------------------------------------------------------

    const math::Vector3 zero(0.0, 0.0, 0.0);
    const math::Vector3 largeSensorLimit(
        1000.0, 1000.0, 1000.0);

    sensors::AccelerometerModel accelerometer(
        zero,
        zero,
        largeSensorLimit,
        12345u);

    sensors::GyroscopeModel gyroscope(
        zero,
        zero,
        largeSensorLimit,
        67890u);

    sensors::ImuModel imu(
        accelerometer,
        gyroscope);

    sensors::SensorSchedule sensorSchedule(
        100.0,
        0.01);

    // ----------------------------------------------------------
    // 6. Body moment actuator
    // ----------------------------------------------------------

    actuators::BodyMomentActuator actuator(
        math::Vector3(20.0, 20.0, 20.0),
        math::Vector3(200.0, 200.0, 200.0));

    // ----------------------------------------------------------
    // 7. HIL-aware integration step
    // ----------------------------------------------------------

    hil::HilClosedLoopStep hilStep(
        bridge,
        imu,
        sensorSchedule,
        actuator,
        moments,
        model,
        integrator,
        state,
        [](double, const simulation::State&)
        {
            // No external forces or gravity in this scenario,
            // therefore true specific force is zero.
            return math::Vector3();
        }
    );

    // ----------------------------------------------------------
    // 8. Simulate the closed-loop exchange
    // ----------------------------------------------------------

    constexpr double dtS = 0.01;
    constexpr int totalSteps = 1000;

    for (int i = 0; i < totalSteps; ++i)
    {
        const double timeS = i * dtS;

        // Simulation runs using commands accepted earlier,
        // then publishes a new IMU sample.
        hilStep.step(timeS, dtS);

        // Independent controller processes the sensor
        // packet and sends back a control command.
        controller.pump();

        // Simulation bridge receives that command.
        // It will affect a subsequent integration step.
        std::uint64_t nowUs = 0;

        if (!hil::HilSimulationBridge::secondsToMicroseconds(
            timeS + dtS, nowUs))
        {
            return 1;
        }

        bridge.pump(nowUs);
    }

    // ----------------------------------------------------------
    // 9. Report results
    // ----------------------------------------------------------

    const dynamics::RigidBodyState finalTruth(state);
    const math::Vector3 omega =
        finalTruth.getAngularVelocityB();

    std::cout
        << "Final angular velocity B: "
        << omega.x << ", "
        << omega.y << ", "
        << omega.z << " rad/s\n";

    std::cout
        << "Sensor packets queued: "
        << bridge.stats().queuedSamples << "\n";

    std::cout
        << "Accepted control commands: "
        << bridge.stats().acceptedCommands << "\n";

    std::cout
        << "Controller commands generated: "
        << controller.stats().generatedCommands << "\n";

    std::cout
        << "Invalid packets at PC: "
        << pcEndpoint.invalidPacketCount() << "\n";

    std::cout
        << "Invalid packets at controller: "
        << controllerEndpoint.invalidPacketCount() << "\n";

    return 0;
}
