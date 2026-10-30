#include <gtest/gtest.h>

#include <hil/HilClosedLoopStep.h>
#include <hil/HilSimulationBridge.h>
#include <hil/HilSoftwareController.h>
#include <hil/HilWireEndpoint.h>

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
#include <math/Quaternion.h>
#include <math/Vector3.h>

#include "HilTestByteLink.h"

#include <cmath>
#include <cstdint>
#include <stdexcept>

using namespace aeroforge;
using namespace aeroforge::hil;
using namespace aeroforge::hil::tests;

namespace
{
    class HilClosedLoopIntegrationTests :
        public ::testing::Test
    {
    protected:
        HilTestByteLink link;

        HilWireEndpoint hostEndpoint{ link.portA() };
        HilWireEndpoint controllerEndpoint{ link.portB() };

        HilBridgeConfig bridgeConfig{};
        HilSimulationBridge bridge{
            hostEndpoint, bridgeConfig
        };

        HilRateControllerConfig controllerConfig{};
        HilSoftwareController controller{
            controllerEndpoint, controllerConfig
        };

        dynamics::ForceAccumulator forces;
        dynamics::MomentAccumulator moments;

        dynamics::MassProperties mass{ 10.0 };

        dynamics::InertiaProperties inertia{
            math::Matrix(
                3, 3,
                {
                    2.0, 0.0, 0.0,
                    0.0, 3.0, 0.0,
                    0.0, 0.0, 4.0
                })
        };

        dynamics::SixDoFDynamicsModel model{
            forces, moments, mass, inertia
        };

        simulation::RK4Integrator integrator;

        sensors::AccelerometerModel accelerometer{
            math::Vector3(),
            math::Vector3(),
            math::Vector3(1000, 1000, 1000),
            12345u
        };

        sensors::GyroscopeModel gyroscope{
            math::Vector3(),
            math::Vector3(),
            math::Vector3(1000, 1000, 1000),
            67890u
        };

        sensors::ImuModel imu{
            accelerometer, gyroscope
        };

        sensors::SensorSchedule schedule{
            100.0, 0.01
        };

        actuators::BodyMomentActuator actuator{
            math::Vector3(20, 20, 20),
            math::Vector3(200, 200, 200)
        };

        simulation::State state =
            dynamics::RigidBodyState(
                math::Vector3(),
                math::Vector3(),
                math::Quaternion(1.0, 0.0, 0.0, 0.0),
                math::Vector3(0.0, 0.0, 0.4)
            ).toStateVector();

        HilClosedLoopStep hilStep{
            bridge,
            imu,
            schedule,
            actuator,
            moments,
            model,
            integrator,
            state,
            [](double, const simulation::State&)
            {
                // Zero external force and no gravity.
                // This isolated test therefore uses zero
                // true specific force.
                return math::Vector3();
            }
        };

        static constexpr double dtS = 0.01;

        void advanceWithController(int steps)
        {
            for (int i = 0; i < steps; ++i)
            {
                const double timeS = i * dtS;

                hilStep.step(timeS, dtS);

                // Independent controller processes the
                // sensor measurement from the simulation.
                controller.pump();

                // Deliver its response before the next step.
                const std::uint64_t nowUs =
                    static_cast<std::uint64_t>(i + 1) * 10000u;

                bridge.pump(nowUs);
            }
        }

        void advanceWithoutController(int steps)
        {
            for (int i = 0; i < steps; ++i)
            {
                hilStep.step(i * dtS, dtS);
            }
        }
    };
}

TEST_F(
    HilClosedLoopIntegrationTests,
    RateDampingReducesAngularVelocity)
{
    const dynamics::RigidBodyState initial(state);

    const double initialRate =
        initial.getAngularVelocityB().magnitude();

    advanceWithController(500);

    const dynamics::RigidBodyState finalState(state);

    const double finalRate =
        finalState.getAngularVelocityB().magnitude();

    EXPECT_LT(finalRate, initialRate);
    EXPECT_LT(finalRate, 0.05);

    EXPECT_GT(bridge.stats().acceptedCommands, 0u);
    EXPECT_GT(controller.stats().generatedCommands, 0u);

    EXPECT_TRUE(state.isFinite());
}

TEST_F(
    HilClosedLoopIntegrationTests,
    MissingControllerPreservesTorqueFreeBodyRate)
{
    const dynamics::RigidBodyState initial(state);

    const double initialZ =
        initial.getAngularVelocityB().z;

    advanceWithoutController(100);

    const dynamics::RigidBodyState finalState(state);

    // A principal-axis rotation with zero external
    // moments should retain angular velocity.
    EXPECT_NEAR(
        finalState.getAngularVelocityB().z,
        initialZ,
        1e-8);

    EXPECT_EQ(bridge.stats().acceptedCommands, 0u);
    EXPECT_TRUE(state.isFinite());
}

TEST_F(
    HilClosedLoopIntegrationTests,
    ControllerCommandAffectsSubsequentStep)
{
    const dynamics::RigidBodyState initial(state);
    const double initialZ =
        initial.getAngularVelocityB().z;

    // First step: no controller command exists yet.
    hilStep.step(0.0, dtS);

    const dynamics::RigidBodyState afterFirst(state);

    EXPECT_NEAR(
        afterFirst.getAngularVelocityB().z,
        initialZ,
        1e-8);

    // Controller responds to the first sensor sample.
    controller.pump();
    bridge.pump(10000);

    // Second step: the accepted command is now available.
    hilStep.step(dtS, dtS);

    const dynamics::RigidBodyState afterSecond(state);

    EXPECT_LT(
        afterSecond.getAngularVelocityB().z,
        afterFirst.getAngularVelocityB().z);
}

TEST_F(
    HilClosedLoopIntegrationTests,
    RepeatedSimulationStepsRemainFinite)
{
    advanceWithController(1000);

    EXPECT_TRUE(state.isFinite());

    const dynamics::RigidBodyState finalState(state);

    EXPECT_TRUE(
        finalState.getAngularVelocityB().isFinite());

    EXPECT_TRUE(
        finalState.getPositionN().isFinite());

    EXPECT_TRUE(
        finalState.getVelocityN().isFinite());
}

TEST_F(
    HilClosedLoopIntegrationTests,
    RejectsInvalidSimulationTimeStep)
{
    EXPECT_THROW(
        hilStep.step(0.0, 0.0),
        std::invalid_argument);

    EXPECT_THROW(
        hilStep.step(-1.0, dtS),
        std::invalid_argument);
}