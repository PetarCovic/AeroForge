#include <visualization/SixDoFLiveSimulation.h>
#include <visualization/LiveTelemetryPublisher.h>
#include <visualization/VisualizationSnapshot.h>

#include <dynamics/ConstantGravity.h>
#include <dynamics/ForceAccumulator.h>
#include <dynamics/InertiaProperties.h>
#include <dynamics/MassProperties.h>
#include <dynamics/MomentAccumulator.h>
#include <dynamics/SixDoFDynamicsModel.h>
#include <math/Matrix.h>
#include <math/Quaternion.h>
#include <math/Vector3.h>
#include <simulation/RK4Integrator.h>
#include <simulation/SimulationRunner.h>
#include <simulation/State.h>

#include <chrono>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <thread>

namespace aeroforge::visualization
{
    namespace
    {
        VisualizationSnapshot snapshotFromState(
            double timeS,
            const simulation::State& state,
            const math::Vector3& momentB)
        {
            // The state layout is [position_N(3), velocity_N(3),
            // q_NB(wxyz)(4), angularVelocity_B(3)].
            const math::Vector3 positionN(state[0], state[1], state[2]);
            const math::Vector3 velocityN(state[3], state[4], state[5]);
            const math::Quaternion attitudeNB =
                math::Quaternion(state[6], state[7], state[8], state[9]).normalized();
            const math::Vector3 angularVelocityB(state[10], state[11], state[12]);
            const math::Vector3 zero(0.0, 0.0, 0.0);

            // No sensor estimator or GNC controller runs in this demo:
            // estimated fields deliberately copy truth and are NOT estimates.
            return VisualizationSnapshot{
                timeS,
                positionN,
                velocityN,
                attitudeNB,
                angularVelocityB,
                positionN,
                velocityN,
                attitudeNB,
                angularVelocityB,
                zero,        // no computed attitude error
                zero,        // no commanded controller moment
                momentB      // applied constant body moment
            };
        }
    }

    SixDoFLiveRunResult runSixDoFLiveSimulation(
        LiveTelemetryPublisher& publisher,
        std::stop_token stop,
        double dtS,
        std::size_t steps,
        bool realtime)
    {
        if (!std::isfinite(dtS) || dtS <= 0.0)
            throw std::invalid_argument("6DOF live dtS must be finite and positive");

        constexpr double massKg = 12.0;
        const dynamics::MassProperties mass(massKg);

        dynamics::ForceAccumulator forces;
        forces.addForce(dynamics::ConstantGravity{}.computeForce(mass));
        forces.addForce(math::Vector3(9.0, 2.5, -210.0));

        const math::Vector3 appliedMomentB(0.035, 0.065, -0.030);
        dynamics::MomentAccumulator moments;
        moments.addMoment(appliedMomentB);

        const math::Matrix inertiaTensor(3, 3, {
            2.0, 0.0, 0.0,
            0.0, 3.0, 0.0,
            0.0, 0.0, 4.0
            });
        const dynamics::InertiaProperties inertia(inertiaTensor);
        const dynamics::SixDoFDynamicsModel model(forces, moments, mass, inertia);
        const simulation::RK4Integrator integrator;

        // Body +X is the rocket nose; +90 degrees around body Y maps it
        // to navigation Up (-Z_N) for an initially vertical vehicle.
        const math::Quaternion upright =
            math::Quaternion::fromRotationY(std::numbers::pi / 2.0);
        const simulation::State initial{
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            upright.w, upright.x, upright.y, upright.z,
            0.0, 0.0, 0.0
        };
        simulation::SimulationRunner runner(model, integrator, initial, dtS);

        SixDoFLiveRunResult result;
        using clock = std::chrono::steady_clock;
        const auto wallStart = clock::now();

        for (std::size_t step = 0; step <= steps; ++step)
        {
            if (stop.stop_requested())
            {
                result.cancelled = true;
                return result;
            }

            const double timeS = runner.getCurrentTime();
            publisher.publish(snapshotFromState(timeS, runner.getState(), appliedMomentB));
            ++result.publicationAttempts;
            result.lastSimulationTimeS = timeS;

            if (step == steps)
                break;

            if (realtime)
            {
                // Wall time paces the physics only; simulation time remains
                // authoritative, and no OpenGL/ImGui API is called here.
                const auto offset = std::chrono::duration_cast<clock::duration>(
                    std::chrono::duration<double>((step + 1) * dtS));
                std::this_thread::sleep_until(wallStart + offset);
            }
            if (stop.stop_requested())
            {
                result.cancelled = true;
                return result;
            }
            runner.step();
        }
        return result;
    }
}
