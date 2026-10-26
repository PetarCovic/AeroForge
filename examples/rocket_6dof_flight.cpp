// Phase 17.10: a deterministic 6DOF rigid-body recording example.
// Simplified, fixed applied loads; NOT a RocketDynamicsModel engine/fuel simulation.
#include <dynamics/ConstantGravity.h>
#include <dynamics/ForceAccumulator.h>
#include <dynamics/MassProperties.h>
#include <dynamics/InertiaProperties.h>
#include <dynamics/MomentAccumulator.h>
#include <dynamics/SixDoFDynamicsModel.h>

#include <math/Matrix.h>
#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <simulation/RK4Integrator.h>
#include <simulation/SimulationRunner.h>
#include <telemetry/TelemetrySample.h>
#include <telemetry/CsvTelemetryWriter.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace
{
    using aeroforge::math::Vector3;
    using aeroforge::math::Quaternion;
    using aeroforge::simulation::State;
    using aeroforge::telemetry::TelemetrySample;

    // AeroForge 13-element 6DOF state:
    // [position_N(3), velocity_N(3), q_NB(wxyz)(4), omega_B(3)]
    TelemetrySample sampleAt(double time, const State& x, const Vector3& appliedMomentB)
    {
        const Vector3 positionN(x[0], x[1], x[2]);
        const Vector3 velocityN(x[3], x[4], x[5]);
        const Quaternion attitudeNB = Quaternion(x[6], x[7], x[8], x[9]).normalized();
        const Vector3 angularVelocityB(x[10], x[11], x[12]);

        // Every field is populated explicitly, preserving the 40-column writer contract.
        // No navigation estimator or GNC controller runs in this demonstration.
        TelemetrySample s{};
        s.timeS = time;
        s.positionN = positionN;
        s.velocityN = velocityN;
        s.attitudeNB = attitudeNB;
        s.angularVelocityB = angularVelocityB;
        s.estimatedPositionN = positionN;                 // truth copied, NOT an estimate
        s.estimatedVelocityN = velocityN;                 // truth copied, NOT an estimate
        s.estimatedAttitudeNB = attitudeNB;               // truth copied, NOT an estimate
        s.estimatedAngularVelocityB = angularVelocityB;   // truth copied, NOT an estimate
        s.attitudeErrorB = Vector3(0.0, 0.0, 0.0);       // no controller
        s.commandedMomentB = Vector3(0.0, 0.0, 0.0);     // no controller
        s.actualMomentB = appliedMomentB;                 // fixed disturbance input
        s.throttle = 0.0;                                 // no engine model
        s.gimbalPitchRad = 0.0;
        s.gimbalYawRad = 0.0;
        s.controllerSaturated = false;
        return s;
    }
}

int main(int argc, char* argv[])
{
    try
    {
        namespace fs = std::filesystem;
        using namespace aeroforge;

        constexpr double massKg = 12.0;
        constexpr double dt = 0.01;
        constexpr std::uint64_t steps = 1000; // 10 seconds + the initial sample

        const dynamics::MassProperties mass(massKg);
        dynamics::ForceAccumulator forces;
        forces.addForce(dynamics::ConstantGravity{}.computeForce(mass));

        // Fixed +N, +E, and upward (-D) navigation-frame load, used to
        // exercise the integrator; it is NOT rotating engine thrust.
        forces.addForce(Vector3(9.0, 2.5, -210.0));

        const Vector3 appliedMomentB(0.035, 0.065, -0.030);
        dynamics::MomentAccumulator moments;
        moments.addMoment(appliedMomentB);

        // Constant diagonal inertia tensor [kg m^2]; no propellant depletion.
        const math::Matrix inertiaTensor(3, 3, {
            2.0, 0.0, 0.0,
            0.0, 3.0, 0.0,
            0.0, 0.0, 4.0
            });
        const dynamics::InertiaProperties inertia(inertiaTensor);
        const dynamics::SixDoFDynamicsModel model(forces, moments, mass, inertia);
        const simulation::RK4Integrator integrator;

        // Body +X is the nose axis; pitch +90 deg maps it to NED Up (-Z_N).
        const Quaternion upright = Quaternion::fromRotationY(std::numbers::pi / 2.0);
        const State initialState{
            0.0, 0.0, 0.0,    // North, East, Down [m]
            0.0, 0.0, 0.0,    // North, East, Down velocity [m/s]
            upright.w, upright.x, upright.y, upright.z, // body-to-NED attitude
            0.0, 0.0, 0.0     // body angular velocity [rad/s]
        };
        simulation::SimulationRunner runner(model, integrator, initialState, dt);
        std::vector<TelemetrySample> samples;
        samples.reserve(static_cast<std::size_t>(steps) + 1);

        for (std::uint64_t step = 0; step <= steps; ++step)
        {
            samples.push_back(sampleAt(runner.getCurrentTime(), runner.getState(), appliedMomentB));
            if (step < steps)
            {
                runner.step();
            }
        }

        const fs::path output = argc > 1
            ? fs::path(argv[1])
            : fs::path("examples/output/rocket_6dof_flight.csv");
        if (output.has_parent_path())
        {
            fs::create_directories(output.parent_path());
        }
        std::ofstream file(output, std::ios::out | std::ios::trunc);
        if (!file)
        {
            throw std::runtime_error("Unable to create output CSV");
        }
        telemetry::CsvTelemetryWriter::write(file, samples);
        file.close();
        if (!file)
        {
            throw std::runtime_error("Failed to write output CSV");
        }

        const auto& last = samples.back();
        std::cout << "Wrote " << samples.size() << " real integrated 6DOF samples to "
            << fs::absolute(output).string() << '\n';
        std::cout << "Final time: " << last.timeS << " s; altitude: "
            << -last.positionN.z << " m; attitude quaternion: ("
            << last.attitudeNB.w << ", " << last.attitudeNB.x << ", "
            << last.attitudeNB.y << ", " << last.attitudeNB.z << ")\n";
        std::cout << "NOTE: this is a simplified constant-load rigid-body demo, "
            "not a propulsion/GNC rocket flight.\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "6DOF recording failed: " << e.what() << '\n';
        return 1;
    }
}
