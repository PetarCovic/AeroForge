#include <hil/HilClosedLoopStep.h>

#include <dynamics/RigidBodyState.h>
#include <dynamics/RigidBodyQuaternionNormalization.h>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace aeroforge::hil
{
    HilClosedLoopStep::HilClosedLoopStep(
        HilSimulationBridge& bridge,
        sensors::ImuModel& imu,
        sensors::SensorSchedule& sensorSchedule,
        actuators::BodyMomentActuator& actuator,
        dynamics::MomentAccumulator& moments,
        const simulation::DynamicsModel& model,
        const simulation::Integrator& integrator,
        simulation::State& state,
        SpecificForceProvider specificForceProvider)
        : bridge_(bridge),
        imu_(imu),
        sensorSchedule_(sensorSchedule),
        actuator_(actuator),
        moments_(moments),
        model_(model),
        integrator_(integrator),
        state_(state),
        specificForceProvider_(
            std::move(specificForceProvider))
    {
        if (!specificForceProvider_)
        {
            throw std::invalid_argument(
                "HIL requires a specific-force provider");
        }
    }

    void HilClosedLoopStep::step(
        double timeS,
        double dtS)
    {
        if (!std::isfinite(timeS) ||
            !std::isfinite(dtS) ||
            timeS < 0.0 ||
            dtS <= 0.0 ||
            !std::isfinite(timeS + dtS))
        {
            throw std::invalid_argument(
                "Invalid HIL simulation time or step");
        }

        std::uint64_t currentTimeUs = 0;

        if (!HilSimulationBridge::secondsToMicroseconds(
            timeS, currentTimeUs))
        {
            throw std::overflow_error(
                "HIL simulation timestamp overflow");
        }

        // 1. Receive and validate any available commands.
        bridge_.pump(currentTimeUs);

        // 2. Get the current valid command or fallback.
        const math::Vector3 command =
            bridge_.currentCommandedMomentB(
                currentTimeUs);

        // 3. Apply actuator dynamics and limits.
        const math::Vector3 appliedMoment =
            actuator_.update(command, dtS);

        // This setup assumes that HIL owns the only body
        // moment contribution for this scenario.
        //
        // If your scenario also has aerodynamic or propulsion
        // moments, combine those with appliedMoment instead.
        moments_.clear();
        moments_.addMoment(appliedMoment);

        // 4. Advance the 6DOF simulation.
        state_ = integrator_.step(
            model_, timeS, state_, dtS);

        dynamics::normalizeRigidBodyQuaternion(state_);

        // 5. Generate the scheduled sensor measurement.
        const double nextTimeS = timeS + dtS;

        if (sensorSchedule_.shouldSample(nextTimeS))
        {
            const dynamics::RigidBodyState truth(state_);

            const math::Vector3 trueSpecificForceB =
                specificForceProvider_(
                    nextTimeS, state_);

            const math::Vector3 trueAngularVelocityB =
                truth.getAngularVelocityB();

            const sensors::ImuMeasurement measurement =
                imu_.sample(
                    nextTimeS,
                    trueSpecificForceB,
                    trueAngularVelocityB);

            bridge_.publishSensorSample(measurement);
        }

        // 6. Flush outgoing sensor bytes without waiting
        // for the external controller to respond.
        std::uint64_t nextTimeUs = 0;

        if (!HilSimulationBridge::secondsToMicroseconds(
            nextTimeS, nextTimeUs))
        {
            throw std::overflow_error(
                "HIL simulation timestamp overflow");
        }

        bridge_.pump(nextTimeUs);
    }
}