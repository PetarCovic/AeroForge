#pragma once

#include <hil/HilSimulationBridge.h>

#include <actuators/BodyMomentActuator.h>
#include <dynamics/MomentAccumulator.h>
#include <math/Vector3.h>

#include <sensors/ImuModel.h>
#include <sensors/SensorSchedule.h>

#include <simulation/DynamicsModel.h>
#include <simulation/Integrator.h>
#include <simulation/State.h>

#include <functional>

namespace aeroforge::hil
{
    using SpecificForceProvider = std::function<
        math::Vector3(
            double timeS,
            const simulation::State& state)
    >;

    class HilClosedLoopStep
    {
    public:
        HilClosedLoopStep(
            HilSimulationBridge& bridge,
            sensors::ImuModel& imu,
            sensors::SensorSchedule& sensorSchedule,
            actuators::BodyMomentActuator& actuator,
            dynamics::MomentAccumulator& moments,
            const simulation::DynamicsModel& model,
            const simulation::Integrator& integrator,
            simulation::State& state,
            SpecificForceProvider specificForceProvider
        );

        void step(double timeS, double dtS);

    private:
        HilSimulationBridge& bridge_;
        sensors::ImuModel& imu_;
        sensors::SensorSchedule& sensorSchedule_;

        actuators::BodyMomentActuator& actuator_;
        dynamics::MomentAccumulator& moments_;

        const simulation::DynamicsModel& model_;
        const simulation::Integrator& integrator_;

        simulation::State& state_;

        SpecificForceProvider specificForceProvider_;
    };
}