#pragma once

#include <simulation/DynamicsModel.h>
#include <dynamics/ForceAccumulator.h>
#include <dynamics/MomentAccumulator.h>
#include <rocket/PropellantTank.h>
#include <rocket/RocketMassProperties.h>
#include <rocket/RocketEngine.h>

namespace aeroforge::rocket
{
	class RocketDynamicsModel : public simulation::DynamicsModel
	{
    public:
        RocketDynamicsModel(
            const dynamics::ForceAccumulator& externalForceAccumulator,
            const dynamics::MomentAccumulator& externalMomentAccumulator,
            const PropellantTank& propellantTank,
            const RocketMassProperties& rocketMassProperties,
            const RocketEngine& rocketEngine);

        simulation::StateDerivative evaluate(
            double time,
            const simulation::State& state) const override;

    private:
        const dynamics::ForceAccumulator& externalForceAccumulator;
        const dynamics::MomentAccumulator& externalMomentAccumulator;
        const PropellantTank& propellantTank;

        RocketMassProperties rocketMassProperties;
        RocketEngine rocketEngine;
	};
}