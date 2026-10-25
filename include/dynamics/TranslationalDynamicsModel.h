#pragma once

#include <simulation/DynamicsModel.h>
#include <dynamics/ForceAccumulator.h>
#include <dynamics/MassProperties.h>
#include <dynamics/TranslationalState.h>

namespace aeroforge::dynamics
{
	class TranslationalDynamicsModel : public simulation::DynamicsModel
	{
	public:
		TranslationalDynamicsModel(const ForceAccumulator& forceAccumulator, const MassProperties& massProperties);

		simulation::StateDerivative evaluate(double time, const simulation::State& state) const override;

	private:
		const ForceAccumulator& forceAccumulator;
		const MassProperties& massProperties;
	};
}