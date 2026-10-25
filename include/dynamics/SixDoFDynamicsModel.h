#pragma once

#include <dynamics/ForceAccumulator.h>
#include <dynamics/MomentAccumulator.h>
#include <dynamics/MassProperties.h>
#include <dynamics/InertiaProperties.h>
#include <simulation/State.h>
#include <simulation/StateDerivative.h>
#include <simulation/DynamicsModel.h>

namespace aeroforge::dynamics
{
	class SixDoFDynamicsModel : public simulation::DynamicsModel
	{
	public:
		SixDoFDynamicsModel(
			const ForceAccumulator& forceAccumulator,
			const MomentAccumulator& momentAccumulator,
			const MassProperties& massProperties,
			const InertiaProperties& inertiaProperties
		);

		simulation::StateDerivative evaluate(double time, const simulation::State& state) const override;

	private:
		const ForceAccumulator& forceAccumulator;
		const MomentAccumulator& momentAccumulator;
		const MassProperties& massProperties;
		const InertiaProperties& inertiaProperties;
	};
}