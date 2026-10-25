#include <dynamics/TranslationalDynamicsModel.h>

#include <stdexcept>

namespace aeroforge::dynamics
{
	TranslationalDynamicsModel::TranslationalDynamicsModel(const ForceAccumulator& forceAccumulator, const MassProperties& massProperties)
		: forceAccumulator(forceAccumulator), massProperties(massProperties)
	{
	}

	simulation::StateDerivative TranslationalDynamicsModel::evaluate(double time, const simulation::State& state) const
	{
		(void)time; // Unused parameter

		TranslationalState translationalState(state);
		const math::Vector3 accelerationN = forceAccumulator.getTotalForce() / massProperties.getMassKg();

		if (!accelerationN.isFinite())
		{
			throw std::runtime_error("Acceleration is not finite. Check force and mass values.");
		}

		simulation::StateDerivative derivative(TranslationalState::STATE_SIZE);
		
		derivative[0] = translationalState.getVelocity().x;
		derivative[1] = translationalState.getVelocity().y;
		derivative[2] = translationalState.getVelocity().z;
		derivative[3] = accelerationN.x;
		derivative[4] = accelerationN.y;
		derivative[5] = accelerationN.z;

		return derivative;
	}
} // namespace aeroforge::dynamics