#include <dynamics/TranslationalState.h>

#include <stdexcept>

namespace aeroforge::dynamics {

	TranslationalState::TranslationalState(aeroforge::math::Vector3 position,
		aeroforge::math::Vector3 velocity)
		: positionN(position), velocityN(velocity)
	{
		if (!positionN.isFinite() || !velocityN.isFinite())
		{
			throw std::invalid_argument("Position and velocity must be finite values.");
		}
	}

	TranslationalState::TranslationalState(const aeroforge::simulation::State& stateVector)
	{
		if (stateVector.size() != STATE_SIZE)
		{
			throw std::invalid_argument("State vector must have a size of 6.");
		}
		positionN.x = stateVector[0];
		positionN.y = stateVector[1];
		positionN.z = stateVector[2];
		velocityN.x = stateVector[3];
		velocityN.y = stateVector[4];
		velocityN.z = stateVector[5];
		if (!positionN.isFinite() || !velocityN.isFinite())
		{
			throw std::invalid_argument("Position and velocity must be finite values.");
		}
	}

	const aeroforge::math::Vector3& TranslationalState::getPosition() const
	{
		return positionN;
	}

	const aeroforge::math::Vector3& TranslationalState::getVelocity() const
	{
		return velocityN;
	}

	aeroforge::simulation::State TranslationalState::toStateVector() const
	{
		aeroforge::simulation::State stateVector(STATE_SIZE);
		stateVector[0] = positionN.x;
		stateVector[1] = positionN.y;
		stateVector[2] = positionN.z;
		stateVector[3] = velocityN.x;
		stateVector[4] = velocityN.y;
		stateVector[5] = velocityN.z;
		return stateVector;
	}
} // namespace aeroforge::dynamics