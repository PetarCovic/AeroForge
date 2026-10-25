#include <rocket/RocketEngine.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::rocket
{
	RocketEngine::RocketEngine(
		const ThrustCurve& thrustCurve,
		double specificImpulseS,
		const math::Vector3& enginePositionB,
		const math::Vector3& thrustDirectionB
	)
		: thrustCurve(thrustCurve), specificImpulseS(specificImpulseS), enginePositionB(enginePositionB), thrustDirectionB(thrustDirectionB)
	{
		if (!std::isfinite(specificImpulseS) || specificImpulseS <= 0)
		{
			throw std::invalid_argument("specificImpulseS must be finite and positive");
		}

		if(!enginePositionB.isFinite())
		{
			throw std::invalid_argument("enginePositionB must be finite");
		}

		if (!thrustDirectionB.isFinite() || thrustDirectionB.isZero())
		{
			throw std::invalid_argument("thrustDirectionB must be finite and non-zero");
		}

		this->thrustDirectionB = this->thrustDirectionB.normalize();
	}

	double RocketEngine::getSpecificImpulseS() const
	{
		return specificImpulseS;
	}

	const math::Vector3& RocketEngine::getEnginePositionB() const
	{
		return enginePositionB;
	}

	const math::Vector3& RocketEngine::getThrustDirectionB() const
	{
		return thrustDirectionB;
	}

	double RocketEngine::getThrustN(
		double timeS,
		double throttle
	) const
	{
		if (!std::isfinite(throttle) || throttle > 1 || throttle < 0)
		{
			throw std::invalid_argument("throttle must be finite and in [0, 1]");
		}

		return thrustCurve.getThrustN(timeS) * throttle;
	}

	math::Vector3 RocketEngine::computeThrustForceB(
		double timeS,
		double throttle
	) const
	{
		return thrustDirectionB * getThrustN(timeS, throttle);
	}

	double RocketEngine::computeMassFlowRateKgPerS(
		double timeS,
		double throttle
	) const
	{
		return getThrustN(timeS, throttle) / (specificImpulseS * 9.80665);
	}
}