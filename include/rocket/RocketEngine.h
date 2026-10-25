#pragma once

#include <rocket/ThrustCurve.h>
#include <math/Vector3.h>

namespace aeroforge::rocket
{
	class RocketEngine
	{
	public:
		RocketEngine(
			const ThrustCurve& thrustCurve,
			double specificImpulseS,
			const math::Vector3& enginePositionB,
			const math::Vector3& thrustDirectionB
		);

		double getSpecificImpulseS() const;

		const math::Vector3& getEnginePositionB() const;

		const math::Vector3& getThrustDirectionB() const;

		double getThrustN(
			double timeS,
			double throttle
		) const;

		math::Vector3 computeThrustForceB(
			double timeS,
			double throttle
		) const;

		double computeMassFlowRateKgPerS(
			double timeS,
			double throttle
		) const;

	private:
		ThrustCurve thrustCurve;
		double specificImpulseS;
		math::Vector3 enginePositionB;
		math::Vector3 thrustDirectionB;
	};
}