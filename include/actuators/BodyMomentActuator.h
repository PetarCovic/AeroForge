#pragma once

#include <math/Vector3.h>

namespace aeroforge::actuators
{
	class BodyMomentActuator
	{
	public:
		BodyMomentActuator(
			const math::Vector3& maxAbsMomentB,
			const math::Vector3& maxRateB
		);

		math::Vector3 update(
			const math::Vector3& commandedMomentB,
			double dtS
		);

		const math::Vector3& getActualMomentB() const;

	private:
		math::Vector3 maxAbsMomentB;
		math::Vector3 maxRateB;
		math::Vector3 actualMomentB;
	};
}