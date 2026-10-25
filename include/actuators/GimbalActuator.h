#pragma once

#include <math/Vector3.h>

namespace aeroforge::actuators
{
	struct GimbalAngles
	{
		double pitchRad;
		double yawRad;
	};

	class GimbalActuator
	{
	public:
		GimbalActuator(double maxAngleRad, double maxRateRadPerS);

		GimbalAngles update(double commandedPitchRad, double commandedYawRad, double dtS);

		const GimbalAngles& getAngles() const;

		math::Vector3 getThrustDirectionB() const;

	private:
		double maxAngleRad;
		double maxRateRadPerS;

		GimbalAngles angles;
	};
}