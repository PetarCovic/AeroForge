#pragma once

namespace aeroforge::actuators
{
	class ThrottleActuator
	{
	public:
		ThrottleActuator(double initialThrottle, double maxRatePerS);

		double update(double commandedThrottle, double dtS);

		double getThrottle() const;

	private:
		double throttle;
		double maxRatePerS;
	};
}