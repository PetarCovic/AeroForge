#pragma once

#include <math/Vector3.h>
#include <cstddef>

namespace aeroforge::telemetry
{
	struct FlightMetrics
	{
		std::size_t sampleCount;
		double durationS;
		double maxAltitudeM;
		double maxSpeedMps;
		double maxAccelerationMps2;
		double maxAngularRateRadS;
		math::Vector3 finalPositionN;
		math::Vector3 finalVelocityN;
	};
}