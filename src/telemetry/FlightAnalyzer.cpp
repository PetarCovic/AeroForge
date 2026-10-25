#include <telemetry/FlightAnalyzer.h>

#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace aeroforge::telemetry
{
	FlightMetrics FlightAnalyzer::analyze(const std::vector<TelemetrySample>& samples)
	{
		if (samples.empty())
		{
			throw std::invalid_argument("samples cannot be empty");
		}

		FlightMetrics metric;
		metric.sampleCount = samples.size();
		metric.durationS = samples.back().timeS - samples.front().timeS;
		metric.maxAltitudeM = -samples[0].positionN.z;
		
		double x = samples[0].velocityN.x;
		double y = samples[0].velocityN.y;
		double z = samples[0].velocityN.z;
		metric.maxSpeedMps = std::sqrt(x * x + y * y + z * z);

		double wx = samples[0].angularVelocityB.x;
		double wy = samples[0].angularVelocityB.y;
		double wz = samples[0].angularVelocityB.z;
		metric.maxAngularRateRadS = std::sqrt(wx * wx + wy * wy + wz * wz);

		metric.maxAccelerationMps2 = 0.0;

		std::size_t count = 0;
		for (const TelemetrySample& sample : samples)
		{
			double altitude = -sample.positionN.z;
			metric.maxAltitudeM = std::max(metric.maxAltitudeM, altitude);

			x = sample.velocityN.x;
			y = sample.velocityN.y;
			z = sample.velocityN.z;
			double speed = std::sqrt(x * x + y * y + z * z);
			metric.maxSpeedMps = std::max(metric.maxSpeedMps, speed);

			wx = sample.angularVelocityB.x;
			wy = sample.angularVelocityB.y;
			wz = sample.angularVelocityB.z;
			double angularVelocity = std::sqrt(wx * wx + wy * wy + wz * wz);
			metric.maxAngularRateRadS = std::max(metric.maxAngularRateRadS, angularVelocity);

			if (count >= 1)
			{
				double dt = samples[count].timeS-samples[count-1].timeS;

				if (dt <= 0)
				{
					throw std::invalid_argument("dt must be positive");
				}

				math::Vector3 deltaVelocity= samples[count].velocityN - samples[count - 1].velocityN;

				double accelerationMagnitude = deltaVelocity.magnitude() / dt;
				metric.maxAccelerationMps2 = std::max(metric.maxAccelerationMps2, accelerationMagnitude);
			}

			count++;
		}

		metric.finalPositionN = samples.back().positionN;
		metric.finalVelocityN = samples.back().velocityN;

		return metric;
	}
}