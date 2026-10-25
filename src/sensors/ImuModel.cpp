#include <sensors/ImuModel.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::sensors
{
	ImuModel::ImuModel(const AccelerometerModel& accelerometer, const GyroscopeModel& gyroscope)
		: accelerometer(accelerometer), gyroscope(gyroscope)
	{

	}

	ImuMeasurement ImuModel::sample(
		double timeS,
		const math::Vector3& trueSpecificForceB,
		const math::Vector3& trueAngularVelocityB
	)
	{
		if (!std::isfinite(timeS) || timeS < 0)
		{
			throw std::invalid_argument("timeS must be finite and non-negative");
		}


		math::Vector3 accelerometerMeasure = accelerometer.measure(trueSpecificForceB);
		math::Vector3 gyroscopeMeasure = gyroscope.measure(trueAngularVelocityB);

		return ImuMeasurement{timeS, accelerometerMeasure, gyroscopeMeasure};
	}
}