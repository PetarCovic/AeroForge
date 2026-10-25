#pragma once

#include <math/Vector3.h>
#include <sensors/AccelerometerModel.h>
#include <sensors/GyroscopeModel.h>

namespace aeroforge::sensors
{
	struct ImuMeasurement
	{
		double timeS;
		math::Vector3 specificForceB;
		math::Vector3 angularVelocityB;
	};

	class ImuModel
	{
	public:
		ImuModel(const AccelerometerModel& accelerometer, const GyroscopeModel& gyroscope);

		ImuMeasurement sample(
			double timeS,
			const math::Vector3& trueSpecificForceB,
			const math::Vector3& trueAngularVelocityB
		);

	private:
		AccelerometerModel accelerometer;
		GyroscopeModel gyroscope;
	};
}