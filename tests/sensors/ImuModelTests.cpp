#include <gtest/gtest.h>

#include <sensors/AccelerometerModel.h>
#include <sensors/GyroscopeModel.h>
#include <sensors/ImuModel.h>

#include <math/Vector3.h>

#include <stdexcept>

namespace aeroforge::sensors::tests
{
	TEST(
		ImuModelTests,
		CombinesAccelerometerAndGyroscope)
	{
		AccelerometerModel accelerometer(
			math::Vector3(0.1, 0.0, 0.0),
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(100.0, 100.0, 100.0),
			1);

		GyroscopeModel gyroscope(
			math::Vector3(0.0, 0.2, 0.0),
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(20.0, 20.0, 20.0),
			1);

		ImuModel imu(
			accelerometer,
			gyroscope);

		const ImuMeasurement measurement =
			imu.sample(
				1.5,
				math::Vector3(1.0, 2.0, 3.0),
				math::Vector3(0.1, 0.2, 0.3));

		EXPECT_DOUBLE_EQ(
			measurement.timeS,
			1.5);

		EXPECT_NEAR(
			measurement.specificForceB.x,
			1.1,
			1e-12);

		EXPECT_NEAR(
			measurement.angularVelocityB.y,
			0.4,
			1e-12);
	}

	TEST(
		ImuModelTests,
		RejectsInvalidTimestamp)
	{
		AccelerometerModel accelerometer(
			math::Vector3(),
			math::Vector3(),
			math::Vector3(100.0, 100.0, 100.0),
			1);

		GyroscopeModel gyroscope(
			math::Vector3(),
			math::Vector3(),
			math::Vector3(20.0, 20.0, 20.0),
			1);

		ImuModel imu(
			accelerometer,
			gyroscope);

		EXPECT_THROW(
			imu.sample(
				-1.0,
				math::Vector3(),
				math::Vector3()),
			std::invalid_argument);
	}
}