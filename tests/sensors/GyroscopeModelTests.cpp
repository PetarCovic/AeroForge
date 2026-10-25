#include <gtest/gtest.h>

#include <sensors/GyroscopeModel.h>

#include <math/Vector3.h>

#include <stdexcept>

namespace aeroforge::sensors::tests
{
	TEST(
		GyroscopeModelTests,
		PerfectSensorReturnsTrueAngularVelocity)
	{
		GyroscopeModel sensor(
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(20.0, 20.0, 20.0),
			1234);

		const math::Vector3 measured =
			sensor.measure(
				math::Vector3(1.0, 2.0, 3.0));

		EXPECT_DOUBLE_EQ(measured.x, 1.0);
		EXPECT_DOUBLE_EQ(measured.y, 2.0);
		EXPECT_DOUBLE_EQ(measured.z, 3.0);
	}

	TEST(
		GyroscopeModelTests,
		AddsBias)
	{
		GyroscopeModel sensor(
			math::Vector3(0.01, -0.02, 0.03),
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(20.0, 20.0, 20.0),
			1234);

		const math::Vector3 measured =
			sensor.measure(
				math::Vector3(1.0, 2.0, 3.0));

		EXPECT_NEAR(measured.x, 1.01, 1e-12);
		EXPECT_NEAR(measured.y, 1.98, 1e-12);
		EXPECT_NEAR(measured.z, 3.03, 1e-12);
	}

	TEST(
		GyroscopeModelTests,
		SaturatesMeasurement)
	{
		GyroscopeModel sensor(
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(5.0, 5.0, 5.0),
			1234);

		const math::Vector3 measured =
			sensor.measure(
				math::Vector3(8.0, -10.0, 2.0));

		EXPECT_DOUBLE_EQ(measured.x, 5.0);
		EXPECT_DOUBLE_EQ(measured.y, -5.0);
		EXPECT_DOUBLE_EQ(measured.z, 2.0);
	}

	TEST(
		GyroscopeModelTests,
		SameSeedIsDeterministic)
	{
		GyroscopeModel first(
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(0.01, 0.01, 0.01),
			math::Vector3(20.0, 20.0, 20.0),
			99);

		GyroscopeModel second(
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(0.01, 0.01, 0.01),
			math::Vector3(20.0, 20.0, 20.0),
			99);

		for (int i = 0; i < 10; ++i)
		{
			const math::Vector3 a =
				first.measure(
					math::Vector3(1.0, 2.0, 3.0));

			const math::Vector3 b =
				second.measure(
					math::Vector3(1.0, 2.0, 3.0));

			EXPECT_DOUBLE_EQ(a.x, b.x);
			EXPECT_DOUBLE_EQ(a.y, b.y);
			EXPECT_DOUBLE_EQ(a.z, b.z);
		}
	}

	TEST(
		GyroscopeModelTests,
		RejectsInvalidConfiguration)
	{
		EXPECT_THROW(
			(GyroscopeModel(
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3(0.0, -1.0, 0.0),
				math::Vector3(10.0, 10.0, 10.0),
				1)),
			std::invalid_argument);
	}
}