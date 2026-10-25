#include <gtest/gtest.h>

#include <sensors/AccelerometerModel.h>

#include <math/Vector3.h>

#include <cstdint>
#include <limits>
#include <stdexcept>

namespace aeroforge::sensors::tests
{
	TEST(
		AccelerometerModelTests,
		PerfectSensorReturnsTrueSpecificForce)
	{
		AccelerometerModel sensor(
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(100.0, 100.0, 100.0),
			1234);

		const math::Vector3 measured =
			sensor.measure(
				math::Vector3(1.0, 2.0, 3.0));

		EXPECT_DOUBLE_EQ(measured.x, 1.0);
		EXPECT_DOUBLE_EQ(measured.y, 2.0);
		EXPECT_DOUBLE_EQ(measured.z, 3.0);
	}

	TEST(
		AccelerometerModelTests,
		AddsConfiguredBias)
	{
		AccelerometerModel sensor(
			math::Vector3(0.1, -0.2, 0.3),
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(100.0, 100.0, 100.0),
			1234);

		const math::Vector3 measured =
			sensor.measure(
				math::Vector3(1.0, 2.0, 3.0));

		EXPECT_NEAR(measured.x, 1.1, 1e-12);
		EXPECT_NEAR(measured.y, 1.8, 1e-12);
		EXPECT_NEAR(measured.z, 3.3, 1e-12);
	}

	TEST(
		AccelerometerModelTests,
		SaturatesEachAxis)
	{
		AccelerometerModel sensor(
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(10.0, 20.0, 30.0),
			1234);

		const math::Vector3 measured =
			sensor.measure(
				math::Vector3(100.0, -100.0, 25.0));

		EXPECT_DOUBLE_EQ(measured.x, 10.0);
		EXPECT_DOUBLE_EQ(measured.y, -20.0);
		EXPECT_DOUBLE_EQ(measured.z, 25.0);
	}

	TEST(
		AccelerometerModelTests,
		SameSeedProducesSameNoiseSequence)
	{
		AccelerometerModel first(
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(0.1, 0.2, 0.3),
			math::Vector3(100.0, 100.0, 100.0),
			42);

		AccelerometerModel second(
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(0.1, 0.2, 0.3),
			math::Vector3(100.0, 100.0, 100.0),
			42);

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
		AccelerometerModelTests,
		RejectsNegativeNoiseStandardDeviation)
	{
		EXPECT_THROW(
			(AccelerometerModel(
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3(-0.1, 0.0, 0.0),
				math::Vector3(100.0, 100.0, 100.0),
				1)),
			std::invalid_argument);
	}

	TEST(
		AccelerometerModelTests,
		RejectsNonPositiveSaturation)
	{
		EXPECT_THROW(
			(AccelerometerModel(
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3(0.0, 100.0, 100.0),
				1)),
			std::invalid_argument);
	}

	TEST(
		AccelerometerModelTests,
		RejectsNonFiniteTruth)
	{
		AccelerometerModel sensor(
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(100.0, 100.0, 100.0),
			1);

		EXPECT_THROW(
			sensor.measure(
				math::Vector3(
					std::numeric_limits<double>::infinity(),
					0.0,
					0.0)),
			std::invalid_argument);
	}
}