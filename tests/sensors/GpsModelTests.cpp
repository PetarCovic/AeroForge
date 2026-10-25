#include <gtest/gtest.h>

#include <sensors/GpsModel.h>

#include <math/Vector3.h>

#include <stdexcept>

namespace aeroforge::sensors::tests
{
	TEST(
		GpsModelTests,
		PerfectGpsReturnsTruth)
	{
		GpsModel gps(
			math::Vector3(),
			math::Vector3(),
			math::Vector3(),
			math::Vector3(),
			1);

		const GpsMeasurement measurement =
			gps.sample(
				3.0,
				math::Vector3(100.0, 200.0, -50.0),
				math::Vector3(10.0, 5.0, -2.0));

		EXPECT_DOUBLE_EQ(
			measurement.timeS,
			3.0);

		EXPECT_DOUBLE_EQ(
			measurement.positionN.x,
			100.0);

		EXPECT_DOUBLE_EQ(
			measurement.velocityN.z,
			-2.0);
	}

	TEST(
		GpsModelTests,
		AddsPositionAndVelocityBias)
	{
		GpsModel gps(
			math::Vector3(1.0, 2.0, 3.0),
			math::Vector3(),
			math::Vector3(0.1, 0.2, 0.3),
			math::Vector3(),
			1);

		const GpsMeasurement measurement =
			gps.sample(
				0.0,
				math::Vector3(10.0, 20.0, 30.0),
				math::Vector3(1.0, 2.0, 3.0));

		EXPECT_NEAR(
			measurement.positionN.x,
			11.0,
			1e-12);

		EXPECT_NEAR(
			measurement.positionN.y,
			22.0,
			1e-12);

		EXPECT_NEAR(
			measurement.velocityN.z,
			3.3,
			1e-12);
	}

	TEST(
		GpsModelTests,
		SameSeedProducesSameMeasurements)
	{
		GpsModel first(
			math::Vector3(),
			math::Vector3(1.0, 1.0, 1.0),
			math::Vector3(),
			math::Vector3(0.1, 0.1, 0.1),
			777);

		GpsModel second(
			math::Vector3(),
			math::Vector3(1.0, 1.0, 1.0),
			math::Vector3(),
			math::Vector3(0.1, 0.1, 0.1),
			777);

		for (int i = 0; i < 5; ++i)
		{
			const GpsMeasurement a =
				first.sample(
					static_cast<double>(i),
					math::Vector3(),
					math::Vector3());

			const GpsMeasurement b =
				second.sample(
					static_cast<double>(i),
					math::Vector3(),
					math::Vector3());

			EXPECT_DOUBLE_EQ(
				a.positionN.x,
				b.positionN.x);

			EXPECT_DOUBLE_EQ(
				a.velocityN.y,
				b.velocityN.y);
		}
	}

	TEST(
		GpsModelTests,
		RejectsNegativeNoiseStandardDeviation)
	{
		EXPECT_THROW(
			(GpsModel(
				math::Vector3(),
				math::Vector3(-1.0, 0.0, 0.0),
				math::Vector3(),
				math::Vector3(),
				1)),
			std::invalid_argument);
	}

	TEST(
		GpsModelTests,
		RejectsInvalidTimestamp)
	{
		GpsModel gps(
			math::Vector3(),
			math::Vector3(),
			math::Vector3(),
			math::Vector3(),
			1);

		EXPECT_THROW(
			gps.sample(
				-0.01,
				math::Vector3(),
				math::Vector3()),
			std::invalid_argument);
	}
}