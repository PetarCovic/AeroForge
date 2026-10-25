#include <gtest/gtest.h>

#include <failures/FaultSchedule.h>
#include <failures/GpsFaultInjector.h>

#include <sensors/GpsModel.h>

#include <math/Vector3.h>

#include <limits>
#include <optional>
#include <stdexcept>

namespace aeroforge::failures
{
	TEST(GpsFaultInjectorTests, PassesGpsMeasurementThroughOutsideFault)
	{
		GpsFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			GpsFaultType::Bias,
			math::Vector3(100.0, 0.0, 0.0),
			math::Vector3(10.0, 0.0, 0.0)
		);

		sensors::GpsMeasurement measurement{
			1.0,
			math::Vector3(1.0, 2.0, 3.0),
			math::Vector3(4.0, 5.0, 6.0)
		};

		auto result = injector.apply(measurement);

		ASSERT_TRUE(result.has_value());

		EXPECT_DOUBLE_EQ(result->timeS, 1.0);

		EXPECT_DOUBLE_EQ(result->positionN.x, 1.0);
		EXPECT_DOUBLE_EQ(result->positionN.y, 2.0);
		EXPECT_DOUBLE_EQ(result->positionN.z, 3.0);

		EXPECT_DOUBLE_EQ(result->velocityN.x, 4.0);
		EXPECT_DOUBLE_EQ(result->velocityN.y, 5.0);
		EXPECT_DOUBLE_EQ(result->velocityN.z, 6.0);
	}

	TEST(GpsFaultInjectorTests, AppliesPositionAndVelocityBiasDuringFault)
	{
		GpsFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			GpsFaultType::Bias,
			math::Vector3(100.0, -10.0, 1.0),
			math::Vector3(0.5, -0.25, 2.0)
		);

		sensors::GpsMeasurement measurement{
			3.0,
			math::Vector3(10.0, 20.0, 30.0),
			math::Vector3(1.0, 2.0, 3.0)
		};

		auto result = injector.apply(measurement);

		ASSERT_TRUE(result.has_value());

		EXPECT_DOUBLE_EQ(result->timeS, 3.0);

		EXPECT_DOUBLE_EQ(result->positionN.x, 110.0);
		EXPECT_DOUBLE_EQ(result->positionN.y, 10.0);
		EXPECT_DOUBLE_EQ(result->positionN.z, 31.0);

		EXPECT_DOUBLE_EQ(result->velocityN.x, 1.5);
		EXPECT_DOUBLE_EQ(result->velocityN.y, 1.75);
		EXPECT_DOUBLE_EQ(result->velocityN.z, 5.0);
	}

	TEST(GpsFaultInjectorTests, StuckAtFreezesFirstActiveGpsMeasurement)
	{
		GpsFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			GpsFaultType::StuckAt,
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(0.0, 0.0, 0.0)
		);

		sensors::GpsMeasurement firstMeasurement{
			2.0,
			math::Vector3(10.0, 20.0, 30.0),
			math::Vector3(1.0, 2.0, 3.0)
		};

		auto first = injector.apply(firstMeasurement);

		ASSERT_TRUE(first.has_value());

		sensors::GpsMeasurement laterMeasurement{
			3.0,
			math::Vector3(100.0, 200.0, 300.0),
			math::Vector3(10.0, 20.0, 30.0)
		};

		auto second = injector.apply(laterMeasurement);

		ASSERT_TRUE(second.has_value());

		// A stuck GPS repeats the stale measurement,
		// including its original timestamp.
		EXPECT_DOUBLE_EQ(second->timeS, 2.0);

		EXPECT_DOUBLE_EQ(second->positionN.x, 10.0);
		EXPECT_DOUBLE_EQ(second->positionN.y, 20.0);
		EXPECT_DOUBLE_EQ(second->positionN.z, 30.0);

		EXPECT_DOUBLE_EQ(second->velocityN.x, 1.0);
		EXPECT_DOUBLE_EQ(second->velocityN.y, 2.0);
		EXPECT_DOUBLE_EQ(second->velocityN.z, 3.0);
	}

	TEST(GpsFaultInjectorTests, StuckAtReleasesAfterFaultEnds)
	{
		GpsFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			GpsFaultType::StuckAt,
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(0.0, 0.0, 0.0)
		);

		injector.apply(
			sensors::GpsMeasurement{
				2.0,
				math::Vector3(10.0, 20.0, 30.0),
				math::Vector3(1.0, 2.0, 3.0)
			}
		);

		auto result = injector.apply(
			sensors::GpsMeasurement{
				5.0,
				math::Vector3(100.0, 200.0, 300.0),
				math::Vector3(10.0, 20.0, 30.0)
			}
		);

		ASSERT_TRUE(result.has_value());

		EXPECT_DOUBLE_EQ(result->timeS, 5.0);
		EXPECT_DOUBLE_EQ(result->positionN.x, 100.0);
		EXPECT_DOUBLE_EQ(result->velocityN.x, 10.0);
	}

	TEST(GpsFaultInjectorTests, DropoutReturnsNoMeasurementDuringFault)
	{
		GpsFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			GpsFaultType::Dropout,
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(0.0, 0.0, 0.0)
		);

		auto result = injector.apply(
			sensors::GpsMeasurement{
				3.0,
				math::Vector3(1.0, 2.0, 3.0),
				math::Vector3(4.0, 5.0, 6.0)
			}
		);

		EXPECT_FALSE(result.has_value());
	}

	TEST(GpsFaultInjectorTests, DropoutRecoversAfterFaultEnds)
	{
		GpsFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			GpsFaultType::Dropout,
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(0.0, 0.0, 0.0)
		);

		EXPECT_FALSE(
			injector.apply(
				sensors::GpsMeasurement{
					3.0,
					math::Vector3(),
					math::Vector3()
				}
			).has_value()
		);

		auto recovered = injector.apply(
			sensors::GpsMeasurement{
				5.0,
				math::Vector3(10.0, 0.0, 0.0),
				math::Vector3(1.0, 0.0, 0.0)
			}
		);

		ASSERT_TRUE(recovered.has_value());

		EXPECT_DOUBLE_EQ(recovered->positionN.x, 10.0);
		EXPECT_DOUBLE_EQ(recovered->velocityN.x, 1.0);
	}

	TEST(GpsFaultInjectorTests, RejectsNonFiniteFaultParameters)
	{
		EXPECT_THROW(
			GpsFaultInjector(
				FaultSchedule(2.0, 5.0),
				GpsFaultType::Bias,
				math::Vector3(
					std::numeric_limits<double>::quiet_NaN(),
					0.0,
					0.0
				),
				math::Vector3(0.0, 0.0, 0.0)
			),
			std::invalid_argument
		);

		EXPECT_THROW(
			GpsFaultInjector(
				FaultSchedule(2.0, 5.0),
				GpsFaultType::Bias,
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3(
					std::numeric_limits<double>::infinity(),
					0.0,
					0.0
				)
			),
			std::invalid_argument
		);
	}

	TEST(GpsFaultInjectorTests, RejectsInvalidMeasurementTime)
	{
		GpsFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			GpsFaultType::Bias,
			math::Vector3(),
			math::Vector3()
		);

		EXPECT_THROW(
			injector.apply(
				sensors::GpsMeasurement{
					-1.0,
					math::Vector3(),
					math::Vector3()
				}
			),
			std::invalid_argument
		);
	}

	TEST(GpsFaultInjectorTests, RejectsNonFiniteMeasurementPosition)
	{
		GpsFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			GpsFaultType::Bias,
			math::Vector3(),
			math::Vector3()
		);

		EXPECT_THROW(
			injector.apply(
				sensors::GpsMeasurement{
					3.0,
					math::Vector3(
						std::numeric_limits<double>::quiet_NaN(),
						0.0,
						0.0
					),
					math::Vector3()
				}
			),
			std::invalid_argument
		);
	}

	TEST(GpsFaultInjectorTests, RejectsNonFiniteMeasurementVelocity)
	{
		GpsFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			GpsFaultType::Bias,
			math::Vector3(),
			math::Vector3()
		);

		EXPECT_THROW(
			injector.apply(
				sensors::GpsMeasurement{
					3.0,
					math::Vector3(),
					math::Vector3(
						0.0,
						std::numeric_limits<double>::infinity(),
						0.0
					)
				}
			),
			std::invalid_argument
		);
	}
}