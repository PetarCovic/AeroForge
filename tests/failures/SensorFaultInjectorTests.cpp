#include <gtest/gtest.h>

#include <failures/FaultSchedule.h>
#include <failures/SensorFaultInjector.h>

#include <math/Vector3.h>

#include <limits>
#include <optional>
#include <stdexcept>

namespace aeroforge::failures
{
	TEST(SensorFaultInjectorTests, PassesMeasurementThroughBeforeFault)
	{
		SensorFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			SensorFaultType::Bias,
			math::Vector3(1.0, 2.0, 3.0)
		);

		math::Vector3 input(10.0, 20.0, 30.0);

		auto result = injector.apply(1.0, input);

		ASSERT_TRUE(result.has_value());

		EXPECT_DOUBLE_EQ(result->x, 10.0);
		EXPECT_DOUBLE_EQ(result->y, 20.0);
		EXPECT_DOUBLE_EQ(result->z, 30.0);
	}

	TEST(SensorFaultInjectorTests, AppliesBiasDuringFaultWindow)
	{
		SensorFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			SensorFaultType::Bias,
			math::Vector3(1.0, -2.0, 0.5)
		);

		math::Vector3 input(10.0, 20.0, 30.0);

		auto result = injector.apply(3.0, input);

		ASSERT_TRUE(result.has_value());

		EXPECT_DOUBLE_EQ(result->x, 11.0);
		EXPECT_DOUBLE_EQ(result->y, 18.0);
		EXPECT_DOUBLE_EQ(result->z, 30.5);
	}

	TEST(SensorFaultInjectorTests, StopsApplyingBiasAfterFaultWindow)
	{
		SensorFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			SensorFaultType::Bias,
			math::Vector3(1.0, 2.0, 3.0)
		);

		math::Vector3 input(10.0, 20.0, 30.0);

		auto result = injector.apply(5.0, input);

		ASSERT_TRUE(result.has_value());

		EXPECT_DOUBLE_EQ(result->x, 10.0);
		EXPECT_DOUBLE_EQ(result->y, 20.0);
		EXPECT_DOUBLE_EQ(result->z, 30.0);
	}

	TEST(SensorFaultInjectorTests, StuckAtCapturesFirstActiveMeasurement)
	{
		SensorFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			SensorFaultType::StuckAt,
			math::Vector3(0.0, 0.0, 0.0)
		);

		auto first = injector.apply(
			2.0,
			math::Vector3(10.0, 20.0, 30.0)
		);

		ASSERT_TRUE(first.has_value());

		EXPECT_DOUBLE_EQ(first->x, 10.0);
		EXPECT_DOUBLE_EQ(first->y, 20.0);
		EXPECT_DOUBLE_EQ(first->z, 30.0);

		auto second = injector.apply(
			3.0,
			math::Vector3(100.0, 200.0, 300.0)
		);

		ASSERT_TRUE(second.has_value());

		EXPECT_DOUBLE_EQ(second->x, 10.0);
		EXPECT_DOUBLE_EQ(second->y, 20.0);
		EXPECT_DOUBLE_EQ(second->z, 30.0);
	}

	TEST(SensorFaultInjectorTests, StuckAtDoesNotRemainStuckAfterFaultEnds)
	{
		SensorFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			SensorFaultType::StuckAt,
			math::Vector3(0.0, 0.0, 0.0)
		);

		injector.apply(
			2.0,
			math::Vector3(10.0, 20.0, 30.0)
		);

		auto result = injector.apply(
			5.0,
			math::Vector3(100.0, 200.0, 300.0)
		);

		ASSERT_TRUE(result.has_value());

		EXPECT_DOUBLE_EQ(result->x, 100.0);
		EXPECT_DOUBLE_EQ(result->y, 200.0);
		EXPECT_DOUBLE_EQ(result->z, 300.0);
	}

	TEST(SensorFaultInjectorTests, DropoutReturnsNoMeasurementDuringFault)
	{
		SensorFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			SensorFaultType::Dropout,
			math::Vector3(0.0, 0.0, 0.0)
		);

		auto result = injector.apply(
			3.0,
			math::Vector3(1.0, 2.0, 3.0)
		);

		EXPECT_FALSE(result.has_value());
	}

	TEST(SensorFaultInjectorTests, DropoutPassesMeasurementOutsideFault)
	{
		SensorFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			SensorFaultType::Dropout,
			math::Vector3(0.0, 0.0, 0.0)
		);

		auto result = injector.apply(
			1.0,
			math::Vector3(1.0, 2.0, 3.0)
		);

		ASSERT_TRUE(result.has_value());

		EXPECT_DOUBLE_EQ(result->x, 1.0);
		EXPECT_DOUBLE_EQ(result->y, 2.0);
		EXPECT_DOUBLE_EQ(result->z, 3.0);
	}

	TEST(SensorFaultInjectorTests, RejectsNonFiniteFaultParameter)
	{
		EXPECT_THROW(
			SensorFaultInjector(
				FaultSchedule(2.0, 5.0),
				SensorFaultType::Bias,
				math::Vector3(
					std::numeric_limits<double>::quiet_NaN(),
					0.0,
					0.0
				)
			),
			std::invalid_argument
		);
	}

	TEST(SensorFaultInjectorTests, RejectsNonFiniteMeasurement)
	{
		SensorFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			SensorFaultType::Bias,
			math::Vector3(1.0, 0.0, 0.0)
		);

		EXPECT_THROW(
			injector.apply(
				3.0,
				math::Vector3(
					std::numeric_limits<double>::infinity(),
					0.0,
					0.0
				)
			),
			std::invalid_argument
		);
	}

	TEST(SensorFaultInjectorTests, RejectsInvalidTime)
	{
		SensorFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			SensorFaultType::Bias,
			math::Vector3(0.0, 0.0, 0.0)
		);

		EXPECT_THROW(
			injector.apply(
				-1.0,
				math::Vector3(0.0, 0.0, 0.0)
			),
			std::invalid_argument
		);

		EXPECT_THROW(
			injector.apply(
				std::numeric_limits<double>::quiet_NaN(),
				math::Vector3(0.0, 0.0, 0.0)
			),
			std::invalid_argument
		);
	}
}