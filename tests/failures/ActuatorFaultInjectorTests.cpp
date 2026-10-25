#include <gtest/gtest.h>

#include <failures/FaultSchedule.h>
#include <failures/ActuatorFaultInjector.h>

#include <math/Vector3.h>

#include <limits>
#include <stdexcept>

namespace aeroforge::failures
{
	TEST(ActuatorFaultInjectorTests, PassesOutputThroughOutsideFaultWindow)
	{
		ActuatorFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			ActuatorFaultType::LossOfEffectiveness,
			math::Vector3(0.5, 0.5, 0.5)
		);

		math::Vector3 result = injector.apply(
			1.0,
			math::Vector3(10.0, 20.0, 30.0)
		);

		EXPECT_DOUBLE_EQ(result.x, 10.0);
		EXPECT_DOUBLE_EQ(result.y, 20.0);
		EXPECT_DOUBLE_EQ(result.z, 30.0);
	}

	TEST(ActuatorFaultInjectorTests, AppliesLossOfEffectivenessPerAxis)
	{
		ActuatorFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			ActuatorFaultType::LossOfEffectiveness,
			math::Vector3(0.5, 0.0, 1.0)
		);

		math::Vector3 result = injector.apply(
			3.0,
			math::Vector3(10.0, -20.0, 30.0)
		);

		EXPECT_DOUBLE_EQ(result.x, 5.0);
		EXPECT_DOUBLE_EQ(result.y, 0.0);
		EXPECT_DOUBLE_EQ(result.z, 30.0);
	}

	TEST(ActuatorFaultInjectorTests, AllowsCompleteLossOfAuthority)
	{
		ActuatorFaultInjector injector(
			FaultSchedule(0.0, 5.0),
			ActuatorFaultType::LossOfEffectiveness,
			math::Vector3(0.0, 0.0, 0.0)
		);

		math::Vector3 result = injector.apply(
			1.0,
			math::Vector3(10.0, -20.0, 30.0)
		);

		EXPECT_DOUBLE_EQ(result.x, 0.0);
		EXPECT_DOUBLE_EQ(result.y, 0.0);
		EXPECT_DOUBLE_EQ(result.z, 0.0);
	}

	TEST(ActuatorFaultInjectorTests, StuckAtCapturesFirstActiveOutput)
	{
		ActuatorFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			ActuatorFaultType::StuckAt,
			math::Vector3()
		);

		math::Vector3 first = injector.apply(
			2.0,
			math::Vector3(1.0, 2.0, 3.0)
		);

		EXPECT_DOUBLE_EQ(first.x, 1.0);
		EXPECT_DOUBLE_EQ(first.y, 2.0);
		EXPECT_DOUBLE_EQ(first.z, 3.0);

		math::Vector3 second = injector.apply(
			3.0,
			math::Vector3(10.0, 20.0, 30.0)
		);

		EXPECT_DOUBLE_EQ(second.x, 1.0);
		EXPECT_DOUBLE_EQ(second.y, 2.0);
		EXPECT_DOUBLE_EQ(second.z, 3.0);
	}

	TEST(ActuatorFaultInjectorTests, StuckAtReleasesAfterFaultEnds)
	{
		ActuatorFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			ActuatorFaultType::StuckAt,
			math::Vector3()
		);

		injector.apply(
			2.0,
			math::Vector3(1.0, 2.0, 3.0)
		);

		math::Vector3 result = injector.apply(
			5.0,
			math::Vector3(10.0, 20.0, 30.0)
		);

		EXPECT_DOUBLE_EQ(result.x, 10.0);
		EXPECT_DOUBLE_EQ(result.y, 20.0);
		EXPECT_DOUBLE_EQ(result.z, 30.0);
	}

	TEST(ActuatorFaultInjectorTests, HardOverReturnsConfiguredOutput)
	{
		ActuatorFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			ActuatorFaultType::HardOver,
			math::Vector3(-10.0, 20.0, 30.0)
		);

		math::Vector3 result = injector.apply(
			3.0,
			math::Vector3(1.0, 2.0, 3.0)
		);

		EXPECT_DOUBLE_EQ(result.x, -10.0);
		EXPECT_DOUBLE_EQ(result.y, 20.0);
		EXPECT_DOUBLE_EQ(result.z, 30.0);
	}

	TEST(ActuatorFaultInjectorTests, HardOverEndsWhenFaultWindowEnds)
	{
		ActuatorFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			ActuatorFaultType::HardOver,
			math::Vector3(-10.0, 20.0, 30.0)
		);

		math::Vector3 result = injector.apply(
			5.0,
			math::Vector3(1.0, 2.0, 3.0)
		);

		EXPECT_DOUBLE_EQ(result.x, 1.0);
		EXPECT_DOUBLE_EQ(result.y, 2.0);
		EXPECT_DOUBLE_EQ(result.z, 3.0);
	}

	TEST(ActuatorFaultInjectorTests, RejectsNegativeEffectiveness)
	{
		EXPECT_THROW(
			ActuatorFaultInjector(
				FaultSchedule(2.0, 5.0),
				ActuatorFaultType::LossOfEffectiveness,
				math::Vector3(1.0, -0.1, 1.0)
			),
			std::invalid_argument
		);
	}

	TEST(ActuatorFaultInjectorTests, RejectsEffectivenessAboveOne)
	{
		EXPECT_THROW(
			ActuatorFaultInjector(
				FaultSchedule(2.0, 5.0),
				ActuatorFaultType::LossOfEffectiveness,
				math::Vector3(1.0, 1.1, 1.0)
			),
			std::invalid_argument
		);
	}

	TEST(ActuatorFaultInjectorTests, RejectsNonFiniteFaultParameter)
	{
		EXPECT_THROW(
			ActuatorFaultInjector(
				FaultSchedule(2.0, 5.0),
				ActuatorFaultType::HardOver,
				math::Vector3(
					std::numeric_limits<double>::infinity(),
					0.0,
					0.0
				)
			),
			std::invalid_argument
		);
	}

	TEST(ActuatorFaultInjectorTests, RejectsNonFiniteNominalOutput)
	{
		ActuatorFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			ActuatorFaultType::HardOver,
			math::Vector3(10.0, 0.0, 0.0)
		);

		EXPECT_THROW(
			injector.apply(
				3.0,
				math::Vector3(
					std::numeric_limits<double>::quiet_NaN(),
					0.0,
					0.0
				)
			),
			std::invalid_argument
		);
	}

	TEST(ActuatorFaultInjectorTests, RejectsInvalidTime)
	{
		ActuatorFaultInjector injector(
			FaultSchedule(2.0, 5.0),
			ActuatorFaultType::HardOver,
			math::Vector3(10.0, 0.0, 0.0)
		);

		EXPECT_THROW(
			injector.apply(
				-1.0,
				math::Vector3()
			),
			std::invalid_argument
		);

		EXPECT_THROW(
			injector.apply(
				std::numeric_limits<double>::quiet_NaN(),
				math::Vector3()
			),
			std::invalid_argument
		);
	}
}