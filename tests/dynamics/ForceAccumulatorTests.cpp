#include <gtest/gtest.h>

#include <dynamics/ForceAccumulator.h>

#include <limits>
#include <stdexcept>

namespace aeroforge::dynamics::tests
{
	TEST(
		ForceAccumulatorTests,
		DefaultConstructionStartsAtZeroForce)
	{
		const ForceAccumulator accumulator;

		EXPECT_DOUBLE_EQ(
			accumulator.getTotalForce().x,
			0.0);

		EXPECT_DOUBLE_EQ(
			accumulator.getTotalForce().y,
			0.0);

		EXPECT_DOUBLE_EQ(
			accumulator.getTotalForce().z,
			0.0);
	}

	TEST(
		ForceAccumulatorTests,
		AddsSingleForce)
	{
		ForceAccumulator accumulator;

		accumulator.addForce(
			math::Vector3(
				10.0,
				-5.0,
				3.0));

		const math::Vector3 expected(
			10.0,
			-5.0,
			3.0);

		EXPECT_TRUE(
			accumulator.getTotalForce()
			.isApprox(
				expected,
				1e-12));
	}

	TEST(
		ForceAccumulatorTests,
		AccumulatesMultipleForces)
	{
		ForceAccumulator accumulator;

		accumulator.addForce(
			math::Vector3(
				10.0,
				0.0,
				5.0));

		accumulator.addForce(
			math::Vector3(
				-3.0,
				4.0,
				2.0));

		accumulator.addForce(
			math::Vector3(
				1.0,
				-2.0,
				-1.0));

		const math::Vector3 expected(
			8.0,
			2.0,
			6.0);

		EXPECT_TRUE(
			accumulator.getTotalForce()
			.isApprox(
				expected,
				1e-12));
	}

	TEST(
		ForceAccumulatorTests,
		OpposingForcesCancel)
	{
		ForceAccumulator accumulator;

		accumulator.addForce(
			math::Vector3(
				100.0,
				-50.0,
				25.0));

		accumulator.addForce(
			math::Vector3(
				-100.0,
				50.0,
				-25.0));

		const math::Vector3 expected(
			0.0,
			0.0,
			0.0);

		EXPECT_TRUE(
			accumulator.getTotalForce()
			.isApprox(
				expected,
				1e-12));
	}

	TEST(
		ForceAccumulatorTests,
		PreservesNEDComponents)
	{
		ForceAccumulator accumulator;

		accumulator.addForce(
			math::Vector3(
				25.0,
				-10.0,
				98.1));

		const math::Vector3& total =
			accumulator.getTotalForce();

		EXPECT_DOUBLE_EQ(
			total.x,
			25.0);

		EXPECT_DOUBLE_EQ(
			total.y,
			-10.0);

		EXPECT_DOUBLE_EQ(
			total.z,
			98.1);
	}

	TEST(
		ForceAccumulatorTests,
		PositiveZRepresentsDownwardForce)
	{
		ForceAccumulator accumulator;

		accumulator.addForce(
			math::Vector3(
				0.0,
				0.0,
				100.0));

		EXPECT_DOUBLE_EQ(
			accumulator.getTotalForce().z,
			100.0);
	}

	TEST(
		ForceAccumulatorTests,
		NegativeZRepresentsUpwardForce)
	{
		ForceAccumulator accumulator;

		accumulator.addForce(
			math::Vector3(
				0.0,
				0.0,
				-100.0));

		EXPECT_DOUBLE_EQ(
			accumulator.getTotalForce().z,
			-100.0);
	}

	TEST(
		ForceAccumulatorTests,
		ClearRestoresZeroForce)
	{
		ForceAccumulator accumulator;

		accumulator.addForce(
			math::Vector3(
				10.0,
				20.0,
				30.0));

		accumulator.clear();

		const math::Vector3 expected(
			0.0,
			0.0,
			0.0);

		EXPECT_TRUE(
			accumulator.getTotalForce()
			.isApprox(
				expected,
				1e-12));
	}

	TEST(
		ForceAccumulatorTests,
		CanAccumulateAgainAfterClear)
	{
		ForceAccumulator accumulator;

		accumulator.addForce(
			math::Vector3(
				10.0,
				20.0,
				30.0));

		accumulator.clear();

		accumulator.addForce(
			math::Vector3(
				1.0,
				2.0,
				3.0));

		const math::Vector3 expected(
			1.0,
			2.0,
			3.0);

		EXPECT_TRUE(
			accumulator.getTotalForce()
			.isApprox(
				expected,
				1e-12));
	}

	TEST(
		ForceAccumulatorTests,
		RejectsNaNForce)
	{
		ForceAccumulator accumulator;

		const math::Vector3 force(
			std::numeric_limits<double>::quiet_NaN(),
			0.0,
			0.0);

		EXPECT_THROW(
			accumulator.addForce(force),
			std::invalid_argument);
	}

	TEST(
		ForceAccumulatorTests,
		RejectsPositiveInfinityForce)
	{
		ForceAccumulator accumulator;

		const math::Vector3 force(
			0.0,
			std::numeric_limits<double>::infinity(),
			0.0);

		EXPECT_THROW(
			accumulator.addForce(force),
			std::invalid_argument);
	}

	TEST(
		ForceAccumulatorTests,
		RejectsNegativeInfinityForce)
	{
		ForceAccumulator accumulator;

		const math::Vector3 force(
			0.0,
			0.0,
			-std::numeric_limits<double>::infinity());

		EXPECT_THROW(
			accumulator.addForce(force),
			std::invalid_argument);
	}

	TEST(
		ForceAccumulatorTests,
		InvalidForceDoesNotModifyAccumulator)
	{
		ForceAccumulator accumulator;

		accumulator.addForce(
			math::Vector3(
				1.0,
				2.0,
				3.0));

		const math::Vector3 before =
			accumulator.getTotalForce();

		EXPECT_THROW(
			accumulator.addForce(
				math::Vector3(
					std::numeric_limits<double>::quiet_NaN(),
					5.0,
					6.0)),
			std::invalid_argument);

		EXPECT_TRUE(
			accumulator.getTotalForce()
			.isApprox(
				before,
				1e-12));
	}

	TEST(
		ForceAccumulatorTests,
		RejectsOverflowingTotalForce)
	{
		ForceAccumulator accumulator;

		const double maximum =
			std::numeric_limits<double>::max();

		accumulator.addForce(
			math::Vector3(
				maximum,
				0.0,
				0.0));

		EXPECT_THROW(
			accumulator.addForce(
				math::Vector3(
					maximum,
					0.0,
					0.0)),
			std::overflow_error);
	}

	TEST(
		ForceAccumulatorTests,
		OverflowFailureDoesNotModifyAccumulator)
	{
		ForceAccumulator accumulator;

		const double maximum =
			std::numeric_limits<double>::max();

		accumulator.addForce(
			math::Vector3(
				maximum,
				5.0,
				10.0));

		const math::Vector3 before =
			accumulator.getTotalForce();

		EXPECT_THROW(
			accumulator.addForce(
				math::Vector3(
					maximum,
					1.0,
					1.0)),
			std::overflow_error);

		EXPECT_DOUBLE_EQ(
			accumulator.getTotalForce().x,
			before.x);

		EXPECT_DOUBLE_EQ(
			accumulator.getTotalForce().y,
			before.y);

		EXPECT_DOUBLE_EQ(
			accumulator.getTotalForce().z,
			before.z);
	}

	TEST(
		ForceAccumulatorTests,
		AddingZeroForceDoesNotChangeTotal)
	{
		ForceAccumulator accumulator;

		accumulator.addForce(
			math::Vector3(
				4.0,
				5.0,
				6.0));

		const math::Vector3 before =
			accumulator.getTotalForce();

		accumulator.addForce(
			math::Vector3(
				0.0,
				0.0,
				0.0));

		EXPECT_TRUE(
			accumulator.getTotalForce()
			.isApprox(
				before,
				1e-12));
	}

	TEST(
		ForceAccumulatorTests,
		LargeNumberOfFiniteForcesAccumulateCorrectly)
	{
		ForceAccumulator accumulator;

		constexpr int COUNT =
			10000;

		for (int i = 0;
			i < COUNT;
			++i)
		{
			accumulator.addForce(
				math::Vector3(
					0.1,
					-0.2,
					0.3));
		}

		EXPECT_NEAR(
			accumulator.getTotalForce().x,
			1000.0,
			1e-9);

		EXPECT_NEAR(
			accumulator.getTotalForce().y,
			-2000.0,
			1e-9);

		EXPECT_NEAR(
			accumulator.getTotalForce().z,
			3000.0,
			1e-9);
	}
}