#include <gtest/gtest.h>

#include <dynamics/MomentAccumulator.h>

#include <math/Vector3.h>

#include <limits>
#include <stdexcept>

namespace aeroforge::dynamics::tests
{
	TEST(
		MomentAccumulatorTests,
		StartsAtZero)
	{
		const MomentAccumulator moments;

		EXPECT_DOUBLE_EQ(
			moments.getTotalMomentB().x,
			0.0);

		EXPECT_DOUBLE_EQ(
			moments.getTotalMomentB().y,
			0.0);

		EXPECT_DOUBLE_EQ(
			moments.getTotalMomentB().z,
			0.0);
	}

	TEST(
		MomentAccumulatorTests,
		AddsSingleMoment)
	{
		MomentAccumulator moments;

		moments.addMoment(
			math::Vector3(
				1.0,
				2.0,
				3.0));

		EXPECT_DOUBLE_EQ(
			moments.getTotalMomentB().x,
			1.0);

		EXPECT_DOUBLE_EQ(
			moments.getTotalMomentB().y,
			2.0);

		EXPECT_DOUBLE_EQ(
			moments.getTotalMomentB().z,
			3.0);
	}

	TEST(
		MomentAccumulatorTests,
		AccumulatesMultipleMoments)
	{
		MomentAccumulator moments;

		moments.addMoment(
			math::Vector3(
				1.0,
				2.0,
				3.0));

		moments.addMoment(
			math::Vector3(
				4.0,
				-1.0,
				2.0));

		EXPECT_DOUBLE_EQ(
			moments.getTotalMomentB().x,
			5.0);

		EXPECT_DOUBLE_EQ(
			moments.getTotalMomentB().y,
			1.0);

		EXPECT_DOUBLE_EQ(
			moments.getTotalMomentB().z,
			5.0);
	}

	TEST(
		MomentAccumulatorTests,
		ClearResetsMoment)
	{
		MomentAccumulator moments;

		moments.addMoment(
			math::Vector3(
				1.0,
				2.0,
				3.0));

		moments.clear();

		EXPECT_DOUBLE_EQ(
			moments.getTotalMomentB().x,
			0.0);

		EXPECT_DOUBLE_EQ(
			moments.getTotalMomentB().y,
			0.0);

		EXPECT_DOUBLE_EQ(
			moments.getTotalMomentB().z,
			0.0);
	}

	TEST(
		MomentAccumulatorTests,
		RejectsNonFiniteMoment)
	{
		MomentAccumulator moments;

		EXPECT_THROW(
			moments.addMoment(
				math::Vector3(
					std::numeric_limits<double>::
					infinity(),
					0.0,
					0.0)),
			std::invalid_argument);
	}

	TEST(
		MomentAccumulatorTests,
		RejectsAccumulationOverflow)
	{
		MomentAccumulator moments;

		moments.addMoment(
			math::Vector3(
				std::numeric_limits<double>::max(),
				0.0,
				0.0));

		EXPECT_THROW(
			moments.addMoment(
				math::Vector3(
					std::numeric_limits<double>::max(),
					0.0,
					0.0)),
			std::overflow_error);
	}

	TEST(
		MomentAccumulatorTests,
		FailedAdditionDoesNotModifyExistingMoment)
	{
		MomentAccumulator moments;

		moments.addMoment(
			math::Vector3(
				10.0,
				20.0,
				30.0));

		EXPECT_THROW(
			moments.addMoment(
				math::Vector3(
					std::numeric_limits<double>::
					infinity(),
					0.0,
					0.0)),
			std::invalid_argument);

		EXPECT_DOUBLE_EQ(
			moments.getTotalMomentB().x,
			10.0);
	}
}