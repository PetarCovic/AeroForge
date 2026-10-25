#include <gtest/gtest.h>

#include <environment/ConstantWindModel.h>
#include <environment/WindModel.h>

#include <limits>
#include <stdexcept>
#include <type_traits>

namespace aeroforge::environment::tests
{
	TEST(
		ConstantWindModelTests,
		ImplementsWindModel)
	{
		EXPECT_TRUE(
			(std::is_base_of_v<
				WindModel,
				ConstantWindModel>));
	}

	TEST(
		ConstantWindModelTests,
		DefaultWindIsZero)
	{
		const ConstantWindModel windModel;

		const math::Vector3 wind =
			windModel.getWindVelocityN(
				0.0,
				0.0);

		EXPECT_DOUBLE_EQ(wind.x, 0.0);
		EXPECT_DOUBLE_EQ(wind.y, 0.0);
		EXPECT_DOUBLE_EQ(wind.z, 0.0);
	}

	TEST(
		ConstantWindModelTests,
		ReturnsConfiguredNEDWind)
	{
		const ConstantWindModel windModel(
			math::Vector3(
				12.0,
				-4.0,
				2.0));

		const math::Vector3 wind =
			windModel.getWindVelocityN(
				1000.0,
				25.0);

		EXPECT_DOUBLE_EQ(wind.x, 12.0);
		EXPECT_DOUBLE_EQ(wind.y, -4.0);
		EXPECT_DOUBLE_EQ(wind.z, 2.0);
	}

	TEST(
		ConstantWindModelTests,
		IsIndependentOfAltitude)
	{
		const ConstantWindModel windModel(
			math::Vector3(
				15.0,
				5.0,
				-1.0));

		const math::Vector3 low =
			windModel.getWindVelocityN(
				0.0,
				0.0);

		const math::Vector3 high =
			windModel.getWindVelocityN(
				15000.0,
				0.0);

		EXPECT_DOUBLE_EQ(low.x, high.x);
		EXPECT_DOUBLE_EQ(low.y, high.y);
		EXPECT_DOUBLE_EQ(low.z, high.z);
	}

	TEST(
		ConstantWindModelTests,
		IsIndependentOfTime)
	{
		const ConstantWindModel windModel(
			math::Vector3(
				15.0,
				5.0,
				-1.0));

		const math::Vector3 first =
			windModel.getWindVelocityN(
				1000.0,
				0.0);

		const math::Vector3 later =
			windModel.getWindVelocityN(
				1000.0,
				1000.0);

		EXPECT_DOUBLE_EQ(first.x, later.x);
		EXPECT_DOUBLE_EQ(first.y, later.y);
		EXPECT_DOUBLE_EQ(first.z, later.z);
	}

	TEST(
		ConstantWindModelTests,
		PreservesNEDDirections)
	{
		const ConstantWindModel windModel(
			math::Vector3(
				20.0,
				-10.0,
				-5.0));

		const math::Vector3 wind =
			windModel.getWindVelocityN(
				5000.0,
				10.0);

		// +X = North
		EXPECT_GT(wind.x, 0.0);

		// -Y = West
		EXPECT_LT(wind.y, 0.0);

		// -Z = Up
		EXPECT_LT(wind.z, 0.0);
	}

	TEST(
		ConstantWindModelTests,
		RejectsNaNConfiguredWind)
	{
		const double nan =
			std::numeric_limits<double>::quiet_NaN();

		EXPECT_THROW(
			ConstantWindModel(
				math::Vector3(
					nan,
					0.0,
					0.0)),
			std::invalid_argument);
	}

	TEST(
		ConstantWindModelTests,
		RejectsInfiniteConfiguredWind)
	{
		const double infinity =
			std::numeric_limits<double>::infinity();

		EXPECT_THROW(
			ConstantWindModel(
				math::Vector3(
					0.0,
					infinity,
					0.0)),
			std::invalid_argument);
	}
}