#include <gtest/gtest.h>

#include <environment/RelativeAirVelocity.h>

#include <limits>
#include <stdexcept>

namespace aeroforge::environment::tests
{
	TEST(
		RelativeAirVelocityTests,
		ZeroWindReturnsVehicleVelocity)
	{
		const math::Vector3 vehicleVelocity(
			100.0,
			20.0,
			-10.0);

		const math::Vector3 windVelocity(
			0.0,
			0.0,
			0.0);

		const math::Vector3 relative =
			computeRelativeAirVelocityN(
				vehicleVelocity,
				windVelocity);

		EXPECT_DOUBLE_EQ(relative.x, 100.0);
		EXPECT_DOUBLE_EQ(relative.y, 20.0);
		EXPECT_DOUBLE_EQ(relative.z, -10.0);
	}

	TEST(
		RelativeAirVelocityTests,
		TailwindReducesRelativeAirspeed)
	{
		const math::Vector3 vehicleVelocity(
			100.0,
			0.0,
			0.0);

		const math::Vector3 windVelocity(
			20.0,
			0.0,
			0.0);

		const math::Vector3 relative =
			computeRelativeAirVelocityN(
				vehicleVelocity,
				windVelocity);

		EXPECT_DOUBLE_EQ(relative.x, 80.0);
		EXPECT_DOUBLE_EQ(relative.y, 0.0);
		EXPECT_DOUBLE_EQ(relative.z, 0.0);
	}

	TEST(
		RelativeAirVelocityTests,
		HeadwindIncreasesRelativeAirspeed)
	{
		const math::Vector3 vehicleVelocity(
			100.0,
			0.0,
			0.0);

		const math::Vector3 windVelocity(
			-20.0,
			0.0,
			0.0);

		const math::Vector3 relative =
			computeRelativeAirVelocityN(
				vehicleVelocity,
				windVelocity);

		EXPECT_DOUBLE_EQ(relative.x, 120.0);
	}

	TEST(
		RelativeAirVelocityTests,
		CrosswindProducesOppositeRelativeComponent)
	{
		const math::Vector3 vehicleVelocity(
			100.0,
			0.0,
			0.0);

		const math::Vector3 windVelocity(
			0.0,
			20.0,
			0.0);

		const math::Vector3 relative =
			computeRelativeAirVelocityN(
				vehicleVelocity,
				windVelocity);

		EXPECT_DOUBLE_EQ(relative.x, 100.0);
		EXPECT_DOUBLE_EQ(relative.y, -20.0);
		EXPECT_DOUBLE_EQ(relative.z, 0.0);
	}

	TEST(
		RelativeAirVelocityTests,
		VerticalWindUsesNEDSignConvention)
	{
		const math::Vector3 vehicleVelocity(
			0.0,
			0.0,
			20.0);

		const math::Vector3 windVelocity(
			0.0,
			0.0,
			5.0);

		const math::Vector3 relative =
			computeRelativeAirVelocityN(
				vehicleVelocity,
				windVelocity);

		EXPECT_DOUBLE_EQ(relative.z, 15.0);
	}

	TEST(
		RelativeAirVelocityTests,
		StationaryVehicleInWindHasOppositeRelativeVelocity)
	{
		const math::Vector3 vehicleVelocity(
			0.0,
			0.0,
			0.0);

		const math::Vector3 windVelocity(
			10.0,
			-5.0,
			2.0);

		const math::Vector3 relative =
			computeRelativeAirVelocityN(
				vehicleVelocity,
				windVelocity);

		EXPECT_DOUBLE_EQ(relative.x, -10.0);
		EXPECT_DOUBLE_EQ(relative.y, 5.0);
		EXPECT_DOUBLE_EQ(relative.z, -2.0);
	}

	TEST(
		RelativeAirVelocityTests,
		RejectsNonFiniteVehicleVelocity)
	{
		const math::Vector3 vehicleVelocity(
			std::numeric_limits<double>::
			quiet_NaN(),
			0.0,
			0.0);

		const math::Vector3 windVelocity(
			0.0,
			0.0,
			0.0);

		EXPECT_THROW(
			computeRelativeAirVelocityN(
				vehicleVelocity,
				windVelocity),
			std::invalid_argument);
	}

	TEST(
		RelativeAirVelocityTests,
		RejectsNonFiniteWindVelocity)
	{
		const math::Vector3 vehicleVelocity(
			0.0,
			0.0,
			0.0);

		const math::Vector3 windVelocity(
			0.0,
			std::numeric_limits<double>::
			infinity(),
			0.0);

		EXPECT_THROW(
			computeRelativeAirVelocityN(
				vehicleVelocity,
				windVelocity),
			std::invalid_argument);
	}

	TEST(
		RelativeAirVelocityTests,
		RejectsNonFiniteComputedResult)
	{
		const math::Vector3 vehicleVelocity(
			std::numeric_limits<double>::max(),
			0.0,
			0.0);

		const math::Vector3 windVelocity(
			-std::numeric_limits<double>::max(),
			0.0,
			0.0);

		EXPECT_THROW(
			computeRelativeAirVelocityN(
				vehicleVelocity,
				windVelocity),
			std::overflow_error);
	}
}